#!/usr/bin/env python

from PIL import Image 
import numpy as np
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
import tf_conversions
from tf import TransformBroadcaster


class Simulator:
    def __init__(self, filename, resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist):
        img = Image.open(filename)
        img = img.convert('1') # to gray image
        self.gridmap = 1.0 - np.asarray(img)
        self.gridmap = np.around(self.gridmap)
        self.resolution = resolution   # map resolution
        self.laser_min_angle = laser_min_angle    #degree
        self.laser_max_angle = laser_max_angle    #degree
        self.laser_resolution = laser_resolution  #degree
        self.laser_max_dist = laser_max_dist      #meter
        self.robot_x = 0.0                        #meter
        self.robot_y = 0.0                        #meter
        self.robot_theta = 0.0                    #radian

    def to_xy (self, i, j):
        x = j * self.resolution
        y = (self.gridmap.shape[0] - i) * self.resolution
        return x, y

    def to_ij (self, x, y):
        i = self.gridmap.shape[0] - (y / self.resolution)
        j = x / self.resolution
        return i, j

    def is_inside (self, i, j):
        return i<self.gridmap.shape[0] and j<self.gridmap.shape[1] and i>=0 and j>=0

    def get_borders(self):
        return [0.0, self.gridmap.shape[1] * self.resolution, 0.0, self.gridmap.shape[0] * self.resolution]

    def set_robot_pos(self, x, y, theta):
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

    def get_measurements(self, debug=False):
        laser_data = []
        for i in np.arange(self.laser_min_angle, self.laser_max_angle+self.laser_resolution, self.laser_resolution):
            xp, yp, is_hit = self.raycast(self.robot_x, self.robot_y, np.radians(i) + self.robot_theta, self.laser_max_dist, debug)
            if is_hit:
                laser_data.append(np.sqrt((xp-self.robot_x)**2+(yp-self.robot_y)**2))
            else:
                # if goes beyond max dist return max dist or nan
                #laser_data.append(self.laser_max_dist)
                laser_data.append(float('nan'))
        return np.array(laser_data)

    def raycast(self, x0, y0, theta, max_dist, debug=False):    #x0, y0, max_dist in meters; theta in radian;  debug is for visulizations
        x1 = x0 + max_dist*np.cos(theta)
        y1 = y0 + max_dist*np.sin(theta)
        i0, j0 = self.to_ij(x0, y0)
        i1, j1 = self.to_ij(x1, y1)
        max_dist_cells = max_dist / self.resolution
        ip, jp, is_hit = self.bresenham(i0, j0, i1, j1, max_dist_cells, debug)
        xp, yp = self.to_xy(ip, jp)
        return xp, yp, is_hit
    
    #bresenham method is used to plot the lines
    def bresenham (self, i0, j0, i1, j1, max_dist_cells, debug=False):   # i0, j0 (starting point)
        dx = np.absolute(j1-j0)
        dy = -1 * np.absolute(i1-i0)
        sx = -1
        if j0<j1:
            sx = 1
        sy = -1
        if i0<i1:
            sy = 1
        jp, ip = j0, i0
        err = dx+dy                     # error value e_xy
        while True:                     # loop
            if (jp == j1 and ip == i1) or (np.sqrt((jp-j0)**2+(ip-i0)**2) >= max_dist_cells) or not self.is_inside(ip, jp):  
                return ip, jp, False
            elif self.gridmap[int(ip)][int(jp)]==1:
                return ip, jp, True
            if debug:
                self.gridmap[int(ip)][int(jp)] = 0.5
            e2 = 2*err
            if e2 >= dy:                # e_xy+e_x > 0 
                err += dy
                jp += sx
            if e2 <= dx:                # e_xy+e_y < 0
                err += dx
                ip += sy



class SimulatorROS:
    def __init__(self):
        # OTHER GLOBAL VARIABLES
        self.cmdvel_queue = []
        self.sim_time = 0.0

        # ROS PARAMETERS
        rospy.init_node('RosSimulator', anonymous=True)
        self.map_file = rospy.get_param('~map_file', 'map.png')
        self.map_resolution = rospy.get_param('~map_resolution', 0.05)
        self.time_resolution = rospy.get_param('~time_resolution', 0.1) # dt
        self.laser_min_angle = rospy.get_param('~laser_min_angle', -135)
        self.laser_max_angle = rospy.get_param('~laser_max_angle', 135)
        self.laser_resolution = rospy.get_param('~laser_resolution', 1)
        self.laser_noise_mu = rospy.get_param('~laser_noise_mu', 0.1)
        self.laser_noise_sigma = rospy.get_param('~laser_noise_sigma', 0.02)
        self.laser_max_dist = rospy.get_param('~laser_max_dist', 15.0)
        self.robot_pos_x = rospy.get_param('~robot_pos_x', 0.0)
        self.robot_pos_y = rospy.get_param('~robot_pos_y', 0.0)
        self.robot_pos_theta = rospy.get_param('~robot_pos_theta', 0)
        self.odom_frame = rospy.get_param('~odom_frame', "")
        self.robot_frame = rospy.get_param('~robot_frame', "base_link")
        self.laser_frame = rospy.get_param('~laser_frame', "laser_link")

        # SCAN MESSAGE TEMPLATE
        self.scan = LaserScan()
        self.scan.header.frame_id = self.laser_frame
        self.scan.angle_min = np.radians(self.laser_min_angle)
        self.scan.angle_max = np.radians(self.laser_max_angle)
        self.scan.angle_increment = np.radians(self.laser_resolution)
        self.scan.range_max = self.laser_max_dist

        # INIT SIMULATOR
        self.simulator = Simulator(self.map_file, self.map_resolution, self.laser_min_angle, self.laser_max_angle, self.laser_resolution, self.laser_max_dist)
        self.simulator.set_robot_pos(self.robot_pos_x, self.robot_pos_y, np.radians(self.robot_pos_theta)) # robot start point

        # INIT ROS PUBLISHER/SUBSCRIBERS
        self.laser_pub = rospy.Publisher('/scan', LaserScan, queue_size=2)
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmdvel_callback)
        self.time_pub = rospy.Publisher('/clock', Clock, queue_size=2)
        self.tf_pub = TransformBroadcaster()

    def publish_laserscan(self, data):
        data += np.random.normal(self.laser_noise_mu, self.laser_noise_sigma, data.shape) # apply gaussian noise
        self.scan.header.stamp = rospy.Time.from_sec(self.sim_time)
        self.scan.ranges = data
        self.laser_pub.publish(self.scan)

    def publish_time(self, t):
        ros_t = rospy.Time.from_sec(t)
        self.time_pub.publish(ros_t)

    # NOTE: laser_frame -> robot_frame transform is published by static_transform_publisher node in launch file
    def publish_odom(self, x, y, theta):
        if self.odom_frame != "":
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
            self.tf_pub.sendTransform((x,y,0.0), q, rospy.Time.from_sec(self.sim_time), self.robot_frame, self.odom_frame)

    def process_cmdvel(self):
        if len(self.cmdvel_queue)>0:
            dt = self.time_resolution / len(self.cmdvel_queue) # approximated that time diff between commands have the same period
            for i in self.cmdvel_queue:
                self.simulator.robot_theta += i['ang'] * dt
                self.simulator.robot_theta = (self.simulator.robot_theta + 2 * np.pi) % (2 * np.pi) 
                self.simulator.robot_x += i['lin'] * np.cos(self.simulator.robot_theta) * dt
                self.simulator.robot_y += i['lin'] * np.sin(self.simulator.robot_theta) * dt
            self.cmdvel_queue = []

    def cmdvel_callback(self, data):
        self.cmdvel_queue.append({'lin': data.linear.x, 'ang': data.angular.z})

    def tick(self):
            self.process_cmdvel()
            self.publish_laserscan(self.simulator.get_measurements()) # also applies noise
            self.publish_odom(self.simulator.robot_x, self.simulator.robot_y, self.simulator.robot_theta)
            self.publish_time(self.sim_time)
            self.sim_time += self.time_resolution

    def spin(self):
        while not rospy.is_shutdown():
            start_t = time.time()
            self.tick()
            elapsed_t = time.time() - start_t
            remaining_t = self.time_resolution - elapsed_t
            if (remaining_t > 0):
                time.sleep(remaining_t)


sim = SimulatorROS()
sim.spin()
