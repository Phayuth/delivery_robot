### Hardware
- 1 X Jetson Nano
- 1 X Arduino Due
- 1 X Custom Motor DiverIBT-2
- 1 X Custom Logic Level Converter
- 1 X IMU BNO055
- 2 X Gear DC Motor
- 1 X Lidar
- 1 X Camera
- 2 X Lipo Battery

### Install Prerequired Package
```
sudo apt-get install libgeographic-dev ros-melodic-libgeographic-info ros-melodic-libgeographic-dev
sudo apt-get install ros-melodic-openslam-gmapping ros-melodic-tf2-sensor-msgs
sudo apt-get install ros-melodic-laser-proc ros-melodic-urg-c
sudo apt-get install qt4-qmake qt4-dev-tools
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
### Install Delivery Robot Package
-- To Do --

### Mobile Robot Parameter

| Params               | Value     |
|----------------------|-----------|
| Width                | ...m      |
| Length               | ...m      |
| Height               | ...m      |
| Weight               | ...kg     |
| Base Length          | 0.44 m    |
| Wheel Radius         | 0.07 m    |
| Base to Caster Wheel | ...m      |
| Ground Clearance     | ...m      |
| Motor Left a         | 26.63     |
| Motor Left b         | 17.26     |
| Motor Left c         | 6.776     |
| Motor Left GR        | 19.2x3.75 |
| Motor Left PPR       | 28        |
| Motor Right a        | 28.37     |
| Motor Right b        | 19.15     |
| Motor Right c        | 5.75      |
| Motor Right GR       | 19.2x3.75 |
| Motor Right PPR      | 28        |

### Support Repo
- [Hokuyo Lidar Package urg_node](https://github.com/ros-drivers/urg_node)
- [BNO055 Package](https://github.com/dheera/ros-imu-bno055)
- [Robot localization](https://github.com/cra-ros-pkg/robot_localization)
- [Gmapping](https://github.com/ros-perception/slam_gmapping)
- [Hector Slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam)
- [ROS Navigation](http://wiki.ros.org/navigation)
- [ROS Serial](http://wiki.ros.org/rosserial)

### Other Repo
- [Grid Mapping](https://github.com/salihmarangoz/robot_laser_grid_mapping) - do a minimal laser grid mapping in python and ros
- [Laser Simulation from Image File](https://github.com/salihmarangoz/robot_laser_simulator) - create a simulation of laser scan from image file
- [Python for Robotic by AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics)

### Resoures and Issue
- [How to setup ros nav stack by automaticaddision](https://automaticaddison.com/how-to-set-up-the-ros-navigation-stack-on-a-robot/)
- [Arduino Due Board Installed on IDE](https://forum.arduino.cc/t/installing-arduino-sam-boards-on-arduino-ide-for-arm64/550398) - fix issue when installing arduino due board on nvidia jetson device
- [Why Odom frame and Map frame move away from each other](https://answers.ros.org/question/10302/the-tf-between-map-and-odom-is-not-static/#15185)
- [Install Hector SLAM](https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/)
- [Max range map](https://answers.ros.org/question/138591/gmapping-in-gazebo-with-hokuyo-problems-due-to-max-range-measurements/)
- [Pioneer 3pdx](https://www.generationrobots.com/media/Pioneer3DX-P3DX-RevA.pdf)