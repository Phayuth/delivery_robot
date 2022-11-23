#define USE_USBCON // For Arduino Due rosserial in every board and computer
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

/* Set the delay between fresh samples */
//uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 myIMU = Adafruit_BNO055(55, 0x28);

sensor_msgs::Imu imu_data;
ros::Publisher imupub("imu", &imu_data);
geometry_msgs::Quaternion q;

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

void setup() {

  myIMU.begin();
  delay(1000);
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);

  // ROS Setup
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(imupub);
}

void loop() {
  // Record the time
  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    uint8_t system, gyro, accel, mg = 0;
    myIMU.getCalibration(&system, &gyro, &accel, &mg);
    imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> rpy = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    //populate data to msg
    //imu_data.header.seq = 0;
    //imu_data.header.stamp = 0;
    imu_data.header.frame_id = "imu_link";
    
    q = tf::createQuaternionFromYaw(rpy.z());
    
    imu_data.orientation.x = q.x;
    imu_data.orientation.y = q.y;
    imu_data.orientation.z = q.z;
    imu_data.orientation.w = q.w;

    imu_data.angular_velocity.x = gyr.x();
    imu_data.angular_velocity.y = gyr.y();
    imu_data.angular_velocity.z = gyr.z();

    imu_data.linear_acceleration.x = acc.x();
    imu_data.linear_acceleration.y = acc.y();
    imu_data.linear_acceleration.z = acc.z();

    //publish data
    imupub.publish( &imu_data );
    nh.spinOnce();
  }

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}
