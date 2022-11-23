# Delivery Mobile Robot
This Package is created for My Differential Drive Mobile Robot for Indoor Delivery. It is for ROS Melodic.[For Mom!]

### Hardware

|Hardware|Alternative|Link|Comment|
|--------|-----------|----|-------|
|1 X Jetson Nano|||Running ROS|
|1 X Arduino Due|||Control DC Motor|
|1 X Custom Motor Diver|IBT-2||This driver is custom made. It has the same control method as IBT-2|
|1 X Custom Logic Level Converter|SparkFun Logic Level Converter|[Link](https://www.sparkfun.com/products/12009)|Use for shifting 3v and 5v power in Arduino Due|
|1 X IMU BNO055||[Link](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)|Running ROS|
|2 X Gear DC Motor|||with Encoder|
|1 X Lidar||||
|1 X Camera||||
|2 X Lipo Battery|||1 X 12v for all electronic, 1 X 24v for motor driver only|

### Mobile Robot Parameter

|Parameter|Value|
|---------|-----|
|Width|...m|
|Length|...m|
|Height|...m|
|Weight|...kg|
|Base Length|0.44 m|
|Wheel Radius|0.07 m|
|Wheel Base to Caster Wheel|...m|
|Ground Clearance|...m|

## Connection

### Reference
- Arduino Due Board Installed on IDE : https://forum.arduino.cc/t/installing-arduino-sam-boards-on-arduino-ide-for-arm64/550398
- Grid Mapping : https://github.com/salihmarangoz/robot_laser_grid_mapping
- Laser Simulation from Image File : https://github.com/salihmarangoz/robot_laser_simulator