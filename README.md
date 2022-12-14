<img align="right" src="dlvr_resource/img.jpg" width="240">

# Delivery Mobile Robot
This Package is created for My Differential Drive Mobile Robot for Indoor Delivery. It is for ROS Melodic.[For Mom!]

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

### Mobile Robot Parameter

| Parameter | Value | Parameter                  | Value  | Motor Left | Value     | Motor Right | Value     |
|-----------|-------|----------------------------|--------|------------|-----------|-------------|-----------|
| Width     | ...m  | Base Length                | 0.44 m | a          | 26.63     | a           | 28.37     |
| Length    | ...m  | Wheel Radius               | 0.07 m | b          | 17.26     | b           | 19.15     |
| Height    | ...m  | Wheel Base to Caster Wheel | ...m   | c          | 6.776     | c           | 5.75      |
| Weight    | ...kg | Ground Clearance           | ...m   | GR         | 19.2x3.75 | GR          | 19.2x3.75 |
|           |       |                            |        | PPR        | 28        | PPR         | 28        |


### Reference
- Arduino Due Board Installed on IDE : https://forum.arduino.cc/t/installing-arduino-sam-boards-on-arduino-ide-for-arm64/550398
- Grid Mapping : https://github.com/salihmarangoz/robot_laser_grid_mapping
- Laser Simulation from Image File : https://github.com/salihmarangoz/robot_laser_simulator
