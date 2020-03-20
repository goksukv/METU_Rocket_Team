# METU_Rocket_Team

In this repository I share my codes with METU Rocket Team. I am happy to help this hard working team on some snesor and control issues.

For now I uploaded only an Arduino Code to read imu raw data from altimu 10 v5 and publish it as conventinal ROS message and topic. 

### How to use;

1. Uploade arduino code to arduino by using Arduino IDE or Atmel Studio.

2. Start Ros core bu using Linux Terminal;

   `roscore`

3. Start Rosserial Node by using Linux Terminal

   ` rosrun rosserial_python serial_node.py /dev/ttyACM0`

   if you have boudrate errors try 

   `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600`

4. Listen published data from terminal

   `rosopic echo /imu/imu_raw`

   `rostopic echo /imu/mag`

5. If everything is working well run any IMU filter to get oriantation. I personally use Madgwick Filter;

   `rosrun imu_filter_madgwick imu_filter_node`

6. Finally To visualize The data ;

   `rosrun rviz rviz`# TUBerlin_Intern
