#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

// ROS Integraion Libraries
#include <ros.h>
#include <std_msgs/String.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

//ROS Initialise
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg ;
ros::Publisher imu_raw("imu/data_raw", &imu_msg);
ros::Publisher mag_raw("imu/mag", &mag_msg);
ros::NodeHandle nh;


LSM6 imu;
LIS3MDL mag;
LSM6::vector<int16_t> avrg_acc = {0, 0, 0},  avrg_gyro = {0, 0, 0};
LIS3MDL::vector<int16_t> running_min_mag = {32767, 32767, 32767}, running_max_mag = { -32768, -32768, -32768};


void setup()
{


  /*Arduino Confi*/
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);



  /* Fail Codes*/
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }


  /* Enable*/
  imu.enableDefault();
  mag.enableDefault();


  /* Magnetor meter Calibrate */
  acc_gyro_calibrate();
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay (200);
  }
  delay(200);
  mag_calibrate();


  /*ROS*/
  nh.initNode();
  nh.advertise(imu_raw);
  nh.advertise(mag_raw);

}

void loop()
{
  /*read*/
  imu.read();
  imu_msg.header.stamp = nh.now();
  mag.read();
  mag_msg.header.stamp = imu_msg.header.stamp;

  imu_msg.header.frame_id = "base_link";
  mag_msg.header.frame_id = imu_msg.header.frame_id;


  imu_msg.orientation_covariance[0] = -1;
  imu_msg.orientation_covariance[4] = -1;
  imu_msg.orientation_covariance[8] = -1;
  // Convert And Publish  
  imu_msg.linear_acceleration.x = (float)(imu.a.x - avrg_acc.x) * 0.00059820565 ;//(0.000061 * 9.80665)
  imu_msg.linear_acceleration.y = (float)(imu.a.y - avrg_acc.y) * 0.00059820565 ;//(0.000061 * 9.80665)
  imu_msg.linear_acceleration.z = (float)(imu.a.z - avrg_acc.z) * 0.00059820565 ;//(0.000061 * 9.80665)
  //imu_msg.angular_velocity_covariance[0] = 0.000000015;
  //imu_msg.angular_velocity_covariance[4] = 0.000000015;
  //imu_msg.angular_velocity_covariance[8] = 0.000000015;

  imu_msg.angular_velocity.x = (float)(imu.g.x - avrg_gyro.x) * -0.000152716;//(8.75 * 0.0174532925 * 0.001)
  imu_msg.angular_velocity.y = (float)(imu.g.y - avrg_gyro.y) * -0.000152716;//(8.75 * 0.0174532925 * 0.001)
  imu_msg.angular_velocity.z = (float)(imu.g.z - avrg_gyro.z) * -0.000152716;//(8.75 * 0.0174532925 * 0.001)
  //imu_msg.linear_acceleration_covariance[0] = 0.000000015;
  //imu_msg.linear_acceleration_covariance[4] = 0.000000015;
  //imu_msg.linear_acceleration_covariance[8] = 0.000000015;

  mag_msg.magnetic_field.x = (float)mag.m.x * 0.0000000146156; //(0.0001 / 6842)
  mag_msg.magnetic_field.y = (float)mag.m.y * 0.0000000146156;
  mag_msg.magnetic_field.z = (float)mag.m.z * 0.0000000146156;
  //mag_msg.magnetic_field_covariance[0] = 0.000000015;
  //mag_msg.magnetic_field_covariance[4] = 0.000000015;
  //mag_msg.magnetic_field_covariance[8] = 0.000000015;

  mag_raw.publish( &mag_msg );
  imu_raw.publish( &imu_msg );
  nh.spinOnce();
  delay(10);
}


void acc_gyro_calibrate()
{

  double starttime, endtime;
  starttime = millis();
  endtime = starttime;
  int counter=0;
  while ((endtime - starttime) <= 5000) // do this loop for up to 1000mS
  {
    imu.read();

    avrg_acc.x += imu.a.x;
    avrg_acc.y += imu.a.y;
    avrg_acc.z += imu.a.z;

    avrg_gyro.x += imu.g.x;
    avrg_gyro.y += imu.g.y;
    avrg_gyro.z += imu.g.z;
    
    counter++;
    endtime = millis();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(20);
    digitalWrite(LED_BUILTIN, LOW);
    delay (20);
  }
  avrg_acc.x = avrg_acc.x/counter;
  avrg_acc.y = avrg_acc.y/counter;
  avrg_acc.z = avrg_acc.z/counter;
  avrg_gyro.x = avrg_gyro.x/counter;
  avrg_gyro.y = avrg_gyro.y/counter;
  avrg_gyro.z = avrg_gyro.z/counter;

}

void mag_calibrate()
{

  double starttime, endtime;
  starttime = millis();
  endtime = starttime;
  while ((endtime - starttime) <= 5000) // do this loop for up to 1000mS
  {
    Serial.println("Calibrating!");
    mag.read();

    running_min_mag.x = min(running_min_mag.x, mag.m.x);
    running_min_mag.y = min(running_min_mag.y, mag.m.y);
    running_min_mag.z = min(running_min_mag.z, mag.m.z);

    running_max_mag.x = max(running_max_mag.x, mag.m.x);
    running_max_mag.y = max(running_max_mag.y, mag.m.y);
    running_max_mag.z = max(running_max_mag.z, mag.m.z);



    endtime = millis();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(20);
    digitalWrite(LED_BUILTIN, LOW);
    delay (20);
  }

}
