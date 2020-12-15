#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

const int I2C_ADDR = 0x68;
const int PWR_MGMT_1 = 0x6B;

double Y_ANGULAR_BIAS = 0.0;

float read_word_2c(int fd, int addr)
{
  int high = wiringPiI2CReadReg8(fd, addr);
  int low = wiringPiI2CReadReg8(fd, addr + 1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

void obtain_sensor_calibrations(const ros::NodeHandle &n) {
  n.getParam("/y_angular_bias", Y_ANGULAR_BIAS);
}

void populate_raw_imu(sensor_msgs::Imu &m, int fd)
{
  // Read gyroscope values.
  double scale_ang_vel = 7505.75; // scaling factor to convert to rad/s
  m.angular_velocity.x = read_word_2c(fd, 0x43) / scale_ang_vel;
  m.angular_velocity.y = read_word_2c(fd, 0x45) / scale_ang_vel;
  m.angular_velocity.z = read_word_2c(fd, 0x47) / scale_ang_vel;

  // Read accelerometer values.
  // At default sensitivity of 2g we need to scale by 16384.
  // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
  // But! Imu msg docs say acceleration should be in m/s2 so need to *9.807
  const float la_rescale = 16384.0 / 9.807;
  m.linear_acceleration.x = read_word_2c(fd, 0x3b) / la_rescale;
  m.linear_acceleration.y = read_word_2c(fd, 0x3d) / la_rescale;
  m.linear_acceleration.z = read_word_2c(fd, 0x3f) / la_rescale;
}

void calibrate_imu(sensor_msgs::Imu &m)
{
  m.angular_velocity.y = m.angular_velocity.y - Y_ANGULAR_BIAS;
}

int main(int argc, char **argv)
{

  // Connect to device.
  int fd = wiringPiI2CSetup(I2C_ADDR);
  if (fd == -1)
  {
    printf("no i2c device found?\n");
    return -1;
  }
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);

  // Start ROS node stuff.
  ros::init(argc, argv, "mpu6050_node");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Rate rate(50); // hz

  obtain_sensor_calibrations(node);

  // Publish in loop.
  while (ros::ok())
  {
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = '0'; // no frame

    populate_raw_imu(msg, fd);
    calibrate_imu(msg);

    // Pub & sleep.
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
