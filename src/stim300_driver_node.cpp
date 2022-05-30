#include "driver_stim300.h"
#include "serial_unix.h"
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

Eigen::Quaterniond estimate_orientation(const Eigen::Vector3d& acc)
{

	double pitch = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2]));
	//double roll = atan2(acc[1], sqrt(acc[0]*acc[0] + acc[2]*acc[2]));
	double roll = atan2(acc[1], acc[2]);

	Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ());

  // double yaw = atan2(-acc[1], acc[0]);
  // double pitch = atan2(acc[2], sqrt(acc[0] * acc[0] + acc[1] * acc[1]));
//
  // Eigen::Quaterniond quat = Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()) *
                            // Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            // Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return quat;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "stim300_driver_node");

  ros::NodeHandle node("~");
  std::string imu_path;
  std::string imu_link;
  std::string imu_output;
  node.param<std::string>("device_path", imu_path, "/dev/ttyUSB0");
  node.param<std::string>("imu_frame", imu_link, "sam/imu_link");
  node.param<std::string>("imu_output", imu_output, "stim_imu");

  double variance_gyro{0};
  double variance_acc{0};
  int sample_rate{0};
  double gravity{0};

  node.param("variance_gyro", variance_gyro, 0.0001);
  node.param("variance_acc", variance_acc, 4.0);
  node.param("sample_rate", sample_rate, 125);
  node.param("gravity", gravity, 9.80665);

  sensor_msgs::Imu stim300msg{};
  stim300msg.orientation_covariance[0] = 0.05;
  stim300msg.orientation_covariance[4] = 0.01;
  stim300msg.orientation_covariance[8] = 0.05;
  stim300msg.angular_velocity_covariance[0] = variance_gyro;
  stim300msg.angular_velocity_covariance[4] = variance_gyro;
  stim300msg.angular_velocity_covariance[8] = variance_gyro;
  stim300msg.linear_acceleration_covariance[0] = variance_acc;
  stim300msg.linear_acceleration_covariance[4] = variance_acc;
  stim300msg.linear_acceleration_covariance[8] = variance_acc;
  stim300msg.orientation.x = 0;
  stim300msg.orientation.y = 0;
  stim300msg.orientation.z = 0;
  stim300msg.header.frame_id = imu_link;

  ros::Publisher imuSensorPublisher = node.advertise<sensor_msgs::Imu>(imu_output, 1000);

  ros::Rate loop_rate(sample_rate);

  try {
    SerialUnix serial_driver(imu_path, stim_const::BaudRate::BAUD_921600);
    DriverStim300 driver_stim300(serial_driver);

    ROS_INFO("STIM300 IMU driver initialized successfully");

    while (ros::ok()) {
      switch (driver_stim300.update()) {
      case Stim300Status::NORMAL:
        break;
      case Stim300Status::OUTSIDE_OPERATING_CONDITIONS:
        ROS_DEBUG("Stim 300 outside operating conditions");
      case Stim300Status::NEW_MEASURMENT:
        stim300msg.header.stamp = ros::Time::now();
        // stim300msg.linear_acceleration.x = driver_stim300.getAccX() * gravity;
        // stim300msg.linear_acceleration.y = driver_stim300.getAccY() * gravity;
        // stim300msg.linear_acceleration.z = driver_stim300.getAccZ() * gravity;
        // stim300msg.angular_velocity.x = driver_stim300.getGyroX();
        // stim300msg.angular_velocity.y = driver_stim300.getGyroY();
        // stim300msg.angular_velocity.z = driver_stim300.getGyroZ();
        Eigen::Vector3d linear_acceleration = driver_stim300.getAccData();
        Eigen::Vector3d gyro_velocities = driver_stim300.getGyroData();
        Eigen::Vector3d incl_acceleration = driver_stim300.getInclData();

        Eigen::Quaterniond orientation = estimate_orientation(incl_acceleration);

        stim300msg.linear_acceleration.x = linear_acceleration[0];
        stim300msg.linear_acceleration.y = linear_acceleration[1];
        stim300msg.linear_acceleration.z = linear_acceleration[2];

        stim300msg.angular_velocity.x = gyro_velocities[0];
        stim300msg.angular_velocity.y = gyro_velocities[1];
        stim300msg.angular_velocity.z = gyro_velocities[2];

        stim300msg.orientation.x = orientation.x();
        stim300msg.orientation.y = orientation.y();
        stim300msg.orientation.z = orientation.z();
        stim300msg.orientation.w = orientation.w();

        // stim300msg.linear_acceleration.x = driver_stim300.getAccX() + 0.0023;
        // stim300msg.linear_acceleration.y = driver_stim300.getAccY() + 0.05;
        // stim300msg.linear_acceleration.z = driver_stim300.getAccZ() + 0.027;
        // stim300msg.angular_velocity.x = driver_stim300.getGyroX();
        // stim300msg.angular_velocity.y = driver_stim300.getGyroY();
        // stim300msg.angular_velocity.z = driver_stim300.getGyroZ();
        imuSensorPublisher.publish(stim300msg);
        break;
      case Stim300Status::CONFIG_CHANGED:
        ROS_INFO("Updated Stim 300 imu config: ");
        ROS_INFO("%s", driver_stim300.printSensorConfig().c_str());
        loop_rate = driver_stim300.getSampleRate() * 2;
        break;
      case Stim300Status::STARTING_SENSOR:
        ROS_INFO("Stim 300 IMU is warming up.");
        break;
      case Stim300Status::SYSTEM_INTEGRITY_ERROR:
        ROS_WARN("Stim 300 IMU system integrity error.");
        break;
      case Stim300Status::OVERLOAD:
        ROS_WARN("Stim 300 IMU overload.");
        break;
      case Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL:
        ROS_WARN("Stim 300 IMU error in measurement channel.");
        break;
      case Stim300Status::ERROR:
        ROS_WARN("Stim 300 IMU: internal error.");
      }

      loop_rate.sleep();
      ros::spinOnce();
    }
    return 0;
  } catch (std::runtime_error &error) {
    // TODO: Reset IMU
    ROS_ERROR("%s\n", error.what());
    return 0;
  }
}