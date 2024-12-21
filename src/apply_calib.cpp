#include "imu_calib/apply_calib.h"

namespace imu_calib
{

ApplyCalib::ApplyCalib() 
  : Node("apply_calib"), 
    gyro_sample_count_(0), 
    gyro_bias_x_(0.0), 
    gyro_bias_y_(0.0), 
    gyro_bias_z_(0.0)
{
  // Declare parameters
  this->declare_parameter<std::string>("calib_file", "imu_calib.yaml");
  this->declare_parameter<bool>("calibrate_gyros", true);
  this->declare_parameter<int>("gyro_calib_samples", 100);
  this->declare_parameter<int>("queue_size", 10);

  // Get parameter values
  std::string calib_file = this->get_parameter("calib_file").as_string();
  calibrate_gyros_ = this->get_parameter("calibrate_gyros").as_bool();
  gyro_calib_samples_ = this->get_parameter("gyro_calib_samples").as_int();
  int queue_size = this->get_parameter("queue_size").as_int();

  // Load calibration data
  if (!calib_.loadCalib(calib_file) || !calib_.calibReady())
  {
    RCLCPP_FATAL(this->get_logger(), "Calibration could not be loaded from file: %s", calib_file.c_str());
    rclcpp::shutdown();
  }

  // Subscribers and publishers
  raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "raw", queue_size, std::bind(&ApplyCalib::rawImuCallback, this, std::placeholders::_1));
  corrected_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("corrected", queue_size);
}

void ApplyCalib::rawImuCallback(sensor_msgs::msg::Imu::SharedPtr raw)
{
  if (calibrate_gyros_)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Calibrating gyros; do not move the IMU");

    // Compute mean gyro measurements for bias calculation
    gyro_sample_count_++;
    gyro_bias_x_ = ((gyro_sample_count_ - 1) * gyro_bias_x_ + raw->angular_velocity.x) / gyro_sample_count_;
    gyro_bias_y_ = ((gyro_sample_count_ - 1) * gyro_bias_y_ + raw->angular_velocity.y) / gyro_sample_count_;
    gyro_bias_z_ = ((gyro_sample_count_ - 1) * gyro_bias_z_ + raw->angular_velocity.z) / gyro_sample_count_;

    if (gyro_sample_count_ >= gyro_calib_samples_)
    {
      RCLCPP_INFO(this->get_logger(), "Gyro calibration complete! Bias = [%.3f, %.3f, %.3f]",
                  gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
      calibrate_gyros_ = false;
    }

    return;
  }

  // Apply calibration to the raw IMU data
  sensor_msgs::msg::Imu corrected = *raw;

  calib_.applyCalib(
      raw->linear_acceleration.x, raw->linear_acceleration.y, raw->linear_acceleration.z,
      &corrected.linear_acceleration.x, &corrected.linear_acceleration.y, &corrected.linear_acceleration.z);

  corrected.angular_velocity.x -= gyro_bias_x_;
  corrected.angular_velocity.y -= gyro_bias_y_;
  corrected.angular_velocity.z -= gyro_bias_z_;

  // Publish the corrected IMU data
  corrected_pub_->publish(corrected);
}

} // namespace imu_calib
