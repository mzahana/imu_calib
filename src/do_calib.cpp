#include "imu_calib/do_calib.h"

namespace imu_calib
{

DoCalib::DoCalib() : Node("do_calib"), state_(START)
{
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&DoCalib::imuCallback, this, std::placeholders::_1));

  this->declare_parameter<int>("measurements", 500);
  this->declare_parameter<double>("reference_acceleration", 9.80665);
  this->declare_parameter<std::string>("output_file", "imu_calib.yaml");

  measurements_per_orientation_ = this->get_parameter("measurements").as_int();
  reference_acceleration_ = this->get_parameter("reference_acceleration").as_double();
  output_file_ = this->get_parameter("output_file").as_string();

  orientations_.push(AccelCalib::XPOS);
  orientations_.push(AccelCalib::XNEG);
  orientations_.push(AccelCalib::YPOS);
  orientations_.push(AccelCalib::YNEG);
  orientations_.push(AccelCalib::ZPOS);
  orientations_.push(AccelCalib::ZNEG);

  orientation_labels_[AccelCalib::XPOS] = "X+";
  orientation_labels_[AccelCalib::XNEG] = "X-";
  orientation_labels_[AccelCalib::YPOS] = "Y+";
  orientation_labels_[AccelCalib::YNEG] = "Y-";
  orientation_labels_[AccelCalib::ZPOS] = "Z+";
  orientation_labels_[AccelCalib::ZNEG] = "Z-";
}

bool DoCalib::running()
{
  return state_ != DONE;
}

void DoCalib::imuCallback(sensor_msgs::msg::Imu::SharedPtr imu)
{
  bool accepted;

  switch (state_)
  {
  case START:
    calib_.beginCalib(6 * measurements_per_orientation_, reference_acceleration_);
    state_ = SWITCHING;
    break;

  case SWITCHING:
    if (orientations_.empty())
    {
      state_ = COMPUTING;
    }
    else
    {
      current_orientation_ = orientations_.front();

      orientations_.pop();
      measurements_received_ = 0;

      RCLCPP_INFO(this->get_logger(), "Orient IMU with %s axis up and press Enter",
                  orientation_labels_[current_orientation_].c_str());
      std::cin.get();
      RCLCPP_INFO(this->get_logger(), "Recording measurements...");

      state_ = RECEIVING;
    }
    break;

  case RECEIVING:
    accepted = calib_.addMeasurement(
        current_orientation_,
        imu->linear_acceleration.x,
        imu->linear_acceleration.y,
        imu->linear_acceleration.z);

    measurements_received_ += accepted ? 1 : 0;
    if (measurements_received_ >= measurements_per_orientation_)
    {
      RCLCPP_INFO(this->get_logger(), "Done.");
      state_ = SWITCHING;
    }
    break;

  case COMPUTING:
    RCLCPP_INFO(this->get_logger(), "Computing calibration parameters...");
    if (calib_.computeCalib())
    {
      RCLCPP_INFO(this->get_logger(), "Success!");

      RCLCPP_INFO(this->get_logger(), "Saving calibration file...");
      if (calib_.saveCalib(output_file_))
      {
        RCLCPP_INFO(this->get_logger(), "Success!");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to save calibration file.");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Calibration failed.");
    }
    state_ = DONE;
    break;

  case DONE:
    break;
  }
}

} // namespace imu_calib
