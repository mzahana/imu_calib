#ifndef IMU_CALIB_ACCEL_CALIB_H
#define IMU_CALIB_ACCEL_CALIB_H

#include <Eigen/Dense>
#include <string>

namespace imu_calib
{

class AccelCalib
{
public:
  enum Orientation { XPOS = 0, XNEG, YPOS, YNEG, ZPOS, ZNEG };

  AccelCalib();
  explicit AccelCalib(const std::string &calib_file);

  // Status
  bool calibReady() const;

  // File I/O
  bool loadCalib(const std::string &calib_file);
  bool saveCalib(const std::string &calib_file) const;

  // Calibration procedure
  void beginCalib(int measurements, double reference_acceleration);
  bool addMeasurement(Orientation orientation, double ax, double ay, double az);
  bool computeCalib();

  // Calibration application
  void applyCalib(double raw[3], double corrected[3]) const;
  void applyCalib(double raw_x, double raw_y, double raw_z, double *corr_x, double *corr_y, double *corr_z) const;

private:
  static const int reference_index_[6];
  static const int reference_sign_[6];

  bool calib_ready_;
  Eigen::Matrix3d SM_;  // Combined scale and misalignment parameters
  Eigen::Vector3d bias_;  // Scaled and rotated bias parameters
  double reference_acceleration_;  // Expected acceleration measurement

  bool calib_initialized_;
  int orientation_count_[6];

  Eigen::MatrixXd meas_;  // Least squares measurements matrix
  Eigen::VectorXd ref_;  // Least squares expected measurements vector
  int num_measurements_;  // Number of measurements expected
  int measurements_received_;  // Number of measurements received
};

} // namespace imu_calib

#endif // IMU_CALIB_ACCEL_CALIB_H
