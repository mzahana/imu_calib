cmake_minimum_required(VERSION 3.8)
project(imu_calib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(accel_calib src/accel_calib/accel_calib.cpp)
target_link_libraries(accel_calib yaml-cpp)
ament_target_dependencies(accel_calib rclcpp sensor_msgs yaml-cpp Eigen3)

add_executable(do_calib_node src/do_calib_node.cpp src/do_calib.cpp)
target_link_libraries(do_calib_node accel_calib)
ament_target_dependencies(do_calib_node rclcpp sensor_msgs yaml-cpp Eigen3)

add_executable(apply_calib_node src/apply_calib_node.cpp src/apply_calib.cpp)
target_link_libraries(apply_calib_node accel_calib)
ament_target_dependencies(apply_calib_node rclcpp sensor_msgs yaml-cpp Eigen3)

install(TARGETS
  accel_calib
  do_calib_node
  apply_calib_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
