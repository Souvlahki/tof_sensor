cmake_minimum_required(VERSION 3.8)
project(tof_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
add_executable(range_to_pointcloud src/range_to_pointcloud.cpp)
target_link_libraries(range_to_pointcloud PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
)

add_executable(params_config src/params_config.cpp)
target_link_libraries(params_config PRIVATE 
  rclcpp::rclcpp
  control_msgs::control_msgs
)

install(TARGETS
  range_to_pointcloud
  params_config
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
