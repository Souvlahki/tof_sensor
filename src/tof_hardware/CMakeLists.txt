cmake_minimum_required(VERSION 3.8)
project(tof_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/tof_hardware_interface.cpp
  src/tof_serial.cpp
  src/cobsr.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle hardware_interface pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface tof_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include  # fixed typo
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()