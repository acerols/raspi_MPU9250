cmake_minimum_required(VERSION 3.5)
project(publish_IMU)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mpu9250 REQUIRED)

add_executable(publish_IMU
  src/imu_pub.cpp
  src/MPU9250.cpp
  src/imu_pub_main.cpp
)

ament_target_dependencies(publish_IMU
  rclcpp
  mpu9250
)

target_include_directories(publish_IMU
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  publish_IMU
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
