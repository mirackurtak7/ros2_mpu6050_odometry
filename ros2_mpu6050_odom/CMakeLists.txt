cmake_minimum_required(VERSION 3.5)
project(mpu6050driver)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)          # Yeni eklenen
find_package(geometry_msgs REQUIRED)     # Yeni eklenen
find_package(tf2_ros REQUIRED)           # Yeni eklenen

# MPU6050 Driver Node
add_executable(mpu6050driver src/mpu6050driver.cpp src/mpu6050sensor.cpp)
ament_target_dependencies(mpu6050driver rclcpp sensor_msgs)
target_link_libraries(mpu6050driver i2c)
target_include_directories(
  mpu6050driver PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Odometry Node (Yeni eklenen)
add_executable(odometry_node src/odom.py)
ament_target_dependencies(
  odom
  rclpy
  nav_msgs
  geometry_msgs
  tf2_ros
  sensor_msgs
)

# Install hedefleri
install(TARGETS 
  mpu6050driver
  odometry_node          # Yeni eklenen
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  params 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
