#include "mpu6050driver/mpu6050driver.h"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

MPU6050Driver::MPU6050Driver()
    : Node("mpu6050publisher"), mpu6050_{std::make_unique<MPU6050Sensor>()}
{
    // Declare parameters
    declareParameters();
    // Set parameters
    mpu6050_->setGyroscopeRange(
        static_cast<MPU6050Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
    mpu6050_->setAccelerometerRange(
        static_cast<MPU6050Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
    mpu6050_->setDlpfBandwidth(
        static_cast<MPU6050Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
    mpu6050_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                                 this->get_parameter("gyro_y_offset").as_double(),
                                 this->get_parameter("gyro_z_offset").as_double());
    mpu6050_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                     this->get_parameter("accel_y_offset").as_double(),
                                     this->get_parameter("accel_z_offset").as_double());
    // Check if we want to calibrate the sensor
    if (this->get_parameter("calibrate").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Calibrating...");
        mpu6050_->calibrate();
    }
    mpu6050_->printConfig();
    mpu6050_->printOffsets();
    // Create publisher for IMU data
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    // Create publisher for odometry data
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    // Set timer for handling input
    std::chrono::duration<int64_t, std::milli> frequency =
        1000ms / this->get_parameter("gyro_range").as_int();
    timer_ = this->create_wall_timer(frequency, std::bind(&MPU6050Driver::handleInput, this));
}

void MPU6050Driver::handleInput()
{
    auto imu_message = sensor_msgs::msg::Imu();
    imu_message.header.stamp = this->get_clock()->now();
    imu_message.header.frame_id = "base_link";
    imu_message.linear_acceleration_covariance = {0};
    imu_message.linear_acceleration.x = mpu6050_->getAccelerationX();
    imu_message.linear_acceleration.y = mpu6050_->getAccelerationY();
    imu_message.linear_acceleration.z = mpu6050_->getAccelerationZ();
    imu_message.angular_velocity_covariance[0] = {0};
    imu_message.angular_velocity.x = mpu6050_->getAngularVelocityX();
    imu_message.angular_velocity.y = mpu6050_->getAngularVelocityY();
    imu_message.angular_velocity.z = mpu6050_->getAngularVelocityZ();
    // Invalidate quaternion
    imu_message.orientation_covariance[0] = -1;
    imu_message.orientation.x = 0;
    imu_message.orientation.y = 0;
    imu_message.orientation.z = 0;
    imu_message.orientation.w = 0;
    imu_publisher_->publish(imu_message);

    auto odom_message = nav_msgs::msg::Odometry();
    odom_message.header.stamp = this->get_clock()->now();
    odom_message.header.frame_id = "odom";
    odom_message.child_frame_id = "base_footprint";
    // Odometry data here, this is just example data
    odom_message.pose.pose.position.x = 0.0;
    odom_message.pose.pose.position.y = 0.0;
    odom_message.pose.pose.position.z = 0.0;
    odom_message.pose.pose.orientation.x = 0.0;
    odom_message.pose.pose.orientation.y = 0.0;
    odom_message.pose.pose.orientation.z = 0.0;
    odom_message.pose.pose.orientation.w = 1.0;
    odom_message.twist.twist.linear.x = 0.0;
    odom_message.twist.twist.linear.y = 0.0;
    odom_message.twist.twist.linear.z = 0.0;
    odom_message.twist.twist.angular.x = 0.0;
    odom_message.twist.twist.angular.y = 0.0;
    odom_message.twist.twist.angular.z = 0.0;
    odometry_publisher_->publish(odom_message);
}

void MPU6050Driver::declareParameters()
{
    this->declare_parameter<bool>("calibrate", true);
    this->declare_parameter<int>("gyro_range", MPU6050Sensor::GyroRange::GYR_250_DEG_S);
    this->declare_parameter<int>("accel_range", MPU6050Sensor::AccelRange::ACC_2_G);
    this->declare_parameter<int>("dlpf_bandwidth", MPU6050Sensor::DlpfBandwidth::DLPF_260_HZ);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);
    this->declare_parameter<int>("frequency", 0.0);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050Driver>());
    rclcpp::shutdown();
    return 0;
}
