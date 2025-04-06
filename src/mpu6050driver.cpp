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
  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  std::chrono::duration<int64_t, std::milli> frequency =
      1000ms / this->get_parameter("gyro_range").as_int();
  timer_ = this->create_wall_timer(frequency, std::bind(&MPU6050Driver::handleInput, this));
}

void MPU6050Driver::handleInput()
{
  auto message = sensor_msgs::msg::Imu();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";

  // Get raw sensor data
  double ax = mpu6050_->getAccelerationX();
  double ay = mpu6050_->getAccelerationY();
  double az = mpu6050_->getAccelerationZ();
  message.linear_acceleration.x = ax;
  message.linear_acceleration.y = ay;
  message.linear_acceleration.z = az;
  message.linear_acceleration_covariance = {0};

  message.angular_velocity.x = mpu6050_->getAngularVelocityX();
  message.angular_velocity.y = mpu6050_->getAngularVelocityY();
  message.angular_velocity.z = mpu6050_->getAngularVelocityZ();
  message.angular_velocity_covariance[0] = 0;

  // Estimate orientation (roll and pitch) from the accelerometer
  double roll = atan2(ay, az);
  double pitch = atan2(-ax, sqrt(ay * ay + az * az));
  double yaw = 0.0;  // Yaw remains unknown without a magnetometer

  // Convert Euler angles to quaternion
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  message.orientation.w = cr * cp * cy + sr * sp * sy;
  message.orientation.x = sr * cp * cy - cr * sp * sy;
  message.orientation.y = cr * sp * cy + sr * cp * sy;
  message.orientation.z = cr * cp * sy - sr * sp * cy;

  // Optionally update covariance values if available
  // Here covariance is set to a default style:
  message.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

  publisher_->publish(message);
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