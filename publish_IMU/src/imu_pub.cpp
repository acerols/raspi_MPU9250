#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>
#include <cmath>
#include <mpu9250/msg/sensor.hpp>
#include <publish_IMU/imu_pub.hpp>

static const char *dev_name = "/dev/i2c-1";

namespace IMU{
IMUPub::IMUPub(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("IMUPub", name_space, options){
    using namespace std::chrono_literals;
    
    Sensor = new MPU9250(dev_name);
    Sensor->CalibMag();
    
    pub_ = this->create_publisher<mpu9250::msg::Sensor>("mpu9250", rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
        50ms,
        std::bind(&IMUPub::publish, this)
    );
}

void IMUPub::publish()
{
    auto imu = std::make_shared<mpu9250::msg::Sensor>();
    
    double pi = std::acos(-1);
    double deg;

    int accint[3], rotint[3], magint[3];

    imu->acc.resize(3);
    imu->rot.resize(3);
    imu->mag.resize(3);

    if(Sensor->ReadData_Mag(magint) == 1){
        Sensor->ReadData(accint, rotint);
        Sensor->AccFix(accint, acc);
        Sensor->GyroFix(rotint, rot);
        Sensor->MagFix(magint, mag);
    }
    imu->acc[0] = acc[0];
    imu->acc[1] = acc[1];
    imu->acc[2] = acc[2];

    imu->rot[0] = rot[0];
    imu->rot[1] = rot[1];
    imu->rot[2] = rot[2];
    
    imu->mag[0] = mag[0];
    imu->mag[1] = mag[1];
    imu->mag[2] = mag[2];

    deg = std::atan2(mag[0], mag[1]);
    deg = deg * 180 / pi;
    RCLCPP_INFO(this->get_logger(), "MagX %f, MagY %f, MagZ %f", imu->mag[0], imu->mag[1], imu->mag[2]);
    RCLCPP_INFO(this->get_logger(), "Compass %f", deg);
    pub_->publish(*imu);

}

}
