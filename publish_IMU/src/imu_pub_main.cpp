#include <rclcpp/rclcpp.hpp>
#include <publish_IMU/imu_pub.hpp>

using namespace IMU;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPub>());
    rclcpp::shutdown();
    return 0;
}