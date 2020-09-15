#include <rclcpp/rclcpp.hpp>
#include <mpu9250/msg/sensor.hpp>
#include <MPU9250.hpp>

namespace IMU{
class IMUPub : public rclcpp::Node{
private:
    MPU9250 *Sensor;
    double acc[3], rot[3], mag[3];
    rclcpp::Publisher<mpu9250::msg::Sensor>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    IMUPub(const std::string &name_space="",
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    void publish();
};
}