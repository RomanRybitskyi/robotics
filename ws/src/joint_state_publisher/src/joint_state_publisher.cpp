#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStatePublisher : public rclcpp::Node {
public:
    JointStatePublisher()
        : Node("joint_state_publisher"), angle_(0.0) {
        // Initialize publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        // Create a timer to publish messages periodically
        timer_ = this->create_wall_timer(100ms, std::bind(&JointStatePublisher::publish_joint_states, this));
    }

private:
    void publish_joint_states() {
        // Create a JointState message
        auto message = sensor_msgs::msg::JointState();

        // Populate the header with the current time
        message.header.stamp = this->get_clock()->now();

        // Define joint names
        message.name = {
            "base_to_front_left_wheel",
            "base_to_front_right_wheel",
            "base_to_rear_left_wheel",
            "base_to_rear_right_wheel",
        };
        
        // Simulate joint positions with a sinusoidal movement
      //  if (angle_ < 6.28+6.28+6.28+6.28+6.28){
       // 	angle_+=0.1;
       // }
       angle_ = 3.14*1.9;
        message.position = {
        	angle_,
        	angle_,
        	angle_,	
        	angle_,
        };
        
        // Optionally add velocity and effort
        message.velocity = {-0.1, -0.1, -0.1, -0.1};
        message.effort = {0.0, 0.0, 0.0, 0.0};

        // Publish the message
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published joint states: [%.2f, %.2f, %.2f, %.2f]",
                    message.position[0], message.position[1],
                    message.position[2], message.position[3]);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double angle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
