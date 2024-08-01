#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class MotionController : public rclcpp::Node {
public:
    MotionController() : Node("motion_controller"), counter_(5), linear_velocity_(0.5), angular_velocity_(1.0), in_bounds_(true) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&MotionController::pose_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&MotionController::timer_callback, this));
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        pose_.x = msg->x;
        pose_.y = msg->y;

        const double MIN_BORDER = 2.0;
        const double MAX_BORDER = 8.0;

        if (MIN_BORDER < pose_.x && pose_.x < MAX_BORDER && MIN_BORDER < pose_.y && pose_.y < MAX_BORDER) {
            in_bounds_ = true;
        } else {
            in_bounds_ = false;
        }
    }

    void timer_callback() {
        if (counter_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
            --counter_;
            return;
        }

        auto twist_msg = geometry_msgs::msg::Twist();

        if (in_bounds_) {
            twist_msg.linear.x = linear_velocity_;
            twist_msg.angular.z = angular_velocity_;

            linear_velocity_ += 0.01;
            angular_velocity_ = 1.0 / linear_velocity_;

            RCLCPP_INFO(this->get_logger(), "Drawing spiral");
        } else {
            twist_msg.linear.x = linear_velocity_;
            twist_msg.angular.z = 0.0;

            RCLCPP_INFO(this->get_logger(), "Going straight");
        }

        publisher_->publish(twist_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose pose_;
    int counter_;
    double linear_velocity_;
    double angular_velocity_;
    bool in_bounds_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionController>());
    rclcpp::shutdown();
    return 0;
}
