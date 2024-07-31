#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"

using std::placeholders::_1;

class TurtleColorChanger : public rclcpp::Node
{
public:
  TurtleColorChanger() : Node("service_practice"), current_color("")
  {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&TurtleColorChanger::pose_callback, this, _1));

    pen_client_ = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
    while (!pen_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    if (msg->x > 5.5)
    {
      if (current_color != "red")
      {
        change_pen_color(255, 0, 0);
        current_color = "red";
        RCLCPP_INFO(this->get_logger(), "Set pen color to red");
      }
    }
    else
    {
      if (current_color != "green")
      {
        change_pen_color(0, 255, 0);
        current_color = "green";
        RCLCPP_INFO(this->get_logger(), "Set pen color to green");
      }
    }
  }

  void change_pen_color(uint8_t r, uint8_t g, uint8_t b)
  {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = 3;
    request->off = 0;

    pen_client_->async_send_request(request);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
  std::string current_color;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleColorChanger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
