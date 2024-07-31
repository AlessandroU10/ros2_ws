#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/Sphere.hpp"                                            // CHANGE

using namespace std::chrono_literals;

class SpherePublisher : public rclcpp::Node
{
   public:
   SpherePublisher()
   : Node("sphere_publisher"), count_(0)
   {
      publisher_ = this->create_publisher<tutorial_interfaces::msg::Sphere>("sphere_topic", 10);  // CHANGE
      timer_ = this->create_wall_timer(
         500ms, std::bind(&SpherePublisher::timer_callback, this));
   }

   private:
   void timer_callback()
   {
      auto message = tutorial_interfaces::msg::Sphere();                                   // CHANGE
      message.center.x = this->count_;
      message.center.y = 1.0;
      message.center.z = 2.0;
      message.radius = 10.0;

      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing sphere params " \
      "(x, y, z, radius): x = " << message.center.x << ", y = " \
         << message.center.y << ", z = " << message.center.z << \
         ", radius = " << message.radius);    // CHANGE
      publisher_->publish(message);
      this->count_++;
   }

   rclcpp::TimerBase::SharedPtr timer_;
   rclcpp::Publisher<tutorial_interfaces::msg::Sphere>::SharedPtr publisher_;             // CHANGE
   size_t count_;
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<SpherePublisher>());
   rclcpp::shutdown();
   return 0;
}