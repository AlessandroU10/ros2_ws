#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/Sphere.hpp"                                       // CHANGE

using std::placeholders::_1;

class SphereListener : public rclcpp::Node
{
   public:
   SphereListener()
   : Node("sphere_listener")
   {
      subscription_ = this->create_subscription<tutorial_interfaces::msg::Sphere>(    // CHANGE
         "sphere_topic", 10, std::bind(&SphereListener::topic_callback, this, _1));
   }

   private:
   void topic_callback(const tutorial_interfaces::msg::Sphere & msg) const  // CHANGE
   {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard" \
      ": x = " << msg.center.x << ", y = " \
         << msg.center.y << ", z = " << msg.center.z << \
         ", radius = " << msg.radius);    // CHANGE
   }
   rclcpp::Subscription<tutorial_interfaces::msg::Sphere>::SharedPtr subscription_;  // CHANGE
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<SphereListener>());
   rclcpp::shutdown();
   return 0;
}