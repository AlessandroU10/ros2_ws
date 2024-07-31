import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Sphere                                              # Change

class SphereListener(Node):

   def __init__(self):
      super().__init__('sphere_listener')                                             # Change
      self.subscription_ = self.create_subscription(                                  # Change
            Sphere,
            'sphere_topic',
            self.listener_callback,
            10)
      self.subscription_  # prevent unused variable warning

   def listener_callback(self, msg):
      self.get_logger().info('I heard (x, y, z, radius):'+                            # Change
                              'x=%s, y=%s, z=%s, radius=%s' %
                              (msg.center.x, msg.center.y, msg.center.z, msg.radius))


def main(args=None):
   rclpy.init(args=args)

   sphere_listener = SphereListener()

   rclpy.spin(sphere_listener)

   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   sphere_listener.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()