import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Sphere                                      # Change

class SpherePublisher(Node):                                                    # Change

   def __init__(self):
      super().__init__('sphere_publisher')                                    # Change
      self.publisher_ = self.create_publisher(Sphere, 'sphere_topic', 10)     # Change
      timer_period = 0.5  # seconds
      self.timer_ = self.create_timer(timer_period, self.timer_callback)
      self.count_ = 0.0

   def timer_callback(self):
      msg = Sphere()                                                          # Change
      msg.center.x = self.count_                                              # Change
      msg.center.y = 1.0                                                      # Change
      msg.center.z = 2.0                                                      # Change
      msg.radius = 10.0                                                       # Change
      self.publisher_.publish(msg)                                            # Change
      self.get_logger().info('Publishing sphere params (x, y, z, radius):' +  # Change
                              'x=%s, y=%s, z=%s, radius=%s' %
                              (msg.center.x, msg.center.y, msg.center.z, msg.radius))
      self.count_ += 1.0


def main(args=None):
   rclpy.init(args=args)

   sphere_publisher = SpherePublisher()

   rclpy.spin(sphere_publisher)

   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   sphere_publisher.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()