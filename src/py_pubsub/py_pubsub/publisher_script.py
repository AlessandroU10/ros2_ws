import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

   def __init__(self):
      super().__init__('publisher')
      self.publisher_ = self.create_publisher(String, 'topic', 10)
      timer_period = 0.5  # seconds
      self.timer_ = self.create_timer(timer_period, self.timer_callback)
      self.count_ = 0

   def timer_callback(self):
      msg = String()
      msg.data = 'Hello World: %d' % self.count_
      self.publisher_.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.data)
      self.count_ += 1


def main(args=None):
   rclpy.init(args=args)

   minimal_publisher = MinimalPublisher()

   rclpy.spin(minimal_publisher)

   minimal_publisher.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()