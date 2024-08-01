import sys

import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import AddThreeInts

class AdditionClientAsync(Node):

   def __init__(self):
      super().__init__('add_client_node')
      self.cli = self.create_client(AddThreeInts, 'add_three_ints')
      while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
      self.req = AddThreeInts.Request()

   def send_request(self, a, b, c):
      self.req.a = a
      self.req.b = b
      self.req.c = c
      self.future = self.cli.call_async(self.req)
      rclpy.spin_until_future_complete(self, self.future)
      return self.future.result()


def main():
   rclpy.init()

   add_client = AdditionClientAsync()
   response = add_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
   add_client.get_logger().info(
      'Result of add_three_ints: for %d + %d + %d = %d' %
      (int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), response.sum))

   add_client.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()