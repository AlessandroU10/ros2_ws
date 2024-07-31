from tutorial_interfaces.srv import AddThreeInts

import rclpy
from rclpy.node import Node


class AdditionService(Node):

   def __init__(self):
      super().__init__('add_service_node')
      self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)

   def add_three_ints_callback(self, request, response):
      response.sum = request.a + request.b + request.c
      self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))

      return response

def main():
   rclpy.init()

   addition_service = AdditionService()

   rclpy.spin(addition_service)

   rclpy.shutdown()

if __name__ == '__main__':
   main()