import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

class TurtleColorChanger(Node):

    def __init__(self):
        super().__init__('turtle_color_changer')
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.current_color = None

    def pose_callback(self, msg):
        if msg.x > 5.5:
            if self.current_color != 'red':
                self.change_pen_color(255, 0, 0)
                self.current_color = 'red'
                self.get_logger().info('Set pen color to red')
        else:
            if self.current_color != 'green':
                self.change_pen_color(0, 255, 0)
                self.current_color = 'green'
                self.get_logger().info('Set pen color to green')

    def change_pen_color(self, r, g, b):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 3
        request.off = 0
        self.pen_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    turtle_color_changer = TurtleColorChanger()
    rclpy.spin(turtle_color_changer)
    turtle_color_changer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
