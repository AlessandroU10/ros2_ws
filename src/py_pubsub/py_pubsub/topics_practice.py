import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MotionController(Node):
    
    def __init__(self):
        super().__init__('motion_controller')
        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        
        self.pose_ = Pose()
        self.go_straight = False
        self.linear_velocity = 1.0
        self.angular_velocity = 1.0
        self.angle_ = 0.0
        self.init_pos_x = 5.5
        self.init_pos_y = 5.5

    def pose_callback(self, msg):
        self.pose_.x = msg.x
        self.pose_.y = msg.y
        
        MIN_BORDER = 2.0
        MAX_BORDER = 8.0

        if MIN_BORDER < self.pose_.x < MAX_BORDER and MIN_BORDER < self.pose_.y < MAX_BORDER:
            self.go_straight = True
        else:
            self.go_straight = False

    def timer_callback(self):
        twist_msg = Twist()
        
        if self.go_straight:
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = 0.0
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_velocity
        
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
