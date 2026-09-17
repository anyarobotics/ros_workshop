import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.085
WHEEL_SEPARATION = 0.380527
MAX_WHEEL_SPEED = 10.0

def clamp(value, low, high):
    return max(low, min(high, value))
def twist_to_wheel_speeds(linear_x, angular_z, separation, radius):
    half = separation / 2.0
    left = (linear_x + angular_z * half) / radius
    right = (linear_x - angular_z * half) / radius
    return left, right

class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.declare_parameter('wheel_separation', WHEEL_SEPARATION)
        self.declare_parameter('max_wheel_speed', MAX_WHEEL_SPEED)
        self.radius = self.get_parameter('wheel_radius').value
        self.separation = self.get_parameter('wheel_separation').value
        self.max_speed = self.get_parameter('max_wheel_speed').value
        self.left_pub = self.create_publisher(Float64, '/wheel_left_joint/cmd_vel', 10)
        self.right_pub = self.create_publisher(Float64, '/wheel_right_joint/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        left, right = twist_to_wheel_speeds(msg.linear.x, msg.angular.z, self.separation, self.radius)
        left = clamp(left, -self.max_speed, self.max_speed)
        right = clamp(right, -self.max_speed, self.max_speed)
        left_msg = Float64()
        left_msg.data = left
        right_msg = Float64()
        right_msg.data = right
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
