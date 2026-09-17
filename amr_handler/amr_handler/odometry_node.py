import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS = 0.085
WHEEL_SEPARATION = 0.380527


def normalize_angle(theta):
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def extract_wheel_positions(msg):
    left_pos = None
    right_pos = None
    for name, position in zip(msg.name, msg.position):
        if name == 'wheel_left_joint':
            left_pos = position
        elif name == 'wheel_right_joint':
            right_pos = position
    return left_pos, right_pos


def wheel_deltas_to_pose_delta(delta_left, delta_right, separation, radius):
    dist_left = -delta_left * radius
    dist_right = -delta_right * radius
    delta_dist = (dist_left + dist_right) / 2.0
    delta_theta = (dist_left - dist_right) / separation
    return delta_dist, delta_theta


def integrate_pose(x, y, theta, delta_dist, delta_theta):
    theta_mid = theta + delta_theta / 2.0
    x_new = x + delta_dist * math.cos(theta_mid)
    y_new = y + delta_dist * math.sin(theta_mid)
    theta_new = normalize_angle(theta + delta_theta)
    return x_new, y_new, theta_new


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def build_odometry(odom_frame, base_frame, stamp, x, y, theta, linear, angular):
    qx, qy, qz, qw = yaw_to_quaternion(theta)
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = odom_frame
    odom.child_frame_id = base_frame
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.x = qx
    odom.pose.pose.orientation.y = qy
    odom.pose.pose.orientation.z = qz
    odom.pose.pose.orientation.w = qw
    odom.twist.twist.linear.x = linear
    odom.twist.twist.angular.z = angular
    return odom


def build_transform(odom_frame, base_frame, stamp, x, y, theta):
    qx, qy, qz, qw = yaw_to_quaternion(theta)
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = odom_frame
    tf.child_frame_id = base_frame
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = 0.0
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz
    tf.transform.rotation.w = qw
    return tf


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.declare_parameter('wheel_separation', WHEEL_SEPARATION)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.radius = self.get_parameter('wheel_radius').value
        self.separation = self.get_parameter('wheel_separation').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = None
        self.last_right = None
        self.last_time = None
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        left_pos, right_pos = extract_wheel_positions(msg)
        if left_pos is None or right_pos is None:
            return
        now = stamp_to_seconds(msg.header.stamp)
        if self.last_left is None:
            self.last_left = left_pos
            self.last_right = right_pos
            self.last_time = now
            return
        dt = now - self.last_time
        if dt <= 0.0:
            return
        delta_left = left_pos - self.last_left
        delta_right = right_pos - self.last_right
        self.last_left = left_pos
        self.last_right = right_pos
        self.last_time = now
        delta_dist, delta_theta = wheel_deltas_to_pose_delta(delta_left, delta_right, self.separation, self.radius)
        self.x, self.y, self.theta = integrate_pose(self.x, self.y, self.theta, delta_dist, delta_theta)
        linear = delta_dist / dt
        angular = delta_theta / dt
        odom = build_odometry(self.odom_frame, self.base_frame, msg.header.stamp, self.x, self.y, self.theta, linear, angular)
        self.odom_pub.publish(odom)
        tf = build_transform(self.odom_frame, self.base_frame, msg.header.stamp, self.x, self.y, self.theta)
        self.broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
