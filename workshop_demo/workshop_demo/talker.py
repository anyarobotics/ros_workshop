import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class Talker(Node):

    def __init__(self):
        super().__init__('talker')

        self.publisher = self.create_publisher(
            Int32MultiArray,
            'numbers',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_numbers
        )

    def publish_numbers(self):

        msg = Int32MultiArray()

        msg.data = [5, 7]

        self.publisher.publish(msg)

        self.get_logger().info(
            f'Publishing: {msg.data}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = Talker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()