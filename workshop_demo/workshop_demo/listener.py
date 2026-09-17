import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class Listener(Node):

    def __init__(self):
        super().__init__('listener')

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'numbers',
            self.receive_numbers,
            10
        )

    def receive_numbers(self, msg):

        number1 = msg.data[0]
        number2 = msg.data[1]

        total = number1 + number2

        self.get_logger().info(
            f'Received: {number1}, {number2} | Sum = {total}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = Listener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()