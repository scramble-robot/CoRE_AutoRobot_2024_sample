import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty


class Mazemaze(Node):

    def __init__(self):
        super().__init__('mazemaze')

        self.subscription = self.create_subscription(
            Empty,
            '/mazemaze',
            self.callback,
            10)
        self.servo1_pub = self.create_publisher(Float64, '/servo1/degree', 10)

        self.OFFSET = 10.0
        self.pattern = [
            0.0,
            0.0,
            135.0+self.OFFSET,
            270.0,
            135.0+self.OFFSET
        ]
        self.index = 0

        self.pullTimer = self.create_timer(1.0, self.timer_callback)

    def mazemaze(self, value):
        self.servo1_pub.publish(Float64(data=value))

    def callback(self, msg):
        if self.index == 0:
            self.index = 1

    def timer_callback(self):
        if self.index != 0:
            self.mazemaze(self.pattern[self.index])
            self.index += 1
        if self.index >= len(self.pattern):
            self.index = 0


def main():
    rclpy.init()
    node = Mazemaze()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
