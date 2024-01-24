import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty


class Hammer(Node):
    def __init__(self):
        super().__init__('hammer')

        self.subscription = self.create_subscription(
            Empty,
            '/hammer',
            self.callback,
            10)
        self.servo0_pub = self.create_publisher(Float64, '/servo0/degree', 10)

        self.pull_timer = self.create_timer(0.8, self.pull)
        self.clear_timer = self.create_timer(1.6, self.clear)
        self.pull_timer.cancel()
        self.clear_timer.cancel()
        self.trigger = False

    def hammer(self, value):
        self.servo0_pub.publish(Float64(data=value))

    def push(self):
        print('push')
        self.hammer(85.0)

    def pull(self):
        self.hammer(0.0)
        print('pull')
        self.pull_timer.cancel()

    def clear(self):
        self.trigger = False
        print('clear')
        self.clear_timer.cancel()

    def callback(self, msg):
        if not self.trigger:
            self.trigger = True
            self.push()
            self.pull_timer.reset()
            self.clear_timer.reset()


def main():
    rclpy.init()
    node = Hammer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
