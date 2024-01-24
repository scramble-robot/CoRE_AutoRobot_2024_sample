import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Int64, Empty, Bool
import math

class ManualInput(Node):
    def joy_callback(self, msg):
        self.joy.update(msg)
        joy = self.joy
        # print(vars(joy))

        if joy.L1:
            self.control_pub.publish(Bool(data=True))
        else:
            self.control_pub.publish(Bool(data=False))

        if joy.up:
            self.auto_mode = self.AUTO_MODE_TRACKING
        if joy.down:
            self.auto_mode = self.AUTO_MODE_SEQUENCE

        if joy.L2():
            self.auto_pub.publish(Int64(data=self.auto_mode))
        else:
            self.auto_pub.publish(Int64(data=self.AUTO_MODE_MANUAL))

            self.roller(joy.R2.value()*4000)
            if joy.option:
                self.raw_roller(0.0)
                self.raw_yaw(0.0)
                self.raw_pitch(0.0)
            if joy.R1:
                self.hammer_push()
                self.mazemaze()

            if joy.left:
                self.mazemaze()

            self.yaw(joy.R.y)
            self.pitch(joy.R.x)

    def __init__(self):
        super().__init__('manual_input')

        self.joy = JoyToDualSense()

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            qos_profile_sensor_data)

        self.c620_0_pub = self.create_publisher(Float64, '/can_node/c620_0/target_current', 10)
        self.c620_1_pub = self.create_publisher(Float64, '/can_node/c620_1/target_current', 10)

        self.gm6020_0_pub = self.create_publisher(Int64, '/can_node/gm6020_0/target_volt', 10)
        self.gm6020_1_pub = self.create_publisher(Int64, '/can_node/gm6020_1/target_volt', 10)

        self.hammer_pub = self.create_publisher(Empty, "/hammer", 10)
        self.mazemaze_pub = self.create_publisher(Empty, "/mazemaze", 10)
        self.roller_pub = self.create_publisher(Float64, "/roller", 10)
        self.control_pub = self.create_publisher(Bool, "/control", 10)
        self.yaw_pub = self.create_publisher(Float64, "/yaw", 10)
        self.pitch_pub = self.create_publisher(Float64, "/pitch", 10)
        self.auto_pub = self.create_publisher(Int64, "/auto", 10)

        self.AUTO_MODE_MANUAL   = 0
        self.AUTO_MODE_TRACKING = 1
        self.AUTO_MODE_SEQUENCE = 2
        self.auto_mode = self.AUTO_MODE_MANUAL

    def roller(self,value):
        self.roller_pub.publish(Float64(data=value))

    def raw_roller(self,value):
        self.c620_0_pub.publish(Float64(data=value))
        self.c620_1_pub.publish(Float64(data=value*-1))

    def hammer_push(self):
        self.hammer_pub.publish(Empty())

    def mazemaze(self):
        self.mazemaze_pub.publish(Empty())

    def raw_yaw(self,value):
        self.gm6020_1_pub.publish(Int64(data=int(value*10000)))

    def raw_pitch(self,value):
        self.gm6020_0_pub.publish(Int64(data=int(value*10000)))

    def yaw(self,value):
        self.yaw_pub.publish(Float64(data=float(math.radians(value*90))))

    def pitch(self,value):
        self.pitch_pub.publish(Float64(data=float(math.radians(value*30))))

class JoyToDualSense:
    class XY:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
        def __repr__(self):
            return str(vars(self))

    class AnalogButton:
        def __init__(self):
            self.analog = 0.0
            self.digital = False
            self.initial = True

        def update(self,digital,analog):
            self.digital = digital
            if self.digital == True:
                self.initial=False
            if self.initial == False:
                self.analog = (1.0-analog)/2.0
            else:
                self.analog = 0.0

        def value(self):
            return self.analog
        
        def __call__(self):
            return self.digital

        def __repr__(self):
            return str(vars(self))

    def __init__(self):
        self.up = False
        self.down = False
        self.left = False
        self.right = False

        self.cross = False
        self.circle = False
        self.triangle = False
        self.square = False

        self.create = False
        self.option = False

        self.L1 = False
        self.R1 = False
        self.L2 = self.AnalogButton()
        self.R2 = self.AnalogButton()

        self.L = self.XY()
        self.R = self.XY()
    
    def update(self, msg):
        # self.data = msg

        self.up = msg.axes[7] == 1.0
        self.down = msg.axes[7] == -1.0
        self.left = msg.axes[6] == 1.0
        self.right = msg.axes[6] == -1.0

        self.cross = msg.buttons[0]
        self.circle = msg.buttons[1]
        self.triangle = msg.buttons[2]
        self.square = msg.buttons[3]

        self.create = msg.buttons[8]
        self.option = msg.buttons[9]

        self.L1 = msg.buttons[4]
        self.R1 = msg.buttons[5]
        self.L2.update(msg.buttons[6], msg.axes[2])
        self.R2.update(msg.buttons[7], msg.axes[5])

        self.L.x = msg.axes[1]
        self.L.y = msg.axes[0]
        self.R.x = msg.axes[4]
        self.R.y = msg.axes[3]

def main():
    rclpy.init()
    node = ManualInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()