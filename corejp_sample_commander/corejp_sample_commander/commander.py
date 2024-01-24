import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Int64, Empty, Bool

class Auto(Node):

    def __init__(self):
        super().__init__('auto')

        self.subscription = self.create_subscription(Int64,'/auto',self.callback,10)

        self.pattern = [
                        FireTask(0,0,2000),
                        FireTask(-20,0,2000),
                        FireTask(-40,-8,2000),
                        FireTask(-60,10,2000),
                        FireTask(-80,25,2000),
                        FireTask(80,-8,2000),
                        FireTask(80,30,2000),
                        FireTask(50,10,2000),
                        FireTask(10,-5,2000),
                        FireTask(0,0,0),
                        ]
        self.index = 0

        self.hammer_pub = self.create_publisher(Empty, "/hammer", 10)
        self.mazemaze_pub = self.create_publisher(Empty, "/mazemaze", 10)
        self.roller_pub = self.create_publisher(Float64, "/roller", 10)
        self.control_pub = self.create_publisher(Bool, "/control", 10)
        self.yaw_pub = self.create_publisher(Float64, "/yaw", 10)
        self.pitch_pub = self.create_publisher(Float64, "/pitch", 10)

        self.turret_sub = self.create_subscription(Vector3,'/current_turret_pose',self.turret_callback,10)
        self.target_sub = self.create_subscription(Vector3,'/target',self.target_callback,10)

        self.task_timer =  self.create_timer(2, self.timer_callback)
        self.task_timer.cancel()

        self.turret_data = Vector3()
        self.target_data = Vector3()

        self.AUTO_MODE_MANUAL   = 0
        self.AUTO_MODE_TRACKING = 1
        self.AUTO_MODE_SEQUENCE = 2
        self.auto_mode = self.AUTO_MODE_MANUAL

    def callback(self, msg):
        self.auto_mode = msg.data

        if self.auto_mode == self.AUTO_MODE_SEQUENCE:
            if self.index == 0:
                self.run_task()
                self.task_timer.reset()
        else:
            self.index = 0
            self.task_timer.cancel()

    def turret_callback(self, msg):
        self.turret_data = msg

    def target_callback(self, msg):
        self.target_data = msg
        yaw = self.target_data.z + self.turret_data.z
        pitch = self.target_data.y + self.turret_data.y
        if self.auto_mode == self.AUTO_MODE_TRACKING:
            self.yaw_pub.publish(Float64(data=float(yaw)))
            self.pitch_pub.publish(Float64(data=float(pitch)))
            if math.degrees(self.target_data.z) <= 5 and math.degrees(self.target_data.y<=5):
                self.roller_pub.publish(Float64(data=float(4000)))
                self.hammer_pub.publish(Empty())
                self.mazemaze_pub.publish(Empty())

    def run_task(self):
            task = self.pattern[self.index]
            self.yaw_pub.publish(Float64(data=float(task.yaw)))
            self.pitch_pub.publish(Float64(data=float(task.pitch)))
            self.roller_pub.publish(Float64(data=float(task.speed)))
            if task.speed != 0:
                self.hammer_pub.publish(Empty())
                self.mazemaze_pub.publish(Empty())
            else:
                pass

            self.index += 1

    def timer_callback(self):
        if self.index != 0:
            self.run_task()
        if self.index >= len(self.pattern):
            self.task_timer.cancel()

class FireTask:
    def __init__(self,yaw,pitch,speed):
        self.yaw = math.radians(yaw)
        self.pitch = math.radians(pitch)
        self.speed = speed

def main():
    rclpy.init()
    node = Auto()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()