from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
import image_geometry

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator


class StopSignDetector(Node):

    def __init__(self):
        super().__init__('stop_sign_detector')

        self.model = YOLO('yolov8n.pt')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        self.camera_model = None
        self.sub_camera_info = self.create_subscription(CameraInfo, 'camera/color/camera_info', self.camera_info_callback, 10)

        self.publisher_ = self.create_publisher(Vector3, '/target', 10)

    def camera_info_callback(self, msg):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def image_callback(self, msg):
        if self.camera_model is None:
            print("waiting for camera_model")
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        result = self.model.predict(source=img)[0]
        annotated = deepcopy(img)

        annotator = Annotator(annotated, line_width=1, font_size=10)
        
        angle_x, angle_y = None, None

        for i in range(len(result.boxes.cls)):
            label = result.names[int(result.boxes.cls[i])]
            xyxy = result.boxes.xyxy[i]
            if label == "stop sign":                
                center_x = float(xyxy[0]+xyxy[2])/2
                center_y = float(xyxy[1]+xyxy[3])/2
                angle_x, angle_y, _ = self.camera_model.projectPixelTo3dRay((center_x, center_y))
            annotator.box_label(box=xyxy, label=label, color=(0, 0, 255), txt_color=(255, 255, 255))
        annotator.result()

        if angle_x is not None:
            self.publisher_.publish(Vector3(x=0.0, y=(-angle_y), z=(-angle_x)))

        # cv2.imshow('original', img)
        # cv2.imshow('annotated', annotated)
        # cv2.waitKey(1)


def main():
    rclpy.init()
    node = StopSignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
