#! usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sar_database_msgs.msg import WasteTarget
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
import tf2_geometry_msgs

class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter('robot_namespace', '')
        ns = self.get_parameter('robot_namespace').value
        image_topic = f'/{ns}/camera/image_raw' if ns else '/camera/image_raw'
        camera_info_topic = f'/{ns}/camera/camera_info' if ns else '/camera/camera_info'
        
        # Create subscription
        self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Create publisher
        self.detect_pub = self.create_publisher(
            WasteTarget,
            '/detections',
            10
        )

        # Converting the ros type image to CV
        self.bridge = CvBridge()

        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.aruco_detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # marker_image = cv.aruco.generateImageMarker(self.aruco_dict)

        # Creating a TF2 Buffer
        self.tf_buffer = Buffer()
        # Creating a TF2 Listener
        self.listener = TransformListener(self.tf_buffer, self)

        self.camera_matrix = None
        self.distortion_coefficients = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3,3))

        self.distortion_coefficients = np.array(msg.d)

        self.get_logger().info(f"Camera calibration received.")

    def image_callback(self, msg):

        if self.camera_matrix is None:
            return
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        corners, ids, _ = self.aruco_detector.detectMarkers(self.cv_image)

        # Estimate the pose of every detected marker
        marker_length = 0.05

        if ids is None:
            return

        # rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
        #     corners,
        #     marker_length,
        #     self.camera_matrix,
        #     self.distortion_coefficients
        # )

        # estimatePoseSingleMarkers was removed in newer OpenCV - use solvePnP instead
        half_len = marker_length / 2.0
        obj_points = np.array([
            [-half_len,  half_len, 0],
        [ half_len,  half_len, 0],
        [ half_len, -half_len, 0],
        [-half_len, -half_len, 0]
        ], dtype=np.float32)

        
        for i in range(len(ids)):
            # marker_id = ids[i][0]
            marker_id = int(ids.flatten()[i])
            # rvec = rvecs[i][0] # Rotation vector for this specific marker
            # tvec = tvecs[i][0] # Translation vector for this specific marker
            success, rvec, tvec = cv.solvePnP(
                obj_points,
                corners[i][0],
                self.camera_matrix,
                self.distortion_coefficients
            )
            if not success:
                continue
            tvec = tvec.flatten()
            map_pos = self.transform_to_map(tvec, msg.header.frame_id)

            if map_pos is None:
                continue

            self.publish_detection(marker_id , map_pos)

    def transform_to_map(self, transition, camera_frame):
        try:

            point_cam = PointStamped()
            point_cam.header.frame_id = camera_frame
            point_cam.header.stamp = self.get_clock().now().to_msg()

            point_cam.point.x = float(transition[0])
            point_cam.point.y = float(transition[1])
            point_cam.point.z = float(transition[2])

            point_map = self.tf_buffer.transform(
                point_cam,
                "map",
                timeout=Duration(seconds=1.0)
            )

            return (
                point_map.point.x,
                point_map.point.y,
                point_map.point.z
            )
        
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None
        
    def publish_detection(self,marker_id, map_position):
        msg = WasteTarget()

        msg.marker_id = str(marker_id)
        msg.position.x = map_position[0]
        msg.position.y = map_position[1]
        msg.position.z = map_position[2]

        msg.detected_by = self.get_parameter('robot_namespace').value
        msg.timestamp = self.get_clock().now().to_msg()
        msg.assigned = False
        msg.assigned_to = ""

        self.detect_pub.publish(msg)

        self.get_logger().info(f"Marker{msg.marker_id} was detected at"
                               f"{msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}"
                               )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
