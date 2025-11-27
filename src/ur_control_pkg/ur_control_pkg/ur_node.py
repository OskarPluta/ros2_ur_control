#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np

class URVisualController(Node):
    def __init__(self):
        super().__init__('ur_visual_controller')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        
        self.traj_publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.br = CvBridge()
        
        self.dictionaries_to_check = [
            cv2.aruco.DICT_6X6_250,
            cv2.aruco.DICT_5X5_100,
            cv2.aruco.DICT_4X4_50,
            cv2.aruco.DICT_APRILTAG_36h11
        ]
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detectors = []
        for dict_id in self.dictionaries_to_check:
            dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            detector = cv2.aruco.ArucoDetector(dictionary, self.detector_params)
            self.detectors.append(detector)

        self.joints = [0.0, -1.57, 2.0, -1.57, -0.5, 0.0]
        self.target_lift = 0.0

    def timer_callback(self):
        diff = self.target_lift - self.joints[0]
        self.joints[0] += diff * 0.1
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        joint_msg.position = self.joints
        self.joint_pub.publish(joint_msg)

        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_msg.name
        point = JointTrajectoryPoint()
        point.positions = self.joints
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000
        traj_msg.points = [point]
        self.traj_publisher_.publish(traj_msg)

    def image_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        height, width = frame.shape[:2]
        center_y = height // 2
        found_marker = False
        for detector in self.detectors:
            corners, ids, rejected = detector.detectMarkers(frame)
            
            if ids is not None:
                found_marker = True
                
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                c = corners[0][0]
                marker_x = int((c[0][0] + c[2][0]) / 2)
                marker_y = int((c[0][1] + c[2][1]) / 2)
                
                cv2.circle(frame, (marker_x, marker_y), 10, (0, 255, 0), -1)
                if marker_y < center_y:
                    self.target_lift = -1.0
                    
                else:
                    self.target_lift = -2.0
                    
                
                break 

        cv2.line(frame, (0, center_y), (width, center_y), (255, 0, 0), 1)
        if not found_marker:
             pass

        cv2.imshow("Universal ArUco Detector", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = URVisualController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
