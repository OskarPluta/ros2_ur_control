The package runs a ROS2 simulation using rvizz, ur5 robot. It is fully dockerized and it launches automatically. You just need to build docker with "docker build -t ros2_ur_project ." and then launch it with ./run.sh

When presenting an ArUco marker (within  these
        self.dictionaries_to_check = [
            cv2.aruco.DICT_6X6_250,
            cv2.aruco.DICT_5X5_100,
            cv2.aruco.DICT_4X4_50,
            cv2.aruco.DICT_APRILTAG_36h11
        ]
), you will be able to move the robot right and left. 