# ROS2 UR5 Simulation

This package provides a fully dockerized ROS2 simulation featuring the UR5 robot with visualization through RViz. The simulation launches automatically.

## Getting Started

### Build the Docker Image

```bash
docker build -t ros2_ur_project .
# Run the Simulation
./run.sh
```

# Using ArUco Markers

Once the simulation is running, you can control the robot's lateral movement (left and right) by presenting an ArUco marker. The package supports the following marker dictionaries:

```python
self.dictionaries_to_check = [
    cv2.aruco.DICT_6X6_250,
    cv2.aruco.DICT_5X5_100,
    cv2.aruco.DICT_4X4_50,
    cv2.aruco.DICT_APRILTAG_36h11
]
```