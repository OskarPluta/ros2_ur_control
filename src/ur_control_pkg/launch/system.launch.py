import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            FindPackageShare("ur_description"), "/urdf/ur.urdf.xacro",
            " ", "name:=ur", 
            " ", "ur_type:=ur5",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_node',
            parameters=[{'video_device': '/dev/video0'}]
        ),
        Node(
            package='ur_control_pkg',
            executable='ur_node',
            name='ur_controller'
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', '/vol/vol/123.rviz'],
        )
    ])
