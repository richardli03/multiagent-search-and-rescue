"""
Sample invocation:

ros2 launch neato_node2 bringup_multi.py host:=192.168.16.59 robot_name:=a udp_video_port:=5002 udp_sensor_port:=7777'
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
import json
import os


def generate_launch_description():
    robot_1 = {
        "name": "ally",
        "ip": "192.168.16.93",
        "video_port": "5011",
        "sensor_port": "7777",
    }
    robot_2 = {
        "name": "billy",
        "ip": "192.168.16.102",
        "video_port": "5013",
        "sensor_port": "7778",
    }

    robot_1_name = robot_1["name"]
    robot_1_host = robot_1["ip"]
    robot_1_video = robot_1["video_port"]

    robot_2_name = "billy"
    robot_2_host = robot_2["ip"]
    robot_3_name = "carson"

    interfaces_launch_file_dir = os.path.join(
        get_package_share_directory("search-and-rescue"), ""
    )

    return LaunchDescription(
        [
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            # "bringup_multi.py"
                            os.path.join(
                                interfaces_launch_file_dir, "bringup_multi.py"
                            )  # not sure if this path is correct, but uh
                        ),
                        launch_arguments={
                            "robot_name": robot_1_name,
                            "host": robot_1_host,  # insert correct IP
                            "udp_video_port": "5011",
                            "udp_sensor_port": "7777",
                            "gscam_config": "udpsrc port=5011 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert",
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            # "bringup_multi.py"
                            os.path.join(
                                interfaces_launch_file_dir, "bringup_multi.py"
                            )  # not sure if this path is correct, but uh
                        ),
                        launch_arguments={
                            "robot_name": robot_2_name,
                            "host": robot_2_host,  # insert correct IP
                            "udp_video_port": "5013",
                            "udp_sensor_port": "7779",
                            "gscam_config": "udpsrc port=5013 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert",
                        }.items(),
                    ),
                ]
            ),
        ]
    )
