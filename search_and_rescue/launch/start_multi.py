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
        "ip": "192.168.16.64",
        "video_port": "5011",
        "sensor_port": "7777",
    }
    robot_2 = {
        "name": "billy",
        "ip": "192.168.16.55",
        "video_port": "5013",
        "sensor_port": "7778",
    }

    interfaces_launch_file_dir = os.path.join(
        get_package_share_directory("search_and_rescue"), ""
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
                            "robot_name": robot_1["name"],
                            "host": robot_1["ip"],  # insert correct IP
                            "udp_video_port": robot_1["video_port"],
                            "udp_sensor_port": robot_1["sensor_port"],
                            "gscam_config": f"udpsrc port={robot_1['video_port']} ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert",
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
                            "robot_name": robot_2["name"],
                            "host": robot_2["ip"],  # insert correct IP
                            "udp_video_port": robot_2["video_port"],
                            "udp_sensor_port": robot_2["sensor_port"],
                            "gscam_config": f"udpsrc port={robot_2['video_port']} ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert",
                        }.items(),
                    ),
                    Node(
                        package="search_and_rescue",
                        executable="agent",
                        name="mvmt",
                        remappings=[
                            ("/cmd_vel", "ally/cmd_vel"),
                            ("/odom", "ally/odom"),
                        ],
                        output={
                            "stdout": "screen",
                            "stderr": "screen",
                        },
                    ),
                    Node(
                        package="search_and_rescue",
                        executable="agent",
                        name="mvmt2",
                        remappings=[
                            ("/cmd_vel", "billy/cmd_vel"),
                            ("/odom", "billy/odom"),
                        ],
                        output={
                            "stdout": "screen",
                            "stderr": "screen",
                        },
                    ),
                ]
            ),
        ]
    )
