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
import os


def generate_launch_description():
    use_udp = DeclareLaunchArgument("use_udp", default_value="true")
    host_1 = DeclareLaunchArgument("host1", default_value="192.168.16.93")
    host_2 = DeclareLaunchArgument("host2", default_value="192.168.16.95")

    udp_video_port_1 = LaunchConfiguration("udp_video_port_1")
    udp_video_port_2 = LaunchConfiguration("udp_video_port_2")
    gscam_config_1 = DeclareLaunchArgument(
        "gscam_config",
        default_value=[
            "udpsrc port=",
            udp_video_port_1,
            " ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert",
        ],
    )
    gscam_config_2 = DeclareLaunchArgument(
        "gscam_config",
        default_value=[
            "udpsrc port=",
            udp_video_port_2,
            " ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert",
        ],
    )
    udp_video_port_1_command = DeclareLaunchArgument(
        "udp_video_port_1", default_value="5011"
    )
    udp_video_port_2_command = DeclareLaunchArgument(
        "udp_video_port_2", default_value="5013"
    )
    # udp_video_port = LaunchConfiguration("udp_video_port")
    # udp_video_port_command = DeclareLaunchArgument(
    #     "udp_video_port", default_value="5002"
    # )

    udp_sensor_port_1 = DeclareLaunchArgument("udp_sensor_port_1", default_value="7777")
    udp_sensor_port_2 = DeclareLaunchArgument("udp_sensor_port_2", default_value="7778")
    robot_name_1 = LaunchConfiguration("robot_name_1")
    robot_name_1_command = DeclareLaunchArgument(
        "robot_name", default_value=f"{robot_1_name}"
    )
    robot_name_2 = LaunchConfiguration("robot_name_2")
    robot_name_2_command = DeclareLaunchArgument(
        "robot_name", default_value=f"{robot_2_name}"
    )

    interfaces_launch_file_dir = os.path.join(
        get_package_share_directory("search-and-rescue"), ""
    )

    robot_1_name = "alice"
    robot_2_name = "billy"

    robot_3_name = "carson"

    return LaunchDescription(
        [
            use_udp,
            host_1,
            host_2,
            udp_sensor_port_1,
            udp_sensor_port_2,
            udp_video_port_1_command,
            udp_video_port_2_command,
            gscam_config_1,
            gscam_config_2,
            robot_name_1_command,
            robot_name_2_command,
            GroupAction(
                actions=[
                    # ONE OF THESE PER NEATO
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            # "bringup_multi.py"
                            os.path.join(
                                interfaces_launch_file_dir, "bringup_multi.py"
                            )  # not sure if this path is correct, but uh
                        ),
                        launch_arguments={
                            "robot_name": robot_1_name,
                            "host": host_1,  # insert correct IP
                            "udp_video_port": udp_video_port_1_command,
                            "udp_sensor_port": udp_sensor_port_1,
                            "gscam_config": gscam_config_1,
                        }.items(),
                    ),
                    # ONE OF THESE PER NEATO
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            # "bringup_multi.py"
                            os.path.join(
                                interfaces_launch_file_dir, "bringup_multi.py"
                            )  # not sure if this path is correct, but uh
                        ),
                        launch_arguments={
                            "robot_name": robot_2_name,
                            "host": host_2,  # insert correct
                            "udp_video_port": udp_video_port_2_command,
                            "udp_sensor_port": udp_sensor_port_2,
                            "gscam_config": gscam_config_2,
                        }.items(),
                    ),
                    # ONE OF THESE PER NEATO
                    # IncludeLaunchDescription(
                    #     PythonLaunchDescriptionSource(
                    #         os.path.join(
                    #             interfaces_launch_file_dir, "/bringup_multi.py"
                    #         )  # not sure if this path is correct, but uh
                    #     ),
                    #     launch_arguments={
                    #         "robot_name": robot_3_name,
                    #         "host": 111111, #insert correct host
                    #         "udp_video_port": 5001,
                    #         "udp_sensor_port": 5002,
                    #     }.items(),
                    # ),
                ]
            ),
        ]
    )
