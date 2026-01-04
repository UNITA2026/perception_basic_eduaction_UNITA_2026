from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 3) RPLIDAR C1 launch include
    rplidar_launch_path = os.path.join(
        get_package_share_directory("rplidar_ros"),
        "launch",
        "rplidar_c1_launch.py",
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path)
        # ⚠️ 여기서 rplidar launch에 argument를 넘기려면 DeclareLaunchArgument/LaunchConfiguration 방식이 필요함
    )

    # 1) Ultrasonic node
    ultrasonic = Node(
        package="ultrasonic_serial",
        executable="ultransonic_node",
        name="ultrasonic_node",
        output="screen",
    )

    # 2) Sound node  (Float32MultiArray /ultrasonic 구독하도록 파라미터 명시)
    sound = Node(
        package="sound_tts",
        executable="sound_node",
        name="sound_node",
        output="screen",
    )

    # 4) Scan cluster marker node
    scan_cluster = Node(
        package="lidar_obstacle_marker",
        executable="scan_cluster_marker",
        name="scan_cluster_marker",
        output="screen",
    )

    # RViz
    rviz_config = "/home/jkm/unita_ws/src/main_launch_pkg/rviz/main.rviz"


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        rplidar,
        ultrasonic,
        sound,
        scan_cluster,
        rviz,
    ])
