import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Launches IMU, Lidar, and Pose nodes."""

    # 获取 imu 包的共享目录路径
    imu_package_share_dir = get_package_share_directory('imu')
    # 获取 lidar 包的共享目录路径
    lidar_package_share_dir = get_package_share_directory('lidar')

    # 包含 imu.launch.py
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_package_share_dir, 'launch', 'imu.launch.py')
        )
    )

    # 包含 ld14p.launch.py
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_package_share_dir, 'launch', 'ld14p.launch.py')
        )
    )

    # 运行 pose 包的 main 节点
    pose_node = Node(
        package='pose',
        executable='main',
        name='pose_node', # 你可以给节点取一个自定义的名字
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        imu_launch,
        lidar_launch,
        pose_node
    ])