import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy

def generate_launch_description():
    """在一个线程池中启动 IMU、Lidar 和 Pose 节点."""

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
        name='pose_node',
        output='screen',
        emulate_tty=True,
    )

    # 创建 LaunchDescription
    launch_description = LaunchDescription([
        imu_launch,
        lidar_launch,
        pose_node
    ])

    return launch_description

def main():
    # 初始化 ROS 2 客户端库 (rclpy)
    rclpy.init()

    # 创建并配置 MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    # 获取 LaunchDescription 并启动节点
    launch_desc = generate_launch_description()
    launch_desc._generate_launch_actions()
    executor.add_node(launch_desc)

    try:
        # 执行 executor，这将保持所有节点运行
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭 ROS 2 系统
        rclpy.shutdown()

if __name__ == '__main__':
    main()
