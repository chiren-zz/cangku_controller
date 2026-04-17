import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── 可配置参数（可在命令行覆盖）────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/myserial',
        description='STM32 串口设备路径')

    linear_scale_x_arg = DeclareLaunchArgument(
        'linear_scale_x', default_value='1.0',
        description='X 方向里程计校准系数')

    linear_scale_y_arg = DeclareLaunchArgument(
        'linear_scale_y', default_value='1.0',
        description='Y 方向里程计校准系数')

    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale', default_value='1.0',
        description='角速度里程计校准系数')

    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='true',
        description='是否发布 odom → base_footprint 的 TF 变换')

    imu_link_arg = DeclareLaunchArgument(
        'imu_link', default_value='imu_link',
        description='IMU 坐标系名称')

    # ── 节点1：硬件驱动（串口 ↔ STM32）────────────────
    driver_node = Node(
        package='yahboomcar_base_node',
        executable='Mcnamu_driver_X3',
        name='driver_node',
        output='screen',
        parameters=[{
            'serial_port':    LaunchConfiguration('serial_port'),
            'imu_link':       LaunchConfiguration('imu_link'),
            'xlinear_limit':  1.0,
            'ylinear_limit':  1.0,
            'angular_limit':  5.0,
        }]
    )

    # ── 节点2：轮式里程计（速度积分 → /odom_raw）──────
    odom_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        name='base_node',
        output='screen',
        parameters=[{
            'linear_scale_x':       LaunchConfiguration('linear_scale_x'),
            'linear_scale_y':       LaunchConfiguration('linear_scale_y'),
            'angular_scale':        LaunchConfiguration('angular_scale'),
            'pub_odom_tf':          LaunchConfiguration('pub_odom_tf'),
            'odom_frame':           'odom',
            'base_footprint_frame': 'base_footprint',
            'wheelbase':            0.25,
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        linear_scale_x_arg,
        linear_scale_y_arg,
        angular_scale_arg,
        pub_odom_tf_arg,
        imu_link_arg,
        driver_node,
        odom_node,
    ])