import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── 可配置参数 ────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/myserial',
        description='STM32 串口设备路径')

    linear_scale_x_arg = DeclareLaunchArgument(
        'linear_scale_x', default_value='1.0')
    linear_scale_y_arg = DeclareLaunchArgument(
        'linear_scale_y', default_value='1.0')
    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale', default_value='1.0')
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='true')
    imu_link_arg = DeclareLaunchArgument(
        'imu_link', default_value='imu_link')

    # ── 节点1：硬件驱动 ───────────────────────────
    driver_node = Node(
        package='yahboomcar_base_node',
        executable='Mcnamu_driver_X3',
        name='driver_node',
        output='screen',
        parameters=[{
            'serial_port':   LaunchConfiguration('serial_port'),
            'imu_link':      LaunchConfiguration('imu_link'),
            'xlinear_limit': 1.0,
            'ylinear_limit': 1.0,
            'angular_limit': 5.0,
        }]
    )

    # ── 节点2：轮式里程计 ─────────────────────────
    odom_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        name='base_node',
        output='screen',
        parameters=[{
            'linear_scale_x': ParameterValue(
                LaunchConfiguration('linear_scale_x'), value_type=float),
            'linear_scale_y': ParameterValue(
                LaunchConfiguration('linear_scale_y'), value_type=float),
            'angular_scale': ParameterValue(
                LaunchConfiguration('angular_scale'), value_type=float),
            'pub_odom_tf': ParameterValue(
                LaunchConfiguration('pub_odom_tf'), value_type=bool),
            'odom_frame':           'odom',
            'base_footprint_frame': 'base_footprint',
            'wheelbase':            0.25,
        }]
    )

    # ── 节点3：手柄输入 ───────────────────────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id':       0,       # js0 = Controller（你的手柄）
            'deadzone':        0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    # ── 节点4：手柄 → cmd_vel 转换 ────────────────
    # !! 使用时必须一直按住 LB（button 4）才能发送指令 !!
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[{
            # 左摇杆上下 → 前后（linear.x）
            'axis_linear.x':        1,
            'scale_linear.x':       0.3,
            'scale_linear_turbo.x': 0.6,

            # 右摇杆左右 → 横移（linear.y，麦克纳姆专有）
            'axis_linear.y':        3,
            'scale_linear.y':       0.3,
            'scale_linear_turbo.y': 0.6,

            # 左摇杆左右 → 转向（angular.yaw）
            'axis_angular.yaw':        0,
            'scale_angular.yaw':       1.0,
            'scale_angular_turbo.yaw': 2.0,

            # LB = 死人开关（必须按住），RB = 加速
            'enable_button':         4,
            'enable_turbo_button':   5,
            'require_enable_button': False,
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
        joy_node,
        teleop_node,
    ])
