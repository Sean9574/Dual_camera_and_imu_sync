import os

from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription

home         = os.path.expanduser('~')
scripts_path = os.path.join(home, 'ros2_ws/src/igvc_camstuff/igvc_camstuff')
config_path  = os.path.join(home, 'ros2_ws/src/igvc_camstuff/openvins_config/estimator_config.yaml')
rviz_config  = os.path.join(home, 'ros2_ws/src/igvc_camstuff/rviz/vio.rviz')


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('vio',  default_value='true',  description='launch OpenVINS VIO'),
        DeclareLaunchArgument('rviz', default_value='true',  description='launch RViz2'),

        # Arm both cameras into external hardware-trigger mode
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_path, 'setup_trigger.py')],
            output='screen'
        ),

        # Fix camera exposure: disable dynamic framerate, set manual low exposure
        ExecuteProcess(
            cmd=['v4l2-ctl', '-d', '/dev/video5',
                 '-c', 'exposure_dynamic_framerate=0',
                 '-c', 'exposure_time_absolute=100'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['v4l2-ctl', '-d', '/dev/video7',
                 '-c', 'exposure_dynamic_framerate=0',
                 '-c', 'exposure_time_absolute=100'],
            output='screen'
        ),

        # IMU serial driver (also publishes /camera/trigger on flagged samples)
        Node(package='igvc_camstuff', executable='imu_serial_node',
             name='imu_serial_node', output='screen',
             parameters=[{'port': '/dev/ttyACM0', 'baud': 115200}]),

        # Stereo camera capture (GStreamer, deferred-gi)
        TimerAction(period=2.0, actions=[
            Node(package='igvc_camstuff', executable='camera_node',
                 name='camera_node', output='screen',
                 parameters=[{'width': 1280, 'height': 800}]),
        ]),

        # Compressed -> image_raw decode for VIO
        TimerAction(period=3.0, actions=[
            Node(package='igvc_decode_cpp', executable='stereo_decode',
                 name='stereo_decode', output='screen'),
        ]),

        # OpenVINS stereo-inertial VIO (after sensors are up). Disable: vio:=false
        TimerAction(period=6.0, actions=[
            Node(package='ov_msckf', executable='run_subscribe_msckf',
                 name='ov_msckf', output='screen',
                 condition=IfCondition(LaunchConfiguration('vio')),
                 parameters=[{'config_path': config_path}]),
        ]),

        # RViz2 with the VIO view preloaded. Disable: rviz:=false
        TimerAction(period=6.0, actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2',
                 output='screen',
                 condition=IfCondition(LaunchConfiguration('rviz')),
                 arguments=['-d', rviz_config]),
        ]),

    ])