from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

scripts_path = os.path.expanduser('~/ros2_ws/src/igvc_camstuff/igvc_camstuff')

def generate_launch_description():
    return LaunchDescription([

        # Arm both cameras into external hardware-trigger mode
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_path, 'setup_trigger.py')],
            output='screen'
        ),

        # IMU serial driver (also publishes /camera/trigger on flagged samples)
        Node(
            package='igvc_camstuff',
            executable='imu_serial_node',
            name='imu_serial_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 115200,
            }]
        ),

        # Stereo camera node (starts after IMU so trigger stamps are flowing)
        TimerAction(period=2.0, actions=[
            Node(
                package='igvc_camstuff',
                executable='camera_node',
                name='camera_node',
                output='screen',
                parameters=[{
                    'device_left':  0,
                    'device_right': 2,
                    'width':        1280,
                    'height':       800,
                }]
            ),
        ]),

        # IMU console monitor (optional; comment out to save ~70MB RAM for VIO)
        TimerAction(period=5.0, actions=[
            Node(
                package='igvc_camstuff',
                executable='imu_monitor',
                name='imu_monitor',
                output='screen',
            ),
        ]),

    ])
