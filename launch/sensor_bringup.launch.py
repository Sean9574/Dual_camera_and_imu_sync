from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

display      = os.environ.get('DISPLAY', 'localhost:10.0')
scripts_path = os.path.expanduser('~/ros2_ws/src/igvc_camstuff/igvc_camstuff')

def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_path, 'setup_trigger.py')],
            output='screen'
        ),

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

        TimerAction(period=2.0, actions=[
            Node(
                package='igvc_camstuff',
                executable='camera_node',
                name='camera_node',
                output='screen',
                parameters=[{
                    'device_left':  0,
                    'device_right': 2,
                    'fps':          30,
                    'width':        1280,
                    'height':       800,
                }]
            ),
        ]),

        TimerAction(period=4.0, actions=[
            Node(
                package='igvc_camstuff',
                executable='camera_display_node',
                name='camera_display_node',
                output='screen',
                additional_env={'DISPLAY': display},
            ),
        ]),

        TimerAction(period=5.0, actions=[
            Node(
                package='igvc_camstuff',
                executable='imu_monitor',
                name='imu_monitor',
                output='screen',
            ),
        ]),

    ])
