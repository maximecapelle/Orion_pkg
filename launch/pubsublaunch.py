import launch
from launch_ros.actions import Node
import sys
sys.stdout.flush()

def generate_launch_description():
    return launch.LaunchDescription([
        # Node(
        #     package='orion_pkg',
        #     executable='publisher.py',
        #     output = 'screen',
        #     name='pub'),
        # Node(
        #     package='orion_pkg',
        #     executable='subscriber.py',
        #     name='sub',
        #     output= 'screen'
        # ),
        Node(
            package='orion_pkg',
            executable='joystick_commands.py',
            name='joystick',
        ),
        Node(
            package='orion_pkg',
            executable='Joystick_sub.py',
            name='Joy_sub',
            output= 'screen'
        ),
        Node(
            package='orion_pkg',
            executable='Ps4Read.py',
            name='InputReader',
            output= 'screen'
        ),
    ])

