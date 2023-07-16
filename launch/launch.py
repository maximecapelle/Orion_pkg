import launch
from launch_ros.actions import Node
import sys
# sys.stdout.flush()

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
            executable='Joystick_sub.py',
            name='Joy_sub',
            output= 'screen'
        ),
        Node(
            package='orion_pkg',
            executable='Joystick_pub.py',
            name='InputReader',
            output= 'screen'
        ),
        # Node(
        #     package='orion_pkg',
        #     executable='UltrasonicSensor_pub.py',
        #     name='USPublisher',
        #     output= 'screen'
        # ),
        # Node(
        #     package='orion_pkg',
        #     executable='UltrasonicSensor_sub.py',
        #     name='USSubscriber',
        #     output= 'screen'
        # ),
        Node(
            package='orion_pkg',
            executable='CameraDisplay.py',
            name='CameraDisplay',
            output= 'screen'
        ),
        # Node(
        #     package='orion_pkg',
        #     executable='VideoStream.py',
        #     name='VideoStream',
        #     output= 'screen'
        # ),
        # Node(
        #     package='orion_pkg',
        #     executable='Camera_sub.py',
        #     name='CameraDisplay',
        #     output= 'screen'
        # ),
    ])

if __name__ == '__main__':
    generate_launch_description()