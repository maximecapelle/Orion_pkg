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
            executable='JoystickRead.py',
            name='InputReader',
            output= 'screen'
        ),
        Node(
            package='orion_pkg',
            executable='UltrasonicSensorReadings.py',
            name='USPublisher',
            output= 'screen'
        ),
        Node(
            package='orion_pkg',
            executable='UltrasonicSensorSub.py',
            name='USSubscriber',
            output= 'screen'
        ),
        Node(
            package='orion_pkg',
            executable='OpenCVWebcamera.py',
            name='CameraOpen',
            output= 'screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()