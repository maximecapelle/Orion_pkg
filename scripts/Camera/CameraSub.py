#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from scripts import Init_Parameters as IP


class CameraImageSubNode(Node):
    def __init__(self):
        super().__init__("CameraImageSubNode")
        self.sub = self.create_subscription(Image, IP.CAM_TopicName, self.subscriber_callback, IP.qos_profile)
    
    def subscriber_callback(self, Image):
        img = Image
        
        # .... Find out how to make it show with imshow in docker container ....
        
        

def main(args=None):
    
    if IP.EnableJoystick or IP.EnableAll:
        rclpy.init()
        mysub = CameraImageSubNode()
        print(f"\nSubscriber Initialized, waiting for data publication.")

        try: 
            rclpy.spin(mysub)
        except KeyboardInterrupt:
            print("Terminating Node")
            mysub.destroy_node()

if __name__ == '__main__':    #This means that this file can be run as a script but isnt actually redefined in the other file.
    main()
