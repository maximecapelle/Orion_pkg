#!/usr/bin/env python3

import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from scripts import Init_Parameters as IP

class CameraImagePubNode(Node):
    def __init__(self):
        super().__init__('CameraImagePubNode')
        self.pub = self.create_publisher(Image, IP.CAM_TopicName, qos_profile= IP.qos_profile)
        CameraFeed = self.FormatImage()
    

    def FormatImage(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            print(frame)
            cv2.imshow("Webcamera", frame)
        
def main(args=None):
    if IP.EnableCamera or IP.EnableAll:
        rclpy.init()
        my_pub = CameraImagePubNode()
        print("Camera opened successfully ...")

        try: 
            rclpy.spin(my_pub)
        except KeyboardInterrupt:
            print("Terminating Camera Node")
            my_pub.destroy_node()


if __name__ == '__main__':    #This means that this file can be run as a script but isnt actually redefined in the other file.
    main()