#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from scripts import Init_Parameters as IP
from rclpy.node import Node
import threading
import cv2
import numpy as np

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Create a separate thread for OpenCV GUI operations
        self.cv_thread = threading.Thread(target=self.cv_thread_function)
        self.cv_thread.daemon = True  # Exit the thread when the main program ends
        self.cv_thread.start()

    def cv_thread_function(self):
        cv2.namedWindow('Video')  # Create a named window for the video display

        video_capture = cv2.VideoCapture('path/to/video')  # Replace with your video source

        while True:
            # Read a frame from the video
            ret, frame = video_capture.read()

            # Display the frame in the OpenCV GUI thread
            cv2.imshow('Video', frame)
            cv2.waitKey(1)  # Required to process GUI events

        video_capture.release()
        cv2.destroyAllWindows()
    
    def subscriber_callback(self, msg):
        #Convert msg into numpy array
        button_array = np.array(msg.buttons)
        #If Button is pressed
        if np.sum(button_array) >= 1:

            #Converts inputs to Idxs
            idxs = np.asarray(np.where(button_array == 1))
        
            #If X is pressed
            if IP.JS_BUTTON_MAP['X'] in idxs and self.CAM_start_flag == False: 

                # formatted_datetime = self.CreateTimeStamp()
                # print(f"Start was pressed: {formatted_datetime}")
                self.CAM_start_flag = True

            #If BACK is pressed 
            if IP.JS_BUTTON_MAP['LB'] in idxs and self.CAM_start_flag == True:
                
                # formatted_datetime = self.CreateTimeStamp()
                # print(f"Back was pressed: {formatted_datetime}")
                self.CAM_start_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    # Create subscriptions, publishers, or timers as needed
    # For example:
    sub = node.create_subscription(Joy, IP.JS_TopicName, node.subscriber_callback, IP.qos_profile)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()