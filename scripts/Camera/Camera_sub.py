#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from scripts import Init_Parameters as IP
import time
import datetime
import threading


class CameraImageSubNode(Node):
    def __init__(self):
        super().__init__("CameraImageSubNode")
        self.sub = self.create_subscription(Joy, IP.JS_TopicName, self.subscriber_callback, IP.qos_profile)
        self.CAM_start_flag = False
        self.thread = None
        self.is_running = False
        

    def DisplayCamera(self):
        cap = cv2.VideoCapture(IP.VideoCapture)
        cv2.namedWindow('WebCamera')  # Create a named window for the video display

        while True:
            while self.CAM_start_flag:
                ret, frame = cap.read()
                print(frame)
                cv2.imshow("Webcamera", frame)
            cv2.destroyAllWindows()

    def start_function(self):
        if not self.is_running:
            self.is_running = True
            self.thread = threading.Thread(target=self.run_DisplayCamera)
            self.thread.start()

    def run_DisplayCamera(self):
        while self.is_running:
            self.DisplayCamera()

    
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

    def CreateTimeStamp(self):
        epoch = time.time()
        # Convert timestamp to datetime object
        datetime_obj = datetime.fromtimestamp(epoch)

        # Format the datetime object as desired
        formatted_datetime = datetime_obj.strftime('%Y-%m-%d %H:%M:%S')

        return formatted_datetime 
        
        
        

def main():
    
    if IP.EnableJoystick or IP.EnableAll:
        rclpy.init()
        mysub = CameraImageSubNode()
        print(f"\nSubscriber Initialized, waiting for data publication.")

        try:
            mysub.start_function() 
            rclpy.spin(mysub)
        except KeyboardInterrupt:
            print("Terminating Node")
            mysub.destroy_node()

if __name__ == '__main__':    #This means that this file can be run as a script but isnt actually redefined in the other file.
    main()
