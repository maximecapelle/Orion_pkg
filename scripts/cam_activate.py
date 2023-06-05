#!/usr/bin/env python3
from picamera import PiCamera
from time import sleep
import numpy as np
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 

#Button position in array
ButtonNameConversion = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'EMPTY', 'BACK', 'START', 'HOME', 'L3', 'R3']

class Camera_Activation(Node):
    def __init__(self):
        super().__init__("CameraNode")
        self.camera = False
        self.sub = self.create_subscription(String, "joy" , self.subscriber_callback, 10)
    
    def subscriber_callback(self, msg):
        #Convert msg into numpy array
        button_array = np.array(msg.buttons)

        #Checks if button was pressed, if yes, prints equivalent letter(s)
        if self.camera == True and button_array[8] == 1:
            self.camera = True
            print(" ..... Camera Activated .....")
            camera = PiCamera()

            camera.start_preview()
        
        if self.camera == True and button_array[9] == 1:
            camera.stop_preview()
            print(" ..... Camera Deactivated .....")

        return


def main(args=None):
    rclpy.init()
    Cam_Sub = Camera_Activation()
    print("Waiting for data to be published...")

    try: 
        rclpy.spin(Cam_Sub)
    except KeyboardInterrupt:
        print("Terminating Node")
        Cam_Sub.destroy_node()


if __name__ == '__main__':    #This means that this file can be run as a script but isnt actually redefined in the other file
    main()