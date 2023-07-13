#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from scripts import Init_Parameters as IP

class Joystick_Command_Interpreter(Node):
    def __init__(self):
        super().__init__("Joystick_Interpreter_Node")
        self.sub = self.create_subscription(Joy, IP.JS_TopicName, self.subscriber_callback, IP.qos_profile)
    
    def subscriber_callback(self, msg):

        #Convert msg into numpy array
        button_array = np.array(msg.buttons)

        #Checks if button was pressed, if yes, prints equivalent letter(s)
        if np.sum(button_array) >= 1:
            idxs = np.asarray(np.where(button_array == 1))
            text = ""
            for idx in idxs[0]:
                text = text + f" {IP.JS_ButtonNameConversion[idx]} +"
            print(f"\n\n YOU PRESSED: {text[0:-2]}")
            

def main(args=None):
    
    if IP.EnableJoystick or IP.EnableAll:
        rclpy.init()
        JoystickReader = Joystick_Command_Interpreter()
        print(f"\nSubscriber Initialized, waiting for data publication.")

        try: 
            rclpy.spin(JoystickReader)
        except KeyboardInterrupt:
            print("Terminating Node")
            JoystickReader.destroy_node()

if __name__ == '__main__':    #This means that this file can be run as a script but isnt actually redefined in the other file.
    main()
