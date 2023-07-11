#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from scripts import Init_Parameters as IP

# Logitech Gamepad F710 Mapping
F710_BUTTON_MAP = {
    'A': 0,
    'B': 1,
    'X': 2,
    'Y': 3,
    'LB': 4,
    'RB': 5,
    'EMPTY': 6,
    'BACK': 7,
    'START': 8,
    'HOME': 9,
    'L3': 10,
    'R3': 11
}



class Joystick_Command_Interpreter(Node):
    def __init__(self):
        super().__init__("Joystick_Interpreter_Node")
        self.sub = self.create_subscription(Joy, IP.JS_TopicName, self.subscriber_callback, IP.qos_profile)
    
    def subscriber_callback(self, msg):
        '''
        Formats the Joystick input: Axes, Buttons. 

        Prints the buttons that you pressed in letter format. 
        Will be used to check the history of inputs.

        Args:

            msg:
                The "sensor_msgs.msg.Joy" input which is split into:
                    Header header           # timestamp in the header is the time the data is received from the joystick
                    float32[] axes          # the axes measurements from a joystick
                    int32[] buttons         # the buttons measurements from a joystick 
        
        Returns:
            None

        Prints:
            "YOU PRESSED: {LETTER BUTTON NAME}"
        '''
        
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
