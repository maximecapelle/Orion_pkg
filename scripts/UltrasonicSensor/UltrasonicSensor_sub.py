#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 
from scripts import Init_Parameters as IP


class UltraSonicSubscriber(Node):
    def __init__(self):
        super().__init__("Ultrasonic_Sub")
        self.sub = self.create_subscription(String, IP.US_TopicName, self.subscriber_callback, IP.qos_profile)
    
    def subscriber_callback(self, msg):
        print("UltraSonic Subscriber Recieved: " + msg.data)      


def main(args=None):

    if IP.EnableUS or IP.EnableAll:
        rclpy.init()
        my_sub = UltraSonicSubscriber()
        print("Waiting for data to be published...")

        try: 
            rclpy.spin(my_sub)
        except KeyboardInterrupt:
            print("Terminating Node")
            my_sub.destroy_node()


if __name__ == '__main__':    #This means that this file can be run as a script but isnt actually redefined in the other file.
    main()
