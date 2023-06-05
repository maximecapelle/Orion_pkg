#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hw_pub_node')
        self.pub = self.create_publisher(String, 'hello_world', qos_profile= 10) #QOS the queue size for if there is an unstable connection, it is the number of messages that are stored
        self.timer = self.create_timer(0.5, self.func_publish_hello_world) #Essentially creating a timer that runs the function every 0.5s
        self.counter = 0

    def func_publish_hello_world(self):
        msg = String() #Initializing the msg data type
        msg.data = "Hello World" + str(self.counter)#Defining the message data
        self.pub.publish(msg) # publishing the msg on the created publisher
        self.counter += 1


def main(args=None):
    rclpy.init()
    my_pub = HelloWorldPublisher()
    print("Publisher node is running...")

    try: 
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        print("Terminating Node")
        my_pub.destroy_node()


if __name__ == '__main__':    #This means that this file can be run as a script but isnt actually redefined in the other file.
    main()

