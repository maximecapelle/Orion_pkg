#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
import time
import numpy as np
from scripts import Init_Parameters as IP
from datetime import datetime

class UltrasonicSensorPub(Node):
    def __init__(self):
        super().__init__('USPub')
        self.US_start_flag = False
        self.pub = self.create_publisher(String, IP.US_TopicName, qos_profile= IP.qos_profile) # QOS the queue size for if there is an unstable connection,it is the number of messages that are stored
        self.sub = self.create_subscription(Joy, IP.JS_TopicName, self.subscriber_callback, IP.qos_profile)
        self.GPIO_TRIGGER, self.GPIO_ECHO, self.GPIO  = self.SetupUltraSonicSensor(IP.US_GPIO_TRIGGER, IP.US_GPIO_ECHO)  # Initiate the Ultrasonic sensor
        self.timer = self.create_timer(IP.US_PublishRate, self.PublishUltraSonicRange)  # Essentially creating a timer that runs the function every 0.5s
        self.NumReadings = IP.US_measurement_array_size                                  # Define the Number of UltraSonic Readings per print
        self.US_SampleFreq = IP.US_SampleFreq

    def SetupUltraSonicSensor(self, US_GPIO_TRIGGER, US_GPIO_ECHO):
        #GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)
        
        #set GPIO direction (IN / OUT)
        GPIO.setup(IP.US_GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(IP.US_GPIO_ECHO, GPIO.IN)
        
        return US_GPIO_TRIGGER, US_GPIO_ECHO, GPIO

    def PublishUltraSonicRange(self):
        if self.US_start_flag == True:
            msg = String()                                                                              # Initializing the msg data type
            Range = self.read_sensor(self.GPIO_TRIGGER, self.GPIO_ECHO, self.GPIO, self.NumReadings, self.US_SampleFreq)    # Call the Ultrasonic sensor reading function
            msg.data = "Current Range Measurement: " + str(Range)                                       # Defining the message data  
            self.pub.publish(msg)                                   	                                # publishing the msg on the created publisher
    
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

        
        if np.sum(button_array) >= 1:       #Checks if button was pressed
            idxs = np.asarray(np.where(button_array == 1)) #Gives indexs of buttons pressed
            
            

            if 7 in idxs and self.US_start_flag == False: #If Start is pressed
                
                epoch = time.time()
                # Convert timestamp to datetime object
                datetime_obj = datetime.fromtimestamp(epoch)

                # Format the datetime object as desired
                formatted_datetime = datetime_obj.strftime('%Y-%m-%d %H:%M:%S')
                
                print(f"Start was pressed: {formatted_datetime}")
                self.US_start_flag = True
        
            if 6 in idxs and self.US_start_flag == True:  #If Back is pressed
                
                epoch = time.time()
                # Convert timestamp to datetime object
                datetime_obj = datetime.fromtimestamp(epoch)

                # Format the datetime object as desired
                formatted_datetime = datetime_obj.strftime('%Y-%m-%d %H:%M:%S')
                
                print(f"Back was pressed: {formatted_datetime}")
                self.US_start_flag = False
            
        return self.US_start_flag
            

    def read_sensor(self, GPIO_TRIGGER, GPIO_ECHO, GPIO, NumReadings, US_SampleFreq):
        
        # Initialize range vector
        ranges = np.zeros((NumReadings))

        for idx in range(NumReadings):

            # set Trigger to HIGH
            GPIO.output(GPIO_TRIGGER, True)

            # set Trigger after 0.01ms to LOW
            time.sleep(US_SampleFreq)
            GPIO.output(GPIO_TRIGGER, False)

            StartTime = time.time()
            StopTime = time.time()

            # save StartTime
            while GPIO.input(GPIO_ECHO) == 0:
                StartTime = time.time()
            
            # save time of arrival
            while GPIO.input(GPIO_ECHO) == 1:
                StopTime = time.time()
        
            # time difference between start and arrival
            TimeElapsed = StopTime - StartTime
            # multiply with the sonic speed (34300 cm/s)
            # and divide by 2, because there and back
            ranges[idx] = (TimeElapsed * 34300) / 2
            
        print(np.median(ranges))
    
        return np.median(ranges)  


def main(args=None):
    rclpy.init()
    my_pub = UltrasonicSensorPub()
    print("UltrasonicSensor node is running...")

    try: 
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        print("Terminating Node")
        my_pub.destroy_node()


if __name__ == '__main__':    #This means that this file can be run as a script but isnt actually redefined in the other file.
    main()

