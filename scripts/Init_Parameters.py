#!/usr/bin/env python3

###############################
## General
###############################

# Queue size for unstable connections
qos_profile = 10                    # [-]

#Activation of compontents
EnableCamera =      True
EnableUS =          False
EnableJoystick =    True
EnableLCD =         False
EnableMotors =      False
EnableAll =         False




###############################
## Camera
###############################

#Video Channel for reaching USB camera
VideoCapture = 0
CAM_PublishRate = 0.0333  # [sec] Roughly 30fps
CAM_TopicName = "CameraFeed"





###############################
## Joystick
###############################

# Positions of the letters in the buttons array [NEEDS TO BE CORRECTED]

# Button Conversion string
JS_ButtonNameConversion = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START', 'HOME', 'R3', 'L3', 'LAURA']

# Publishing Parameter\
# Topic Name
JS_TopicName = "joystick_inputs"    # [-]
# Time step per node call of joystick inputs
JS_PublishRate = 0.1                # [sec]

# Logitech Gamepad F710 Mapping
JS_BUTTON_MAP = {
    'A': 0,
    'B': 1,
    'X': 2,
    'Y': 3,
    'LB': 4,
    'RB': 5,
    'BACK': 6,
    'START': 7,
    'HOME': 8,
    'R3': 9,
    'L3': 10,
    'LAURA': 11
}




###############################
## UltraSonic Sensor
###############################

# Time step per node call of US_Sensor_Node 
US_PublishRate = 1                  # [sec]
# Topic Name
US_TopicName = 'UltraSonicSensor'   # [-]
# Number of measurements medianed per node call of US Sensor
US_measurement_array_size = 10      # [-]
# Set US Sensor GPIO Pins
US_GPIO_TRIGGER = 18               # [-]    
US_GPIO_ECHO = 24                # [-]
# Time step between US sensor triggering
US_SampleFreq = 0.01                # [sec]


#############################
## Functions
#############################
