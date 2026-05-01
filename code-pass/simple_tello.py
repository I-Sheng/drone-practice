#!/usr/bin/env python
# -*- coding: utf-8 -*-

### Defines classes TelloState, TelloController, and Tello_drone ###
### Used for storing Tello info, publishing ROS messages, calling services to control the Tello, and handling ROS subscriptions ###

import rospy # Import package: rospy
import time  # Import package: time

from geometry_msgs.msg import Twist                       # Import Twist message type from geometry_msgs
from std_msgs.msg import Empty, UInt8, Float64MultiArray  # Import Empty, UInt8, Float64MultiArray from std_msgs
from time import sleep                                    # Import sleep from time
from tello_driver.msg import TelloStatus                  # Import TelloStatus from tello_driver

### class TelloState
### Stores information received from topic subscriptions
### Initial values can be adjusted as needed
class TelloState:

    # init function: initialize self variables
    def __init__(self):

        self.height = 0.0                 # Infrared height measurement
        self.temperature_height_m = 0.0   # Barometer height measurement
        self.battery = 100.0              # Battery level
        self.is_flying = False            # Whether the drone is currently flying
        self.fly_mode = 999               # Current flight mode
        self.target_x = -1                # Gate center x received from /target_point subscription
        self.target_y = -1                # Gate center y received from /target_point subscription
        self.canPass = -1                 # Mask area threshold flag: -1 = not yet received, 0 = below threshold, 1 = above threshold

### class TelloController
### Defines functions related to rospy publishers and serviceProxies
### Wrap publishing and service calls into functions to keep the main program clean
class TelloController:

    # move: controls Tello movement; twist holds linear/angular values, limitTime is the send duration
    def move(self, twist, limitTime):
        limitTime = limitTime * 1000 # Convert seconds to milliseconds
        startTime = int(round(time.time()*1000)) # Get current time as start time, convert to ms
        rate = rospy.Rate(10) # rate = 10: ROS tries to run the loop 10 times per second

        # Create publisher pub_move, publishing Twist to /tello/cmd_vel
        pub_move = rospy.Publisher("/tello/cmd_vel", Twist, queue_size = 10)

        # Run while loop; exits when roscore shuts down or break is hit
        while not rospy.is_shutdown():

          # Check connection count; > 0 means topic link is established before publishing
          connections = pub_move.get_num_connections()
          if connections > 0:
            endTime = int(round(time.time()*1000)) # Current time in ms
            if endTime - startTime < limitTime:    # Keep publishing within the time limit
              pub_move.publish(twist)              # Publish twist
            else:                                  # Time limit exceeded; exit loop
              break
            rate.sleep()                           # Sleep to maintain 10 Hz

    # emergency: immediately stops the drone's rotors; using in-air will cause it to drop
    def emergency(self):
        rate = rospy.Rate(10) # rate = 10: run loop 10 times per second

        # Create publisher puber, publishing Empty to /tello/emergency
        puber = rospy.Publisher("/tello/emergency", Empty, queue_size=10)

        # Run while loop; exits when roscore shuts down or break is hit
        while not rospy.is_shutdown():

          # Check connection count; > 0 means topic link is established
          cons = puber.get_num_connections()
          if cons > 0:
            puber.publish(Empty()) # Publish Empty via puber
            rate.sleep()           # Sleep to maintain 10 Hz
            break                  # Exit loop

    # takeoff: commands the Tello to take off
    def takeoff(self):
        rate = rospy.Rate(10) # rate = 10: run loop 10 times per second

        # Create publisher puber, publishing Empty to /tello/takeoff
        puber = rospy.Publisher("/tello/takeoff", Empty, queue_size=10)

        # Run while loop; exits when roscore shuts down or break is hit
        while not rospy.is_shutdown():

          # Check connection count; > 0 means topic link is established
          cons = puber.get_num_connections()
          if cons > 0:
            puber.publish(Empty()) # Publish Empty via puber
            rate.sleep()           # Sleep to maintain 10 Hz
            break                  # Exit loop

    # land: commands the Tello to land
    def land(self):
        rate = rospy.Rate(10) # rate = 10: run loop 10 times per second

        # Create publisher puber, publishing Empty to /tello/land
        puber = rospy.Publisher("/tello/land", Empty, queue_size=10)

        # Run while loop; exits when roscore shuts down or break is hit
        while not rospy.is_shutdown():

          # Check connection count; > 0 means topic link is established
          cons = puber.get_num_connections()
          if cons > 0:
            puber.publish(Empty()) # Publish Empty via puber
            rate.sleep()           # Sleep to maintain 10 Hz
            break                  # Exit loop

    # flip: commands the Tello to flip; i is an integer indicating flip direction
    def flip(self, i):
        rate = rospy.Rate(10) # rate = 10: run loop 10 times per second

        # Create publisher puber, publishing UInt8 to /tello/flip
        puber = rospy.Publisher("/tello/flip", UInt8, queue_size=10)

        # Run while loop; exits when roscore shuts down or break is hit
        while not rospy.is_shutdown():

          # Check connection count; > 0 means topic link is established
          cons = puber.get_num_connections()
          if cons > 0:
            puber.publish(UInt8(data=i)) # Publish UInt8 with flip direction as data=i
            rate.sleep()                 # Sleep to maintain 10 Hz
            break                        # Exit loop

### class Tello_drone()
### Creates the Tello_drone object
### Initializes TelloState and TelloController in __init__
### In _sensor(), subscribes to relevant topics to receive Tello flight info
class Tello_drone():
    def __init__(self):

        self.state = TelloState()          # Object that stores Tello flight info
        self.controler = TelloController() # Object with publisher/serviceProxy functions for Tello control
        self._sensor()                     # Call _sensor() to set up rospy subscribers

    # Sets up rospy subscribers
    def _sensor(self):
        _ts_sub = rospy.Subscriber("/tello/status", TelloStatus, self._ts_cb, queue_size = 10)        # Subscribe to /tello/status; TelloStatus format; callback = _ts_cb; queue = 10
        _tp_sub = rospy.Subscriber("/target_point", Float64MultiArray, self._tp_cb, queue_size = 10)  # Subscribe to /target_point; Float64MultiArray format; callback = _tp_cb; published by test_h264_sub.py

    # Callback for _ts_sub; receives Tello flight and sensor data from /tello/status
    def _ts_cb(self, data):

        # Store values into self.state
        self.state.height = data.height_m                            # Infrared height
        self.state.temperature_height_m = data.temperature_height_m  # Barometer height
        self.state.battery = data.battery_percentage                 # Battery level
        self.state.is_flying = data.is_flying                        # Is flying
        self.state.fly_mode = data.fly_mode                          # Current flight mode

    # Callback for _tp_sub; receives gate center and area threshold flag from /target_point
    def _tp_cb(self, data):

        # Store values into self.state
        self.state.target_x = data.data[0]  # Gate center x from /target_point
        self.state.target_y = data.data[1]  # Gate center y from /target_point
        self.state.canPass = data.data[2]   # Mask area threshold flag from /target_point
