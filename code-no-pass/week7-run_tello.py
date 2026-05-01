#!/usr/bin/env python
# -*- coding: utf-8 -*-

### Example program ###
### Uses simple_tello.py to create and control the Tello ###

import rospy                         # Import package: rospy
import simple_tello                  # Import simple_tello
from geometry_msgs.msg import Twist  # Import Twist message type from geometry_msgs
from std_msgs.msg import Empty       # Import Empty message type from std_msgs

# main: runs the full command sequence
def main():
  t1 = simple_tello.Tello_drone()    # Create Tello_drone instance t1

  while t1.state.is_flying == False: # Loop until takeoff is confirmed (is_flying becomes True)
    t1.controler.takeoff()

  while t1.state.fly_mode != 6:      # Wait until special actions complete (fly_mode == 6 means idle)
    print("wait...")

  t1.controler.flip(0)               # Perform a flip; requires sufficient battery

  while t1.state.fly_mode != 6:      # Wait until flip completes
    print("wait...")

  while t1.state.is_flying == True:  # Loop until landing is confirmed (is_flying becomes False)
    t1.controler.land()

# main
if __name__ == "__main__":

  # Register this script as a ROS node named 'run_tello'; anonymous=True adds a random suffix
  rospy.init_node("run_tello", anonymous=True)

  # Run main function
  main()
