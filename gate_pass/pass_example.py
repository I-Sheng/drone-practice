#!/usr/bin/env python
# -*- coding: utf-8 -*-

### Example program ###
### Uses test_h264_sub.py to find the gate center, then controls movement to align ###
### the drone's image center (480, 200) with the detected gate center ###
### Once aligned, the drone moves forward ###
### When canPass == 1, the vision node considers the drone close enough to the gate; ###
### at this point the drone accelerates through and the control loop exits ###
### Uses simple_tello.py to create and control the Tello ###

import rospy                         # Import package: rospy
import simple_tello                  # Import simple_tello
from geometry_msgs.msg import Twist  # Import Twist message type from geometry_msgs
from std_msgs.msg import Empty       # Import Empty message type from std_msgs

# tello_pass: runs the main control loop
# The loop adjusts movement based on target_x, target_y, and canPass
# target_x: gate center x received from subscribed topic
# target_y: gate center y received from subscribed topic
# canPass: flag indicating whether the mask area exceeds the threshold
# Narrow threshold = 24px: |target_x - 480| < 24 and |target_y - 200| < 24 means aligned
# Wide threshold: 60px on x-axis, 30px on y-axis — if exceeded while aligned, re-correction is needed
def tello_pass(t1):

  # check: tracks whether the drone is currently aligned with the gate
  check = False

  # Start control loop; exits when roscore shuts down or break is hit
  while not rospy.is_shutdown():

    # Wait until target_x and target_y are received (both default to -1 until first message)
    while t1.state.target_x == -1 and t1.state.target_y == -1:
      pass

    # Print received target values
    print(t1.state.target_x, t1.state.target_y)

    # Compute offset between gate center and image center on x-axis
    dx = t1.state.target_x - 480

    # Compute offset between gate center and image center on y-axis
    dy = t1.state.target_y - 200

    # If canPass == 1, mask area exceeded threshold; accelerate through the gate
    if t1.state.canPass == 1:

      # Move forward at 0.4 for 5.2 seconds
      msg = Twist()
      msg.linear.y = 0.4
      #msg.linear.z = 0.1
      t1.controler.move(msg,5.2)

      # Hover (send zero Twist) for 1 second
      msg = Twist()
      t1.controler.move(msg, 1)
      break # Exit control loop

    # If canPass == 0, mask area is below threshold; correct alignment and move forward as needed
    elif t1.state.canPass == 0:

      # When not yet aligned, use the narrow threshold to check for alignment
      if check == False:
        # |dx| < 24 means left/right aligned; |dy| < 24 means up/down aligned
        # Both aligned: set check = True and move forward
        if abs(dx) < 24 and abs(dy) < 24:
          check = True

          # Move forward at 0.3 for 1 second
          msg = Twist()
          msg.linear.y = 0.3
          t1.controler.move(msg, 1)
        # Not aligned: apply correction in x and/or y direction
        else:
          msg = Twist()

          # Correct left/right: linear.x negative = left, positive = right
          # target_x > 480 means gate is right of center (move right +); < 480 means move left (-)
          if dx != 0:
            msg.linear.x = dx / abs(dx) * 0.1

          # Correct up/down: linear.z negative = down, positive = up
          # target_y > 200 means gate is below center (move down = -z); < 200 means move up (+z)
          if dy != 0:
            msg.linear.z = -dy / abs(dy) * 0.2

          # Send correction command for 0.3 seconds
          t1.controler.move(msg, 0.3)

      # When already aligned, use the wide threshold to detect if alignment is lost
      else:

        # If |dx| >= 60 or |dy| >= 30, alignment is lost; re-correct
        if abs(dx) >= 60 or abs(dy) >= 30:
          check = False

          msg = Twist()

          # Correct left/right (same logic as above)
          if dx != 0:
            msg.linear.x = dx / abs(dx) * 0.1
          # Correct up/down (same logic as above)
          if dy != 0:
            msg.linear.z = -dy / abs(dy) * 0.1

          # Send correction command for 0.5 seconds
          t1.controler.move(msg, 0.5)

        # Still within wide threshold: continue moving forward
        else:

          # Move forward at 0.3 for 1 second
          msg = Twist()
          msg.linear.y = 0.3
          t1.controler.move(msg, 1)

      ### After reaching here, the loop restarts, recomputing dx/dy and acting on updated target/canPass values ###

# main function
# Runs the full sequence: takeoff, gate passing, land
def main():

  t1 = simple_tello.Tello_drone() # Create Tello_drone instance t1

  while t1.state.is_flying == False:  # Loop until takeoff is confirmed (is_flying becomes True)
    t1.controler.takeoff()

  while t1.state.fly_mode != 6: # Wait until special actions complete (fly_mode == 6 means idle)
    print("wait...")

  tello_pass(t1) # Run gate-passing control loop using t1

  while t1.state.fly_mode != 6: # Wait until any remaining special action completes
    print("wait...")

  while t1.state.is_flying == True:  # Loop until landing is confirmed (is_flying becomes False)
    t1.controler.land()

# main
if __name__ == "__main__":

  # Register this script as a ROS node named 'pass_example'; anonymous=True adds a random suffix
  rospy.init_node("pass_example", anonymous=True)

  # Run main function
  main()
