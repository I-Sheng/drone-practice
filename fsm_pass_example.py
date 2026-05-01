#!/usr/bin/env python
# -*- coding: utf-8 -*-

### FSM (Finite State Machine) example program ###
### Implements gate passing from pass_example.py using a state machine ###
### Divides the gate-passing process into 4 states ###
### Defines the transition chain between states ###
### Use together with test_h264_sub.py and simple_tello.py from code-pass/ ###
### Recommended to run under Python 3 ###

import rospy                                  # Import package: rospy
import simple_tello                           # Import simple_tello

from geometry_msgs.msg import Twist           # Import Twist message type from geometry_msgs
from std_msgs.msg import Empty                # Import Empty message type from std_msgs
from statemachine import StateMachine, State  # Import StateMachine and State from statemachine
from time import sleep                        # Import sleep from time

# Create Tello_drone instance t1
t1 = simple_tello.Tello_drone()

### class sMachine
### Gate-passing finite state machine
### States: hover (hold position), correction (align), forward (fly toward gate), addSp (speed boost)
### Initial state: hover
### Transition chain:
###     wait4data: hover > hover
###       stop2do: addSp > hover
### start2correct: hover > correction
###  need2correct: correction > correction, forward > correction
### start2forawrd: hover > forward
###  need2forawrd: correction > forward, forward > forward
###    need2addSp: forward > addSp, correction > addSp
### on_enter callbacks trigger control actions when entering each state

class sMachine(StateMachine):

    # state define
    hover = State('Hover', initial = True)
    correction = State('Correction')
    forward = State('Forward')
    addSp = State('AddSp')

    # trans define
    wait4data = hover.to(hover)
    stop2do = addSp.to(hover)

    start2correct = hover.to(correction)
    need2correct = correction.to(correction) | forward.to(correction)

    start2forawrd = hover.to(forward)
    need2forawrd = correction.to(forward) | forward.to(forward)

    need2addSp = forward.to(addSp) | correction.to(addSp)

    # Called on entering hover state
    def on_enter_hover(self):

      # Send zero Twist to hold position for 0.5 seconds
      msg = Twist()
      t1.controler.move(msg, 0.5)

    # Called on entering correction state
    def on_enter_correction(self):

      msg = Twist()

      # Offset between gate center x and image center x (480)
      dx = t1.state.target_x - 480

      # Offset between gate center y and image center y (200)
      dy = t1.state.target_y - 200

      # Correct left/right: linear.x negative = left, positive = right
      # target_x > 480 means gate is right of center; target_x < 480 means left
      if dx != 0:
        msg.linear.x = dx / abs(dx) * 0.1

      # Correct up/down: linear.z negative = down, positive = up
      # target_y > 200 means gate is below center (move down = -z); target_y < 200 means above (move up = +z)
      if dy != 0:
        msg.linear.z = -dy / abs(dy) * 0.2

      # Send correction command for 0.5 seconds
      t1.controler.move(msg, 0.5)

    # Called on entering forward state
    def on_enter_forward(self):

      # Move forward at 0.2 for 0.5 seconds
      msg = Twist()
      msg.linear.y = 0.2
      t1.controler.move(msg, 0.5)

    # Called on entering addSp state
    def on_enter_addSp(self):

      # Accelerate forward at 0.4 for 3 seconds
      msg = Twist()
      msg.linear.y = 0.4
      t1.controler.move(msg, 3)

      # Continue forward at 0.5 for 3 seconds
      msg = Twist()
      msg.linear.y = 0.5
      t1.controler.move(msg, 3)

# class MyModel
# Transition logic model
class MyModel(object):

    # Initialize with the current FSM state label
    def __init__(self, state):
        self.state = state

    # run: evaluates transitions and drives the FSM
    def run(self, fsm):

        # Control loop; exits when roscore shuts down or break is hit
        while not rospy.is_shutdown():

            # Print current state
            print(self.state)

            # In hover state
            if fsm.hover.is_active:

                # No target received yet (-1); stay in hover via wait4data
                if t1.state.target_x == -1 and t1.state.target_y == -1:
                    fsm.wait4data()
                # Target received; decide whether to correct or move forward
                else:
                  dx = t1.state.target_x - 480
                  dy = t1.state.target_y - 200

                  # Within alignment threshold: transition to forward via start2forawrd
                  if abs(dx) < 30 and abs(dy) < 30:
                    fsm.start2forawrd()

                  # Outside threshold: transition to correction via start2correct
                  else:
                    fsm.start2correct()
            # In correction state
            elif fsm.correction.is_active:

                # canPass == 1: mask area exceeded threshold; transition to addSp via need2addSp
                if t1.state.canPass == 1:
                    fsm.need2addSp()
                # canPass != 1: still needs forward movement or correction
                else:
                    dx = t1.state.target_x - 480
                    dy = t1.state.target_y - 200

                    # Within threshold: transition to forward via need2forawrd
                    if abs(dx) < 30 and abs(dy) < 30:
                      fsm.need2forawrd()

                    # Outside threshold: stay in correction via need2correct
                    else:
                      fsm.need2correct()

            # In forward state
            elif fsm.forward.is_active:

                # canPass == 1: transition to addSp via need2addSp
                if t1.state.canPass == 1:
                    fsm.need2addSp()
                # canPass != 1: still needs forward movement or correction
                else:
                    dx = t1.state.target_x - 480
                    dy = t1.state.target_y - 200

                    # Within threshold: stay in forward via need2forawrd
                    if abs(dx) < 30 and abs(dy) < 30:
                      fsm.need2forawrd()

                    # Outside threshold: transition to correction via need2correct
                    else:
                      fsm.need2correct()
            # In addSp state: gate pass complete; exit loop
            elif fsm.addSp.is_active:
                break

# main function
# Runs the full sequence: takeoff, FSM gate passing, land
def main():

  while t1.state.is_flying == False:  # Loop until takeoff is confirmed (is_flying becomes True)
    t1.controler.takeoff()

  while t1.state.fly_mode != 6: # Wait until special actions complete (fly_mode == 6 means idle)
    print("wait...")

  # Initialize the transition model and FSM
  obj = MyModel(state='hover')
  fsm = sMachine(obj)

  # Start the FSM control loop
  obj.run(fsm)

  while t1.state.fly_mode != 6: # Wait until any remaining special action completes
    print("wait...")

  while t1.state.is_flying == True:  # Loop until landing is confirmed (is_flying becomes False)
    t1.controler.land()

# main
if __name__ == '__main__':

    # Register as ROS node named 'h264_pub'; anonymous=True adds a random suffix
    rospy.init_node('h264_pub', anonymous=True)

    # Run main function
    main()
