#!/usr/bin/env python2
# -*- coding: utf-8 -*-

### Vision processing example: applies HSV filtering to the Tello's camera feed, ###
### finds a specific color mask, and displays the result ###

import rospy                                 # Import package: rospy
from sensor_msgs.msg import CompressedImage  # Import CompressedImage from sensor_msgs
import av                                    # Import package: av
import cv2                                   # Import package: cv2
import numpy as np                           # Import numpy as np
import threading                             # Import package: threading
import traceback                             # Import package: traceback

### class StandaloneVideoStream
### Wraps CompressedImage data from /tello/image_raw/h264 into a readable stream
### Provides a stream object that can be decoded frame by frame
class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()

# Create stream instance
stream = StandaloneVideoStream()

# callback: subscribes to /tello/image_raw/h264; msg is the received CompressedImage
def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data)) # Uncomment to monitor incoming byte count
    # Add received CompressedImage data into the stream
    stream.add_frame(msg.data)

# find_Mask: takes an HSV image and returns a binary mask for the red color range
def find_Mask(img):

  # Lower bound for left red range in HSV (see HSV range diagram in slides)
  lr0 = np.array([0, 70, 0])
  # Upper bound for left red range in HSV
  ur0 = np.array([5, 255, 255])

  # Lower bound for right red range in HSV (hue wraps around 180)
  lr1 = np.array([175, 70, 0])
  # Upper bound for right red range in HSV
  ur1 = np.array([180, 255, 255])

  # Filter with left red range
  rm0 = cv2.inRange(img, lr0, ur0)

  # Filter with right red range
  rm1 = cv2.inRange(img, lr1, ur1)

  # Combine both masks with bitwise OR
  rm = cv2.bitwise_or(rm0, rm1)
  return rm

# main function
def main():

    # fourcc: video encoding format (e.g., XVID, MP4V)
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    # out: VideoWriter for test.avi at 20 FPS; width = 1920 (960 original + 960 processed), height = 720
    out = cv2.VideoWriter('test.avi', fourcc, 20.0, (1920, 720))

    # Register as ROS node named 'h264_listener'
    rospy.init_node('h264_listener')

    # Subscribe to /tello/image_raw/h264 (CompressedImage); callback handles each received frame
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)

    # Open the stream with av
    container = av.open(stream)

    rospy.loginfo('main: opened')

    # Decode frames from container; loop exits when no more frames are available
    for frame in container.decode(video=0):

        # Convert frame to numpy BGR image
        image = cv2.cvtColor(np.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)

        # Convert BGR to HSV
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Apply red mask filter
        red_mask = find_Mask(hsv_img)

        # Find contours in the red mask
        # RETR_EXTERNAL: only outermost contours
        # CHAIN_APPROX_NONE: store all contour points
        # Note: some OpenCV versions return (image, contours, hierarchy) — change to: _, c_c, _ = ... if needed
        _, c_c, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Build a blank BGR image for visualization
        show_image = cv2.cvtColor(np.zeros(image.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)

        # Draw all found contours on show_image
        # -1: draw all contours; (0,0,255): BGR color red; -1 thickness: filled
        cv2.drawContours(show_image, c_c, -1, (0, 0, 255), -1)

        # Concatenate original and processed images side by side, write to video file
        out.write(np.concatenate((image, show_image), axis = 1))

        # Display the combined image in a window
        cv2.imshow('result', np.concatenate((image, show_image), axis = 1))

        # Refresh display
        cv2.waitKey(1)

# main
if __name__ == '__main__':

    try:
        # Run main function to start vision processing loop
        main()

    except BaseException:
        traceback.print_exc()

    finally:
        stream.close()
        cv2.destroyAllWindows()
