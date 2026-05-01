#!/usr/bin/env python
# -*- coding: utf-8 -*-

### Vision processing example ###
### Subscribes to Tello's image topic and builds a stream using StandaloneVideoStream ###
### In main(), iterates over stream frames to perform HSV filtering, find contours, and compute the contour center ###
### Uses minAreaRect to get the bounding rectangle of the largest contour; width * height = mask area ###
### Computes mask area / total frame area (960 * 720); if ratio >= threshold, canPass = 1, else 0 ###
### Publishes contour center and canPass flag via rospy.Publisher ###

import rospy                                 # Import package: rospy
from sensor_msgs.msg import CompressedImage  # Import CompressedImage from sensor_msgs
from std_msgs.msg import Float64MultiArray   # Import Float64MultiArray from std_msgs
import av                                    # Import package: av
import cv2                                   # Import package: cv2
import numpy as np                           # Import numpy as np
import threading                             # Import package: threading
import traceback                             # Import package: traceback
import time                                  # Import package: time

### class StandaloneVideoStream
### Wraps the CompressedImage data received from /tello/image_raw/h264 into a readable stream
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

# callback: subscribes to /tello/image_raw/h264; processes incoming CompressedImage (msg = CompressedImage)
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
    # out: VideoWriter for test.avi at 20 FPS; width = 960*2 = 1920 (original + processed side by side), height = 720
    out = cv2.VideoWriter('test.avi', fourcc, 20.0, (1920, 720))

    # Register as ROS node named 'h264_listener'
    rospy.init_node('h264_listener')

    # Subscribe to /tello/image_raw/h264 (CompressedImage); callback processes each received frame
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)

    # Create publisher point_pub, publishing Float64MultiArray to /target_point
    point_pub = rospy.Publisher("/target_point", Float64MultiArray, queue_size = 10)

    # Open the stream with av
    container = av.open(stream)

    rospy.loginfo('main: opened')

    # start_detect: when True, publish target data to /target_point
    start_detect = True

    # Discard the first 300 frames to flush stale buffered frames from stream startup
    frame_skip = 300

    # Decode frames from container; loop exits when no more frames are available
    for frame in container.decode(video=0):

        # Skip frames until frame_skip reaches 0
        if 0 < frame_skip:
          frame_skip -= 1
          continue

        # Record start time to measure processing duration for FPS-based frame skipping
        start_time = time.time()

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

        # Assume the largest contour is the gate; skip frame if no contours found
        if len(c_c) == 0:
          continue
        max_c = max(c_c, key = cv2.contourArea)

        # Get the minimum bounding rectangle of the largest contour
        # rect[0]: center (x, y); rect[1]: (width, height)
        rect = cv2.minAreaRect(max_c)
        r_w, r_h = rect[1]
        rec_x = rect[0][0]
        rec_y = rect[0][1]

        # Build a blank BGR image for visualization
        show_image = cv2.cvtColor(np.zeros(image.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)

        # Draw the bounding rectangle on show_image
        cv2.rectangle(show_image, (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)), (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)), (0,0,255), -1)

        # Print center coordinates
        print("minAreaRect x: ", rec_x)
        print("minAreaRect y: ", rec_y)

        # Draw a circle at the bounding rect center
        cv2.circle(show_image, (int(rec_x), int(rec_y)), 10, (0,50,175), -1)

        # Display the area ratio (mask area / total frame area) as text
        cv2.putText(show_image, str((r_w * r_h) / (960*720.)), (10,40), 5, 2, (255,255,0))

        # Publish target data when start_detect is True
        if start_detect:
          # If mask area >= 35% of total frame, the drone is close enough to the gate
          if (r_w * r_h) / (960*720.0) >= 0.35:
            # Publish center and canPass=1 to indicate threshold reached
            point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 1]))
            # Stop publishing after triggering the pass
            start_detect = False
          else:
            # Publish center and canPass=0; continue alignment and forward movement
            point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0]))

        # Concatenate original and processed images side by side, write to video file
        out.write(np.concatenate((image, show_image), axis = 1))

        # Display the combined image in a window
        cv2.imshow('result', np.concatenate((image, show_image), axis = 1))

        # Refresh display
        cv2.waitKey(1)

        # Compute how many frames to skip based on actual processing time vs. stream FPS
        if frame.time_base < 1.0/60:
          time_base = 1.0/60
        else:
          time_base = frame.time_base
        frame_skip = int((time.time() - start_time)/time_base)

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
