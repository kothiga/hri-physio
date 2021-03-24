#!/usr/bin/env python

# ================================================================================
# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
#     University of Waterloo, All rights reserved.
# 
# Authors: 
#     Austin Kothig <austin.kothig@uwaterloo.ca>
#     Alperen Akgun <sami.alperen.akgun@uwaterloo.ca>
# 
# CopyPolicy: Released under the terms of the BSD 3-Clause License. 
#     See the accompanying LICENSE file for details.
# ================================================================================

import rospy
from std_msgs.msg import String
import cv2


# Global variable.
video_name = ""


def callback(data):
    global video_name
    video_name = data.data
    return
   
    
def listener():

    # Set up the node.
    rospy.init_node('video_streamer', anonymous=True)
    rospy.Subscriber('video_streamer/video_name', String, callback)
    rate = rospy.Rate(100)

    
    # Get a reference to the global var.
    global video_name
    prev_name = ""
    
	# Open the video stream.
    cap = cv2.VideoCapture(video_name)
    
    # Loop while things are okay.
    while not rospy.is_shutdown():

	    # Check if the video_name changed.
        if video_name != prev_name:

            print("Got a new video name ``{}``".format(video_name))

            # Keep track of the last video_name.
            prev_name = video_name

            # Check for break out name.
            if video_name.lower() == "exit" or video_name.lower() == "quit":
                if cap.isOpened():
                    cv2.destroyWindow("videoStreamer")
                rospy.signal_shutdown("Shutting down Video Streamer...")
                exit(0)

            # Open the new video.
            cap.open(video_name)

            # Check if the video opened successfully.
            if not cap.isOpened():
                print("Video name ``{}`` could not be found!!".format(video_name))
                continue
            
            # Get fps.
            fps = cap.get(cv2.CAP_PROP_FPS)
            print("Loaded video. Fps: {} {}".format(fps, 1000 // fps))


        else:
            # Sleep for a short time.
            rate.sleep()
        
        # Loop while the capture is available.
        while cap.isOpened():

            # If the video_name changed break out.
            if video_name != prev_name:
	            break

            # Read the next frame.
            ret, frame = cap.read()

            # Frame ok?
            if ret:
                # Adjust the frame.
                frame = cv2.resize(frame,
                    #(1280,720),
                    (1920,1080),
                    fx=0,fy=0, 
                    interpolation=cv2.INTER_CUBIC
                )
                # Show it.
                cv2.imshow('videoStreamer',frame)
            
            # At the end of the video, loop back to beginning.
            else:
                cap.set(cv2.CAP_PROP_POS_FRAMES,0)

            # Check if capture close has occurred.
            key = cv2.waitKey(int(1000 / fps))
            if key == 27 or key == 1048603:
                cv2.destroyWindow("videoStreamer")
                rospy.signal_shutdown("Shutting down Video Streamer...")
                exit(0)
    return

if __name__ == '__main__':
    listener()
