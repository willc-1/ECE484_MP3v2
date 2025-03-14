#!/usr/bin/env python3
import rospy
from camera_processing import CameraProcessing
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge

def angles_callback(data):
    if data.data:  
        rospy.loginfo_throttle(1, f"Detected edges at angles: {data.data}")

def main():
    rospy.init_node('camera_test')
    rospy.loginfo("Starting camera test node...")
    
    processor = CameraProcessing()
    
    rospy.Subscriber("/gem/edge_angles", Float32MultiArray, angles_callback)
    
    rospy.loginfo("Test node running...")
    rospy.loginfo("View edge detection results:")
    rospy.loginfo("1. rqt_image_view /gem/edge_detection_debug")
    rospy.loginfo("2. rostopic echo /gem/edge_angles")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass