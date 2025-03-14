import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class CameraProcessing:
    def __init__(self, resolution=0.5, fov=90, detection_range=15):
        self.resolution = resolution
        self.fov = fov
        self.detection_range = detection_range
        
        self.cvBridge = CvBridge()
        self.debug = True
        camera_topics = [
            "/gem/front_camera/image_raw",
            "/gem/front_single_camera/front_single_camera/image_raw"
        ]
        for topic in camera_topics:
            rospy.loginfo(f"Attempting to subscribe to {topic}")
            self.cameraSub = rospy.Subscriber(topic, Image, 
                                            self.__cameraHandler, queue_size=1)
        
        self.edgeAnglePub = rospy.Publisher("/gem/edge_angles", Float32MultiArray, 
                                           queue_size=1)
        self.debugPub = rospy.Publisher("/gem/edge_detection_debug", Image, 
                                       queue_size=1)
        
        # Parameters for edge detection
        self.canny_low = 20   
        self.canny_high = 80
        self.hough_threshold = 20  
        self.min_line_length = 50  
        self.max_line_gap = 20 
        self.vertical_threshold = np.pi/4 
        rospy.loginfo("CameraProcessing initialized")
        
    def __cameraHandler(self, data):
        try:
            rospy.loginfo_throttle(1, f"Receiving images from {data._connection_header['topic']}")
            
            cv_image = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
            
            if self.debug:
                rospy.loginfo_throttle(1, f"Image shape: {cv_image.shape}")
            
            edge_angles = self.detectEdges(cv_image)
            msg = Float32MultiArray()
            msg.data = edge_angles
            self.edgeAnglePub.publish(msg)
            
            if edge_angles:
                rospy.loginfo_throttle(1, f"Detected {len(edge_angles)} edges at angles: {edge_angles}")
            
        except Exception as e:
            rospy.logerr(f"Error in camera handler: {str(e)}")




    def detectEdges(self, image):
        """Detect vertical edges in the image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        debug_img = image.copy()
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 
                               threshold=self.hough_threshold,
                               minLineLength=self.min_line_length,
                               maxLineGap=self.max_line_gap)
        angles = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                angle = np.arctan2(y2 - y1, x2 - x1)
                if abs(abs(angle) - np.pi/2) < self.vertical_threshold:
                    center_x = image.shape[1] / 2
                    if x1 < center_x and x2 < center_x:
                        angle_from_center = -90
                    elif x1 > center_x and x2 > center_x:
                        angle_from_center = 90
                    else:
                        angle_from_center = np.degrees(np.arctan2(x2 - center_x, self.detection_range))
                    
                    angles.append(angle_from_center)
                    cv2.line(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        height, width = image.shape[:2]
        cv2.line(debug_img, (width//2, 0), (width//2, height), (0, 255, 0), 1)
        cv2.putText(debug_img, f"Edges: {len(angles)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        try:
            debug_msg = self.cvBridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.debugPub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing debug image: {str(e)}")
        
        return angles

    def getEdgeAngles(self):
        return self.edge_angles