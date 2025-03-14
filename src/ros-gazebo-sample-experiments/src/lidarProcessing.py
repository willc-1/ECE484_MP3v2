import math
import numpy as np

import rospy
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2



class LidarProcessing:
    def __init__(self, resolution=0.5, side_range=(-5., 5.), fwd_range=(0., 15.),
                         height_range=(-1, 1)):
        self.resolution = resolution
        self.side_range = side_range
        self.fwd_range = fwd_range
        self.height_range = height_range

        self.cvBridge = CvBridge()

        # random size initial image
        self.__birds_eye_view =  np.zeros((200, 100))

        self.birdsEyeViewPub = rospy.Publisher("/mp0/BirdsEye", Image, queue_size=1)
        self.pointCloudSub = rospy.Subscriber("/velodyne_points", PointCloud2, self.__pointCloudHandler, queue_size=10)


    def getBirdsEyeView(self):
        return self.__birds_eye_view


    def __pointCloudHandler(self, data):
        """
            Callback function for whenever the lidar point clouds are detected

            Input: data - lidar point cloud

            Output: None

            Side Effects: updates the birds eye view image
        """
        gen = point_cloud2.readgen = point_cloud2.read_points(cloud=data, field_names=('x', 'y', 'z', 'ring'))

        lidarPtBV = []
        for p in gen:
            lidarPtBV.append((p[0],p[1],p[2]))

        self.__birds_eye_view = self.construct_birds_eye_view(lidarPtBV)

    def construct_birds_eye_view(self, data):
        """
            Helper function that constructs the birds eye view image from the lidar point clouds

            Input: data - lidar point cloud

            Output: the birds eye view image corresponindg to the point cloud in the area specified
        """
        # create image from_array
        x_max = 1 + int((self.side_range[1] - self.side_range[0]) / self.resolution)
        y_max = 1 + int((self.fwd_range[1] - self.fwd_range[0]) / self.resolution)
        im = np.zeros([y_max, x_max], dtype=np.uint8)

        if len(data) == 0:
            return im

        # Reference: http://ronny.rest/tutorials/module/pointclouds_01/point_cloud_birdseye/
        data = np.array(data)

        x_points = data[:, 0]
        y_points = data[:, 1]
        z_points = data[:, 2]

        # Only keep points in the range specified above
        fwd_filt = np.logical_and((x_points >= self.fwd_range[0]), (x_points <= self.fwd_range[1]))
        side_filt = np.logical_and((y_points >= -self.side_range[1]), (y_points <= -self.side_range[0]))
        filter = np.logical_and(fwd_filt, side_filt)
        indices = np.argwhere(filter).flatten()

        x_points = x_points[indices]
        y_points = y_points[indices]
        z_points = z_points[indices]

        # convert points to image coords with resolution
        x_img = np.floor(-y_points / self.resolution).astype(np.int32)
        y_img = np.floor(-x_points / self.resolution).astype(np.int32)

        # shift coords to new original
        x_img -= int(np.floor(self.side_range[0] / self.resolution))
        y_img += int(np.ceil(self.fwd_range[1] / self.resolution))

        # clip based on height for pixel Values
        pixel_vals = np.clip(a=z_points, a_min=self.height_range[0], a_max=self.height_range[1])

        def scale_to_255(a, min_val, max_val, dtype=np.uint8):
            return (((a-min_val) / float(max_val - min_val) ) * 255).astype(dtype)

        pixel_vals = scale_to_255(pixel_vals, min_val=self.height_range[0], max_val=self.height_range[1])

        im[y_img, x_img] = pixel_vals

        return im

    def processLidar(self):
        """
            Publishes birds eye view image

            Inputs: None

            Outputs: None
        """
        
        birds_eye_im = self.__birds_eye_view.astype(np.uint8)
        birds_eye_im = self.cvBridge.cv2_to_imgmsg(birds_eye_im, 'mono8')
        self.birdsEyeViewPub.publish(birds_eye_im)
