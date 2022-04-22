from asyncio import FastChildWatcher
from asyncore import file_dispatcher
from email import message
import imp
from socket import MsgFlag
from timeit import repeat

from cv2 import StereoSGBM_create
import roslib
import rospy
import math
import message_filters
import tf
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

from visualization_msgs.msg import Marker
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
rospy.init_node('part2', anonymous=True)

def callback(imgLeft, imgRight, camera_info):
    depth_pub = rospy.Publisher('/zed/zed_node/depth/depth_registered', Image,queue_size=10)
    imgL = imgLeft
    imgR = imgRight
    bridge = CvBridge()
    imgLcv = bridge.imgmsg_to_cv2(imgL, desired_encoding='passthrough')
    imgRcv = bridge.imgmsg_to_cv2(imgR, desired_encoding='passthrough')
    grayl = cv.cvtColor(imgLcv, cv.COLOR_RGB2GRAY)
    grayr = cv.cvtColor(imgRcv, cv.COLOR_RGB2GRAY)
    print(grayl.dtype)
    max_disp = 160
    lamda = 8000
    sigma = 1.5
    vis_mult = 1
    bsize = 3
    p1 = 24*(bsize**2)
    p2 = 96*(bsize**2)
    pre_filter = 63
    mod = 2

    left_for_matcher = grayl
    right_for_matcher = grayr

    left_matcher = StereoSGBM_create(minDisparity = 0, 
    numDisparities = max_disp, 
    blockSize = bsize,
    P1 = p1,
    P2 = p2,
    disp12MaxDiff = 0,
    uniquenessRatio = 15,
    speckleWindowSize = 0,
    speckleRange = 0,
    preFilterCap = pre_filter,
    mode = mod)

    # stereo = cv.StereoBM_create(numDisparities = 16, blockSize = 15)
    # disparity = stereo.compute(grayl, grayr, disparity)
    # plt.imshow('gray', disparity)

    right_matcher = cv.ximgproc.createRightMatcher(left_matcher)
    left_disp = grayl
    right_disp = grayr
    left_disp = left_matcher.compute(grayl, grayr, left_disp)
    right_disp = right_matcher.compute(grayr, grayl, right_disp)
    # cv.imshow("a", left_disp)
    wlsfilter = cv.ximgproc.createDisparityWLSFilter(left_matcher)
    wlsfilter.setLambda(lamda)
    wlsfilter.setSigmaColor(sigma)
    filter_disp = left_disp

    filter_disp = wlsfilter.filter(left_disp, grayl, filter_disp, right_disp)
    # cv.imshow("b", filter_disp)
    filter_disp_vis = grayl

    filter_disp_vis = cv.ximgproc.getDisparityVis(filter_disp, filter_disp_vis, 1.0)
    print(filter_disp_vis.dtype)
    # cv.imshow('gray', filter_disp_vis)
    pub_message = bridge.cv2_to_imgmsg(filter_disp_vis, encoding='mono8')
    
    depth_pub.publish(pub_message)
    rate = rospy.Rate(10)
    rate.sleep()


def listener():
    print("try subscribe")
    left_img_sub = message_filters.Subscriber("/zed/zed_node/left/image_rect_color", Image)
    right_img_sub = message_filters.Subscriber("/zed/zed_node/right/image_rect_color", Image)
    right_camera_info_sub = message_filters.Subscriber("/zed/zed_node/right/camera_info", CameraInfo)
    ts = message_filters.TimeSynchronizer([left_img_sub, right_img_sub, right_camera_info_sub], 10)
    ts.registerCallback(callback)
    print("subscribe success")
    rate = rospy.Rate(10)
    rate.sleep

    

if __name__ == '__main__':
    try:
        
        listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass