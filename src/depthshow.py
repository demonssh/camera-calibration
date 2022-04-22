
from asyncio import InvalidStateError
from asyncore import file_dispatcher
from multiprocessing.connection import wait

from cv2 import CV_64F
import message_filters
import numpy as np
import cv2 as cv
from cv2 import Mat
from cv2 import waitKey
from matplotlib import pyplot as plt
from socket import INADDR_MAX_LOCAL_GROUP, MsgFlag
from timeit import repeat
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
import turtlesim.srv
import random
from visualization_msgs.msg import Marker
import cv_bridge
from cv_bridge import CvBridge
from cv2 import StereoSGBM
import imp


rospy.init_node('part3', anonymous=True)

bridge = CvBridge()
imgL = Image()
imgR = Image()

# def read_left(data):
#     global imgL 
#     imgL = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    
def read_right(left_data,data, camera_info):
    global imgL
    global imgR 
    print("get data")
    imgL = bridge.imgmsg_to_cv2(left_data, desired_encoding='passthrough')
    imgR = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    
    grayl = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayr = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)
    
    win_size = 3
    min_disp = 0
    max_disp = 160
    num_disp = max_disp - min_disp
    a = cv.StereoSGBM_create()
    stereo = cv.StereoSGBM_create(minDisparity = min_disp,
        preFilterCap = 63,
        numDisparities = num_disp,
        blockSize = win_size,
        P1 = 24*win_size**2,
        P2 = 96*win_size**2,
        mode = 2)
    
    sigma = 1.5
    lmbda = 8000.0
    left_matcher = cv.StereoSGBM_create(0, max_disp, win_size)
    right_matcher = cv.ximgproc.createRightMatcher(left_matcher)
    left_disp = left_matcher.compute(grayl, grayr)
    right_disp = right_matcher.compute(grayr, grayl)
    

    filtered_disp_bis = grayl
    wls_filter = cv.ximgproc.createDisparityWLSFilter(left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    filtered_disp = wls_filter.filter(left_disp, grayl, disparity_map_right = right_disp)
    filtered_disp_vis = cv.ximgproc.getDisparityVis(filtered_disp, filtered_disp_bis, 1.0)
    # print('berfore show')
    # cv.imshow('test', filtered_disp_vis)
    # cv.waitKey(1)
    # print("after show")
    right_disp_divided = np.float32(np.divide(right_disp, 16.0))
    puber = rospy.Publisher('/zed/depth/depthsubmission', Image, queue_size = 10)
    pic = bridge.cv2_to_imgmsg(filtered_disp_vis, encoding = "passthrough")
    puber.publish(pic)
    cx = camera_info.K(2)
    cy = camera_info.K(5)
    base_line = 140
    cx_diff = camera_info.P(2)
    f = camera_info.K(4)
    q = Mat((4,4), CV_64F)
    q[0, 0] = 1
    q[1, 1] = 1
    q[0, 3] = -cx
    q[1, 3] = -cy
    q[2, 3] = f
    q[3, 2] = (-1)/base_line
    q[3, 3] = (cx - cx_diff)/base_line
    _3dimage = Mat()

    _3dimage = cv.reprojectImageTo3D(filtered_disp_vis, q, _3dimage, True)
    cloud = pcl.PointCloud_PointXYZRGB()
    for i in range (0, len(filtered_disp_vis) - 1):
        for j in range (0, len(filtered_disp_vis[0])):

    rate = rospy.Rate(10)
    rate.sleep()
    
def listener():
    print("try sub")
    image_left_filter = message_filters.Subscriber("/zed/zed_node/left/image_rect_color", Image)
    image_filter = message_filters.Subscriber("/zed/zed_node/right/image_rect_color", Image)
    info_filter = message_filters.Subscriber("/zed/zed_node/right/camera_info", CameraInfo)
    ts = message_filters.TimeSynchronizer([image_left_filter, image_filter, info_filter], 10)
    ts.registerCallback(read_right)
        
        
if __name__ == '__main__':
    try:
        listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    