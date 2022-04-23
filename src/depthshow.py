# Here are all the reference I read during the process of the coding. All of them are in the website of opencv or ROS or Python documentation.
# It also include the source code of some function of cv I found in github. It is also manage by ros or opencv organazation.
# 
# reference: 1.https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga1bc1152bd57d63bc524204f21fde6e02
#2.https://docs.opencv.org/3.4/dc/d84/group__core__basic.html#ga599fe92e910c027be274233eccad7beb
#3.https://answers.ros.org/question/255351/how-o-save-a-pointcloud2-data-in-python/
#4.https://docs.opencv.org/3.4/dc/d84/group__core__basic.html#ga599fe92e910c027be274233eccad7beb
#5.http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
#6.https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py
#7.http://wiki.ros.org/depth_image_proc
#8.https://docs.python.org/3/library/struct.html#format-characters
#9.https://answers.ros.org/question/234455/pointcloud2-and-pointfield/     
#10.https://docs.opencv.org/3.4/d9/d51/classcv_1_1ximgproc_1_1DisparityWLSFilter.html
#11.https://docs.opencv.org/3.1.0/d3/d14/tutorial_ximgproc_disparity_filtering.html
#12.https://github.com/opencv/opencv_contrib/blob/master/modules/ximgproc/samples/disparity_filtering.cpp

from asyncio import InvalidStateError
from asyncore import file_dispatcher
from multiprocessing.connection import wait
from typing import List

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
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField, PointCloud2
from visualization_msgs.msg import Marker
import cv_bridge
from cv_bridge import CvBridge
from cv2 import StereoSGBM
import imp
import pcl
import struct

rospy.init_node('part3', anonymous=True)

bridge = CvBridge()
imgL = Image()
imgR = Image()
frame_counter = 0
frame_require = 0
    
def read_right(left_data,data, camera_info):
    global frame_counter
    global frame_require
    frame_require = rospy.get_param("frame_num")
    frame_counter += 1
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
    laambda = 8000.0
    left_matcher = cv.StereoSGBM_create(0, max_disp, win_size)
    right_matcher = cv.ximgproc.createRightMatcher(left_matcher)
    left_disp = left_matcher.compute(grayl, grayr)
    right_disp = right_matcher.compute(grayr, grayl)
    

    filtered_disp_bis = grayl
    wls_filter = cv.ximgproc.createDisparityWLSFilter(left_matcher)
    wls_filter.setLambda(laambda)
    wls_filter.setSigmaColor(sigma)
    filtered_disp = wls_filter.filter(left_disp, grayl, disparity_map_right = right_disp)
    filtered_disp_vis = cv.ximgproc.getDisparityVis(filtered_disp, filtered_disp_bis, 1.0)
    # print('berfore show')
    # cv.imshow('test', filtered_disp_vis)
    # cv.waitKey(1)
    # print("after show")
    
    # right_disp_divided = np.float32(np.divide(right_disp, 16.0))
    puber = rospy.Publisher('/zed/depth/depthsubmission', Image, queue_size = 10)
    pic = bridge.cv2_to_imgmsg(filtered_disp_vis, encoding = "passthrough")
    puber.publish(pic)

    print(frame_require)
    print(frame_counter)
    if(frame_counter == frame_require):

        cloud_puber = rospy.Publisher('/zed/cloud', PointCloud2, queue_size = 10)
        pic = bridge.cv2_to_imgmsg(filtered_disp_vis, encoding = "passthrough")
        
        cx = camera_info.K[2]
        cy = camera_info.K[5]
        base_line = 120
        cx_diff = camera_info.P[2]
        f = camera_info.K[4]
        q = np.float32([[1, 0, 0,-cx],
                        [0,1, 0,-cy], # turn points 180 deg around x-axis,
                        [0, 0, 0,  f], # so that y-axis looks up
                        [0, 0, (-1)/base_line, (cx - cx_diff)/base_line]])
    
        _3dimage = grayl
        _3dimage = cv.reprojectImageTo3D(filtered_disp_vis, q, _3dimage, False)
        # print(_3dimage.dtype)
        # cv.imshow('tes', _3dimage)
        # waitKey(1000)
        lis = list()
        for i in range (0, len(filtered_disp_vis) - 1):
            for j in range (0, len(filtered_disp_vis[0]) - 1):
                dis = filtered_disp_vis[i][j]
                if(dis != 0):
                    
                    # p = pcl.PointCloud_PointXYZRGB()
                    rgb = np.int16(imgR[i][j])
                    blue = rgb[2]
                    green = rgb[1]
                    red = rgb[0]
                    rgb_struct = struct.unpack('I', struct.pack('BBBB', red, green, blue, 1))[0] #B means the red/green/blue is 1 byte
                    poin = _3dimage[i][j]  # get the xyz
                    lis.append([poin[0]/600.00, poin[1]/600.00, poin[2]/600.00, rgb_struct])
                    
        # point_cloud.from_list(lis)
        head = Header()
        head.frame_id = 'base_link'
        pf = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgba', 12, PointField.UINT32, 1)]
        msgs_point_cloud = point_cloud2.create_cloud(head, pf, lis)
        msgs_point_cloud.header.stamp = rospy.Time.now()

        cloud_puber.publish(msgs_point_cloud)

    
    

    rate = rospy.Rate(10)
    rate.sleep()

    # cv.imshow('h', _3dimage)
    # waitKey(1)
    


    
    
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
    