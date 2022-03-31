#!/usr/bin/python
#-*- encoding: utf8 -*-
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DetectNode:

    def __init__(self):
        rospy.init_node('detection_node', anonymous=True)
        rospy.logwarn('detection node set up.')
        self.image_ = None
        self.color_range_ = [(26, 70, 50), (34, 255, 255)]# 黄色的HSV范围
        self.bridge_ = CvBridge()
        self.rate=rospy.Rate(20)

        # 接收摄像头图像
        self.imageSub_ = rospy.Subscriber('/AKM_1/camera/rgb/image_raw', Image, self.imageCallback)  
        #发布车道线的横坐标
        self.posPub_=rospy.Publisher('/direction', Int32, queue_size=1)
        while not rospy.is_shutdown():
            self.GetCenter()
            self.rate.sleep()
     

    def imageCallback(self, msg):
        try:
            self.image_ = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            print(err)
        
    def GetCenter(self):
        if self.image_ is None:
            return
        #self.image_ = cv2.GaussianBlur(self.image_.copy(), (3, 3), 0)  # 高斯模糊
        # 将图片转换到HSV空间
        image_hsv = cv2.cvtColor(self.image_.copy(), cv2.COLOR_BGR2HSV) 
        image_hsv = image_hsv[320:,:,:]
        height = image_hsv.shape[0]
        width = image_hsv.shape[1]

        t_start=rospy.get_time()
        #黄色像素点
        #TODO:逐元素访问太慢 2.5~2.7s 似乎必须要加其他技巧
        '''
        data_yellow=[]
        for row in range(height):            
            for col in range(weight):         
                    pixel_h = image_hsv[row][col][0]
                    pixel_s = image_hsv[row][col][1] 
                    pixel_v = image_hsv[row][col][2] 
                    if pixel_h>=self.color_range_[0][0] and pixel_h<=self.color_range_[1][0] \
                        and pixel_s>=self.color_range_[0][1] and pixel_s<=self.color_range_[1][1] \
                        and pixel_v>=self.color_range_[0][2] and pixel_v<=self.color_range_[1][2]:
                        data_yellow.append([row,col])
        #中心位置
        center=0
        for pos in data_yellow:
            center+=pos[1]

        '''
        data_yellow=np.where( (image_hsv[:,:,0]>=self.color_range_[0][0] )& (image_hsv[:,:,0]<=self.color_range_[1][0]) \
                        & (image_hsv[:,:,1]>=self.color_range_[0][1]) & (image_hsv[:,:,1]<=self.color_range_[1][1]) \
                        & (image_hsv[:,:,2]>=self.color_range_[0][2]) & (image_hsv[:,:,2]<=self.color_range_[1][2]) )

        center=data_yellow[1].sum()

        

        if len(data_yellow[1])>0:
            center=center/len(data_yellow[1])
        #发布
        t_span=rospy.get_time()-t_start
        msg = Int32()
        msg.data = int(center)
        self.posPub_.publish(msg)


if __name__ == '__main__':
    cn = DetectNode()
