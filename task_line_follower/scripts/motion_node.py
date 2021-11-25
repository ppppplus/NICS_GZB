#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class MotionNode:
    def __init__(self):
        rospy.init_node('motion_node', anonymous=True)
        rospy.loginfo('motion node set up.')
        self.image_width=640
        self.image_height=480
        self.center_band=50 #车道线偏移在center_band内不用修正

        self.line_center=-1 #init
        self.last_bias=0

        self.rate = rospy.Rate(10) #20 Hz
        # 发布控制速度
        self.posSub_ = rospy.Subscriber('direction', Int32, self.posCallback)  
        #车道线的横坐标
        self.velocityPub_=rospy.Publisher("cmd_vel", Twist, queue_size=10)

        while not rospy.is_shutdown():
            self.GetVelocity()
            self.rate.sleep()

    def GetVelocity(self):
        if self.line_center<0 :
            return

        search_flag = False
        velocity=Twist()
        bias = self.image_width/2- self.line_center#距离中心偏移
        K_p = 0.0018#   0.0008     # PID参数
        print("bias: %s"%bias)
        if bias >=self.image_width/2 or bias <= -self.image_width/2 or \
           (bias > -self.center_band and  bias < self.center_band):
            search_flag = False
            if bias >=self.image_width/2 or bias <= -self.image_width/2:
                search_flag = True
                print("I can't see the line!")        
            bias = 0.0 #清零

        if search_flag == True:
            velocity.linear.x =0.15#0.15
            velocity.angular.z = 0.8 if self.last_bias>=0 else -0.8
        else:
            velocity.linear.x =0.10+ 0.20*(1.0- abs(bias)/(self.image_width/2) )
            velocity.angular.z = K_p*bias

        if velocity.angular.z > 0.86:
	        velocity.angular.z = 0.86
        if velocity.angular.z < -0.86:
	        velocity.angular.z = -0.86        

        velocity.linear.x*=4
        velocity.angular.z*=1
        self.velocityPub_.publish(velocity)
        if abs(bias)>=self.center_band:
            self.last_bias=bias

    def posCallback(self, msg):
        self.line_center=msg.data
        #self.GetVelocity()



if __name__ == '__main__':
    cn = MotionNode()