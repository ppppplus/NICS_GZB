#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32,Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import os
import time
import threading


class MotionNode:
    def __init__(self):
        rospy.init_node('motion_node', anonymous=True)
        rospy.logwarn('motion node set up.')

        self.image_width=640
        #self.image_height=480
        self.center_band=50 #车道线偏移在center_band内不用修正

        self.line_center=-1 #init
        self.lidar_data = None
        self.last_bias=0

        self.rate = rospy.Rate(10) #20 Hz
        #车道线的横坐标       
        self.posSub_ = rospy.Subscriber('/direction', Int32, self.posCallback) 
        #subscribe lidar date 
        self.lidarSub_ = rospy.Subscriber('/lidar_list', Float32MultiArray, self.lidarCallback) 
        # 发布控制速度
        self.velocityPub_=rospy.Publisher('/AKM_1/cmd_vel', Twist, queue_size=5)

        #辅助裁判函数，计时并判断比赛完成
        #self.JugeInit()
        while not rospy.is_shutdown():
            self.GetVelocity()
            self.rate.sleep()

    def GetVelocity(self):
        if self.line_center<0 :
            return

        search_flag = False
        velocity=Twist()
        bias = self.image_width/2- self.line_center#距离中心偏移
        K_p = 0.0030#   0.0023     # PID参数
        #print("bias: %s"%bias)
        if bias >=self.image_width/2 or bias <= -self.image_width/2 or \
           (bias > -self.center_band and  bias < self.center_band):
            search_flag = False
            if bias >=self.image_width/2 or bias <= -self.image_width/2:
                search_flag = True
                #print("I can't see the line!")        
            bias = 0.0 #清零

        if search_flag == True:
            velocity.linear.x =0.15#0.15
            velocity.angular.z = 0.86*1.5 if self.last_bias>=0 else -0.86*1.5
        else:
            velocity.linear.x =0.10+ 0.20*(1.0- abs(bias)/(self.image_width/2) )
            velocity.angular.z = K_p*bias

        velocity.angular.z = self.lidar_avoid(velocity.angular.z)
        if velocity.angular.z > 1.3:
            velocity.angular.z = 1.3
            velocity.linear.x = 0.2
        if velocity.angular.z < -1.4:
            velocity.angular.z = -1.4
            velocity.linear.x = 0.2
        velocity.linear.x*=2
        velocity.angular.z*=1
        self.velocityPub_.publish(velocity)
        if abs(bias)>=self.center_band:
            self.last_bias=bias

    def posCallback(self, msg):
        self.line_center=msg.data
        #self.GetVelocity()
    def lidarCallback(self,msg):
        self.lidar_data=np.array(msg.data)
        # test
        #print(self.lidar_data[0],self.lidar_data[180],self.lidar_data[360],self.lidar_data[540])
        #print(type(self.lidar_data))

    def lidar_avoid(self,angular):
        # use lidar
        K_p = 0.9
        lidar_cut = self.lidar_data[180:540]
        max_idx = np.argmax(lidar_cut)
        right_min_idx = np.argmin(lidar_cut[0:max_idx])
        if (lidar_cut[right_min_idx]) < 0.32:
            dir_idx = max_idx + 80
            rospy.logwarn("close!!!")
        else:
            dir_idx = max_idx
        dir = ((dir_idx + 180)*0.5-180)/180*3.15
        
        angular_new = angular + K_p*dir
        return angular_new

    def JugeInit(self):
        self.start_time=-1
        self.JudgeInfo = rospy.Subscriber('/AKM_1/odom', Odometry, self.JudgeCallback)
        rospy.loginfo("输入任意键开始比赛：")
        a=raw_input()
        self.start_time=time.clock()
        self.finish_flag=False
        self.pose=TwistStamped().twist
        self.task_thread=threading.Thread(target=self.JudgeTask)
        self.task_thread.setDaemon(True)
        self.task_thread.start()
        rospy.on_shutdown(self.ExitTask)

    def JudgeTask(self):
        while not rospy.is_shutdown():
            os.system('clear')
            self.time_now=time.clock()

            if self.finish_flag==False:
                print('当前用时: %ss'%(self.time_now-self.start_time))
                print(" ")
                print("比赛未完成")
            else:
                print('比赛用时: %ss'%self.finish_time)
                print(" ")
                print("比赛已完成！")
            x=self.pose.linear.x
            y=self.pose.linear.y
            if self.time_now>7 and abs(x-0.625)<=0.05 and abs(y+1.75)<1.2:
                self.finish_flag=True
                self.finish_time=time.clock()
            time.sleep(0.1)

    def ExitTask(self,signum=None, frame=None):
        for i in range(10):
            self.velocityPub_.publish(Twist())

    def JudgeCallback(self,msg):
        new_pose=TwistStamped().twist
        new_pose.linear.x=msg.pose.pose.position.x
        new_pose.linear.y=msg.pose.pose.position.y
        new_pose.linear.z=msg.pose.pose.position.z
        self.pose=new_pose





if __name__ == '__main__':
    cn = MotionNode()