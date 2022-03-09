#!/usr/bin/env python
# encoding: utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Int8
import actionlib
from move_base_msgs.msg import *
import sys, select, termios, tty
import time

class RotNav():
    def __init__(self, robot_name, pointy):
        self.robot_name = robot_name
        self.goal_list = [0.6,-2.6,0,-3.2,-0.4,0,-0.3,pointy,1]
        self.markerArray = MarkerArray()
        self.markerArray_number = MarkerArray()
        self.count = 3
        self.index = 0
        self.try_again = 1
        self.mark_pub = rospy.Publisher('path_point', MarkerArray, queue_size = 100)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 1)
        
        time.sleep(8)
        self.showmarker()
        # 先发布第一个目标点
        
        first_pose = PoseStamped()
        first_pose.header.frame_id = self.robot_name + '/map'
        first_pose.header.stamp = rospy.Time.now()
        first_pose.pose.position.x = self.goal_list[0]
        first_pose.pose.position.y = self.goal_list[1]
        first_pose.pose.orientation.z = 1 - self.goal_list[2] # z=sin(yaw/2)
        first_pose.pose.orientation.w = self.goal_list[2] # w=cos(yaw/2)
        self.goal_pub.publish(first_pose)
        
        self.index += 1
        goal_status_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.pose_callback) #用于订阅是否到达目标点状态


        while not rospy.is_shutdown():
            key = self.getKey() #获取键值
            if(key=='c'): #键值为c是清空目标点
                self.count = 0
                self.index = 0
                self.try_again = 1

                self.markerArray = MarkerArray() 
                marker = Marker()
                marker.header.frame_id = self.robot_name + '/map' 
                marker.type = marker.TEXT_VIEW_FACING 
                marker.action = marker.DELETEALL 
                marker.text = '' 
                self.markerArray.markers.append(marker) 

                for m in self.markerArray_number.markers:    
                    m.action = marker.DELETEALL

                self.mark_pub.publish(self.markerArray) 
                self.mark_pub.publish(self.markerArray_number) 
                self.markerArray = MarkerArray() 
                self.markerArray_number = MarkerArray() 

            elif (key == '\x03'): #ctrl+c退出
                break

    #rviz内标记按下的回调函数，输入参数：按下的位置[x, y, z=0]
    def showmarker(self):           
        for i in range(3):
            marker = Marker()      #创建marker对象
            marker.id = i
            marker.header.frame_id = self.robot_name + '/map' #以哪一个TF坐标为原点
            marker.type = marker.ARROW #一直面向屏幕的字符格式
            marker.action = marker.ADD #添加marker
            marker.scale.x = 0.5 #marker大小
            marker.scale.y = 0.05 #marker大小
            marker.scale.z = 0.05 #marker大小，对于字符只有z起作用
            marker.color.a = 1 #字符透明度
            marker.color.r = 1 #字符颜色R(红色)通道
            marker.color.g = 0 #字符颜色G(绿色)通道
            marker.color.b = 0 #字符颜色B(蓝色)通道
            marker.pose.position.x = self.goal_list[i*3] #字符位置
            marker.pose.position.y = self.goal_list[i*3+1] #字符位置
            marker.pose.orientation.z = 1 - self.goal_list[i*3+2] #字符位置
            marker.pose.orientation.w = self.goal_list[i*3+2] #字符位置
            self.markerArray.markers.append(marker) #添加元素进数组

            marker_number = Marker()      #创建marker对象
            marker_number.id = i
            marker_number.header.frame_id = self.robot_name + '/map' #以哪一个TF坐标为原点
            marker_number.type = marker_number.TEXT_VIEW_FACING #一直面向屏幕的字符格式
            marker_number.action = marker_number.ADD #添加marker
            marker_number.scale.x = 0.5 #marker大小
            marker_number.scale.y = 0.5 #marker大小
            marker_number.scale.z = 0.5 #marker大小，对于字符只有z起作用
            marker_number.color.a = 1 #字符透明度
            marker_number.color.r = 1 #字符颜色R(红色)通道
            marker_number.color.g = 0 #字符颜色G(绿色)通道
            marker_number.color.b = 0 #字符颜色B(蓝色)通道
            marker_number.pose.position.x = self.goal_list[i*3] #字符位置
            marker_number.pose.position.y = self.goal_list[i*3+1] #字符位置
            marker_number.pose.position.z = 0.1 #字符位置
            marker_number.pose.orientation.z = 0 #字符位置
            marker_number.pose.orientation.w = 1 #字符位置
            marker_number.text = str(i) #字符内容
            self.markerArray_number.markers.append(marker_number) #添加元素进数组

            self.mark_pub.publish(self.markerArray) #发布markerArray，rviz订阅并进行可视化
            self.mark_pub.publish(self.markerArray_number) #发布markerArray，rviz订阅并进行可视化


    #到达目标点成功或失败的回调函数，输入参数：[3：成功， 其它：失败](4：ABORTED)
    def pose_callback(self, msg):
        if msg.status.status == 3 and self.count>0 :  #成功到达任意目标点，前往下一目标点
            self.try_again = 1 #允许再次尝试前往尚未抵达的该目标点

            #count表示当前目标点计数，index表示已完成的目标点计数
            if self.index == self.count:                   #当index等于count时，表示所有目标点完成，重新开始巡航
                rospy.loginfo('Reach the target point '+str(self.index-1)+':')
                rospy.loginfo('x:'+str(self.markerArray.markers[self.index-1].pose.position.x)+
                    ', y:'+str(self.markerArray.markers[self.index-1].pose.position.y)+
                    ', z:'+str(self.markerArray.markers[self.index-1].pose.orientation.z)+
                    ', w:'+str(self.markerArray.markers[self.index-1].pose.orientation.w))   

                if self.count>1: print 'Complete instructions!' #只有一个目标点不算巡航
                self.index = 0
                pose = PoseStamped()
                pose.header.frame_id = self.robot_name + '/map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = self.markerArray.markers[self.index].pose.position.x
                pose.pose.position.y = self.markerArray.markers[self.index].pose.position.y
                pose.pose.orientation.z = self.markerArray.markers[self.index].pose.orientation.z
                pose.pose.orientation.w = self.markerArray.markers[self.index].pose.orientation.w
                self.goal_pub.publish(pose)
                self.index += 1 #下一次要发布的目标点序号

            elif self.index < self.count:                   #当index小于count时，表示还未完成所有目标点，目标巡航未完成
                rospy.loginfo('Reach the target point '+str(self.index-1)+':')    
                rospy.loginfo('x:'+str(self.markerArray.markers[self.index-1].pose.position.x)+
                    ', y:'+str(self.markerArray.markers[self.index-1].pose.position.y)+
                    ', z:'+str(self.markerArray.markers[self.index-1].pose.orientation.z)+
                    ', w:'+str(self.markerArray.markers[self.index-1].pose.orientation.w)) 

                pose = PoseStamped()
                pose.header.frame_id = self.robot_name + '/map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = self.markerArray.markers[self.index].pose.position.x
                pose.pose.position.y = self.markerArray.markers[self.index].pose.position.y
                pose.pose.orientation.z = self.markerArray.markers[self.index].pose.orientation.z
                pose.pose.orientation.w = self.markerArray.markers[self.index].pose.orientation.w
                self.goal_pub.publish(pose)
                self.index += 1 #下一次要发布的目标点序号

        elif self.count>0: #未抵达设定的目标点    
            rospy.logwarn('Can not reach the target point '+str(self.index-1)+':'+'\r\n'+
                          'x:'+str(self.markerArray.markers[self.index-1].pose.position.x)+
                        ', y:'+str(self.markerArray.markers[self.index-1].pose.position.y)+
                        ', z:'+str(self.markerArray.markers[self.index-1].pose.orientation.z)+
                        ', w:'+str(self.markerArray.markers[self.index-1].pose.orientation.w)) 

            #如果未尝试过前往尚未抵达的目标点，则尝试前往尚未抵达的目标点
            if self.try_again == 1:
                rospy.logwarn('trying reach the target point '+str(self.index-1)+' again!'+'\r\n'+
                              'x:'+str(self.markerArray.markers[self.index-1].pose.position.x)+
                            ', y:'+str(self.markerArray.markers[self.index-1].pose.position.y)+
                            ', z:'+str(self.markerArray.markers[self.index-1].pose.orientation.z)+
                            ', w:'+str(self.markerArray.markers[self.index-1].pose.orientation.w)) 

                pose = PoseStamped()
                pose.header.frame_id = self.robot_name + '/map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = self.markerArray.markers[self.index - 1].pose.position.x           
                pose.pose.position.y = self.markerArray.markers[self.index - 1].pose.position.y
                pose.pose.orientation.z = self.markerArray.markers[self.index-1].pose.orientation.z
                pose.pose.orientation.w = self.markerArray.markers[self.index-1].pose.orientation.w
                self.goal_pub.publish(pose)
                self.try_again = 0 #不允许再次尝试前往尚未抵达的该目标点

            #如果已经尝试过前往尚未抵达的目标点，则前往下一个目标点
            elif self.index < len(self.markerArray.markers):      #若还未完成目标点
                rospy.logwarn('try reach the target point '+str(self.index-1)+' failed! reach next point:'+'\r\n'+
                              'x:'+str(self.markerArray.markers[self.index-1].pose.position.x)+
                            ', y:'+str(self.markerArray.markers[self.index-1].pose.position.y)+
                            ', z:'+str(self.markerArray.markers[self.index-1].pose.orientation.z)+
                            ', w:'+str(self.markerArray.markers[self.index-1].pose.orientation.w)) 

                if self.index==self.count: self.index=0 #如果下一个目标点序号为count，说明当前目标点为最后一个目标点，下一个目标点序号应该设置为0
                pose = PoseStamped()
                pose.header.frame_id = self.robot_name + '/map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = self.markerArray.markers[self.index].pose.position.x      
                pose.pose.position.y = self.markerArray.markers[self.index].pose.position.y
                pose.pose.orientation.z = self.markerArray.markers[self.index].pose.orientation.z
                pose.pose.orientation.w = self.markerArray.markers[self.index].pose.orientation.w
                self.goal_pub.publish(pose)
                self.index += 1 #下一次要发布的目标点序号
                self.try_again = 1 #允许再次尝试前往尚未抵达的该目标点

    #获取键值函数
    def getKey(self):
        fd = sys.stdin.fileno()
        new_settings = termios.tcgetattr(fd)
        new_settings[3]=new_settings[3] | termios.ECHO
        try:
            # termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
            # tty.setraw(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
        return key

def breakkey():
    fd = sys.stdin.fileno()
    new_settings = termios.tcgetattr(fd)
    new_settings[3]=new_settings[3] | termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin) #获取键值初始化
    rospy.on_shutdown(breakkey)#退出前执行键值初始化
    rospy.init_node('path_point_demo') #初始化节点
    robot_name = rospy.get_param('~robot_name', 'AI_1')
    pointy = rospy.get_param('~pointy', 4)
    rospy.logwarn("ai_name:%s", robot_name)
    node = RotNav(robot_name, pointy)

