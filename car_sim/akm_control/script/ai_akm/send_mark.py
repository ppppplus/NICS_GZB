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
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.markerArray = MarkerArray() #目标点标记数组
        self.markerArray_number = MarkerArray() #目标点标记数组
        self.count = 0
        self.index = 0
        self.try_again = 1
        self.mark_pub = rospy.Publisher('path_point', MarkerArray, queue_size = 100) #用于发布所有目标点
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 1) #用于发布当前目标点
        self.sendflagPublisher = rospy.Publisher('send_flag', Int8, queue_size =1)
        
        # rospy.Subscriber('send_mark_goal', PoseStamped, navGoal_callback) #订阅rviz内标记按下的位置
        rospy.Subscriber('clicked_point', PointStamped, self.click_callback) #订阅rviz内标记按下的位置
        
        send_flag=Int8()
        send_flag.data=1
        self.sendflagPublisher.publish(send_flag)
        rospy.sleep(1.)
        self.sendflagPublisher.publish(send_flag)
        rospy.loginfo('a=%d',send_flag.data)
        goal_status_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.pose_callback) #用于订阅是否到达目标点状态


        while not rospy.is_shutdown():
            key = self.getKey() #获取键值
            if(key=='c'): #键值为c是清空目标点
                self.count = 0
                self.index = 0
                self.try_again = 1

                self.markerArray = MarkerArray() 
                marker = Marker()
                marker.header.frame_id = 'AKM_1/map' 
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
    def click_callback(self, msg):           
        rospy.loginfo('Add a new target point '+str(self.count)+':')
        rospy.loginfo('x:'+str(msg.point.x)+
            ', y:'+str(msg.point.y)+
            ', z:0'+', w:1') 

        marker = Marker()      #创建marker对象
        marker.header.frame_id = self.robot_name + '/map' #以哪一个TF坐标为原点
        marker.type = marker.ARROW #一直面向屏幕的字符格式
        marker.action = marker.ADD #添加marker
        marker.scale.x = 0.2 #marker大小
        marker.scale.y = 0.05 #marker大小
        marker.scale.z = 0.05 #marker大小，对于字符只有z起作用
        marker.color.a = 1 #字符透明度
        marker.color.r = 1 #字符颜色R(红色)通道
        marker.color.g = 0 #字符颜色G(绿色)通道
        marker.color.b = 0 #字符颜色B(蓝色)通道
        marker.pose.position.x = msg.point.x #字符位置
        marker.pose.position.y = msg.point.y #字符位置
        marker.pose.orientation.z = 0 #字符位置
        marker.pose.orientation.w = 1 #字符位置
        self.markerArray.markers.append(marker) #添加元素进数组

        marker_number = Marker()      #创建marker对象
        marker_number.header.frame_id = 'AKM_1/map' #以哪一个TF坐标为原点
        marker_number.type = marker_number.TEXT_VIEW_FACING #一直面向屏幕的字符格式
        marker_number.action = marker_number.ADD #添加marker
        marker_number.scale.x = 0.5 #marker大小
        marker_number.scale.y = 0.5 #marker大小
        marker_number.scale.z = 0.5 #marker大小，对于字符只有z起作用
        marker_number.color.a = 1 #字符透明度
        marker_number.color.r = 1 #字符颜色R(红色)通道
        marker_number.color.g = 0 #字符颜色G(绿色)通道
        marker_number.color.b = 0 #字符颜色B(蓝色)通道
        marker_number.pose.position.x = msg.point.x #字符位置
        marker_number.pose.position.y = msg.point.y #字符位置
        marker_number.pose.position.z = 0.1 #字符位置
        marker_number.pose.orientation.z = 0 #字符位置
        marker_number.pose.orientation.w = 1 #字符位置
        marker_number.text = str(self.count) #字符内容
        self.markerArray_number.markers.append(marker_number) #添加元素进数组

        #markers的id不能一样，否则rviz只会识别最后一个元素
        id = 0
        for m in self.markerArray.markers:    #遍历marker分别给id赋值
            m.id = id
            id += 1

        for m in self.markerArray_number.markers:    #遍历marker分别给id赋值
            m.id = id
            id += 1

        self.mark_pub.publish(self.markerArray) #发布markerArray，rviz订阅并进行可视化
        self.mark_pub.publish(self.markerArray_number) #发布markerArray，rviz订阅并进行可视化

        #第一次添加marker时直接发布目标点
        if self.count == 0:
            pose = PoseStamped() #创建目标点对象
            pose.header.frame_id = 'AKM_1/map' #以哪一个TF坐标为原点
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = msg.point.x #目标点位置
            pose.pose.position.y = msg.point.y #目标点位置
            pose.pose.orientation.z = 0 #四元数，到达目标点后小车的方向，z=sin(angle/2)
            pose.pose.orientation.w = 1 #四元数，到达目标点后小车的方向，w=cos(angle/2)
            self.goal_pub.publish(pose)
            self.index += 1 #下一次要发布的目标点序号

        self.count += 1 #有几个目标点


    #到达目标点成功或失败的回调函数，输入参数：[3：成功， 其它：失败](4：ABORTED)
    def pose_callback(self, msg):
        if msg.status.status == 3 and self.count>0 :  #成功到达任意目标点，前往下一目标点
            self.try_again = 1 #允许再次尝试前往尚未抵达的该目标点

            #count表示当前目标点计数，index表示已完成的目标点计数
            if self.index == self.count:                   #当index等于count时，表示所有目标点完成，重新开始巡航
                print ('Reach the target point '+str(self.index-1)+':')
                print('x:'+str(self.markerArray.markers[self.index-1].pose.position.x)+
                    ', y:'+str(self.markerArray.markers[self.index-1].pose.position.y)+
                    ', z:'+str(self.markerArray.markers[self.index-1].pose.orientation.z)+
                    ', w:'+str(self.markerArray.markers[self.index-1].pose.orientation.w))   

                if self.count>1: print 'Complete instructions!' #只有一个目标点不算巡航
                self.index = 0
                pose = PoseStamped()
                pose.header.frame_id = 'AKM_1/map'
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
                pose.header.frame_id = 'AKM_1/map'
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
                pose.header.frame_id = 'AKM_1/map'
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
                pose.header.frame_id = 'AKM_1/map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = self.markerArray.markers[self.index].pose.position.x      
                pose.pose.position.y = self.markerArray.markers[self.index].pose.position.y
                pose.pose.orientation.z = self.markerArray.markers[self.index].pose.orientation.z
                pose.pose.orientation.w = self.markerArray.markers[self.index].pose.orientation.w
                self.goal_pub.publish(pose)
                self.index += 1 #下一次要发布的目标点序号
                self.try_again = 1 #允许再次尝试前往尚未抵达的该目标点

    #rviz内NavGoal标记按下的回调函数，输入参数：按下的位置[x, y, z=0]
    def navGoal_callback(self, msg):           
        print('Add a new target point '+str(self.count)+':')
        print('x:'+str(msg.pose.position.x)+
            ', y:'+str(msg.pose.position.y)+
            ', z:'+str(msg.pose.orientation.z)+
            ', w:'+str(msg.pose.orientation.w)) 

        marker = Marker()      #创建marker对象
        marker.header.frame_id = 'AKM_1/map' #以哪一个TF坐标为原点
        marker.type = marker.ARROW#TEXT_VIEW_FACING #一直面向屏幕的字符格式
        marker.action = marker.ADD #添加marker
        marker.scale.x = 0.5 #marker大小
        marker.scale.y = 0.05 #marker大小
        marker.scale.z = 0.05 #marker大小，对于字符只有z起作用
        marker.color.a = 1 #字符透明度
        marker.color.r = 1 #字符颜色R(红色)通道
        marker.color.g = 0 #字符颜色G(绿色)通道
        marker.color.b = 0 #字符颜色B(蓝色)通道
        marker.pose.position.x = msg.pose.position.x #字符位置
        marker.pose.position.y = msg.pose.position.y #字符位置
        marker.pose.position.z = 0.1#msg.position.z #字符位置
        marker.pose.orientation.z = msg.pose.orientation.z #字符位置
        marker.pose.orientation.w = msg.pose.orientation.w #字符位置
        self.markerArray.markers.append(marker) #添加元素进数组

        marker_number = Marker()      #创建marker对象
        marker_number.header.frame_id = 'AKM_1/map' #以哪一个TF坐标为原点
        marker_number.type = marker_number.TEXT_VIEW_FACING #一直面向屏幕的字符格式
        marker_number.action = marker_number.ADD #添加marker
        marker_number.scale.x = 0.5 #marker大小
        marker_number.scale.y = 0.5 #marker大小
        marker_number.scale.z = 0.5 #marker大小，对于字符只有z起作用
        marker_number.color.a = 1 #字符透明度
        marker_number.color.r = 1 #字符颜色R(红色)通道
        marker_number.color.g = 0 #字符颜色G(绿色)通道
        marker_number.color.b = 0 #字符颜色B(蓝色)通道
        marker_number.pose.position.x = msg.pose.position.x #字符位置
        marker_number.pose.position.y = msg.pose.position.y #字符位置
        marker_number.pose.position.z = 0.1#msg.position.z #字符位置
        marker_number.pose.orientation.z = msg.pose.orientation.z #字符位置
        marker_number.pose.orientation.w = msg.pose.orientation.w #字符位置
        marker_number.text = str(self.count) #字符内容
        self.markerArray_number.markers.append(marker_number) #添加元素进数组

        #markers的id不能一样，否则rviz只会识别最后一个元素
        id = 0
        for m in self.markerArray.markers:    #遍历marker分别给id赋值
            m.id = id
            id += 1

        for m in self.markerArray_number.markers:    #遍历marker分别给id赋值
            m.id = id
            id += 1

        self.mark_pub.publish(self.markerArray) #发布markerArray，rviz订阅并进行可视化
        self.mark_pub.publish(self.markerArray_number) #发布markerArray，rviz订阅并进行可视化

        #第一次添加marker时直接发布目标点
        if self.count == 0:
            pose = PoseStamped() #创建目标点对象
            pose.header.frame_id = 'AKM_1/map' #以哪一个TF坐标为原点
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = msg.pose.position.x #目标点位置
            pose.pose.position.y = msg.pose.position.y #目标点位置
            pose.pose.orientation.z = marker.pose.orientation.z
            pose.pose.orientation.w = marker.pose.orientation.w
            self.goal_pub.publish(pose)
            self.index += 1 #下一次要发布的目标点序号

        self.count += 1 #有几个目标点

    

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
    robot_name = rospy.get_param('robot_name', 'AKM_1')
    node = RotNav(robot_name)

