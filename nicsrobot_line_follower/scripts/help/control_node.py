#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from geometry_msgs.msg import Twist
import time
import os


def check_data(head,num):
    try:
        x=float(head[num])
    except:
        print("指令错误")
        time.sleep(0.5)
        return None
    return x

def pub_action(pub,vel,t):
    for i in range(int(abs(t)/0.025)):
        pub.publish(vel)
        time.sleep(0.025)
    for i in range(8):
        pub.publish(Twist())
        time.sleep(0.025)

def sign(x):
    return 1 if x>=0 else -1


class ActionNode():
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)
        rospy.logwarn('motion node set up.')
        # 发布控制速度
        self.pub=rospy.Publisher('/AKM_1/cmd_vel', Twist, queue_size=1)
        self.main()

    def main(self):
        while  not rospy.is_shutdown():
            os.system('clear')
            ctl=str(raw_input("请输出控制指令:    "))
            head=ctl.split(' ')
            if len(head)>0 and head[0]=='help':
                print(" ")
                print("--------控制指令如下----------")
                print("---向前走x(m)，x可为负数---")
                print("   forward x ")
                print("---向右转x(度)，x可为负数---")
                print("   turn right x")
                print("---向左转x(度)，x可为负数---")
                print("   turn left x")
                print("----------------------------")
                print(" ")
            elif len(head)>1 and head[0]=='forward':
                x=check_data(head,1)
                if x==None :
                    continue
                vel=Twist()
                vel.linear.x=1*sign(x)
                pub_action(self.pub,vel,x)
            elif len(head)>2 and head[0]=='turn' and head[1]=='right':
                x=check_data(head,2)
                vel=Twist()
                vel.linear.x=2/4*sign(x)
                vel.angular.z=3.1415926535/4#45度每秒
                pub_action(self.pub,vel,x/45)
            elif len(head)>2 and head[0]=='turn' and head[1]=='left':
                x=check_data(head,2)
                vel=Twist()
                vel.linear.x=2/4*sign(x)
                vel.angular.z=-3.1415926535/4#45度每秒
                pub_action(self.pub,vel,x/45) 
            elif len(head)>0 and head[0]=='exit':
                break
            else :
                print("指令格式错误") 
                time.sleep(0.5)

            






        


    def Callback(self):
        #原计划接受rostopic来执行动作
        pass
    

if __name__== '__main__':
    an=ActionNode()