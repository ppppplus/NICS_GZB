#!/usr/bin/python
#-*- encoding: utf8 -*-
import rospy
from geometry_msgs.msg import Twist
import time
import os


class ActionNode():
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)
        rospy.logwarn('motion node set up.')
        self.rate=rospy.Rate(40)
        # 发布控制速度
        self.pub=rospy.Publisher('/AKM_1/cmd_vel', Twist, queue_size=5)
        self.main()
    
    def main(self):
        while not rospy.is_shutdown():
            os.system('clear')
            print(" ")
            print("----------控制指令如下------------")
            print("--- 前进速度 转向速度 持续时间 ---")
            print("--- x(m/s) w(rad/s) t(s)   ---")
            print("--------- 输入exit退出------------")
            print(" ")
            print(" ")
            ctl=str(raw_input("请输出控制指令:    "))
            head=ctl.split(' ')

            if len(head)==3 :
                try:
                    x=float(head[0])
                    z=float(head[1])
                    t=float(head[2])
                    if t>100:
                        print("时间过长")
                    if t<0.5:
                        print("时间过短")
                except:
                    print("指令错误")
                    continue
                
                vel=Twist()
                vel.linear.x=x
                vel.angular.z=z

                for i in range(int(abs(t)/0.025)):
                    self.pub.publish(vel)
                    self.rate.sleep()
                for i in range(8):
                    self.pub.publish(Twist())
                    self.rate.sleep()
                
                
            elif len(head)==1 and head[0]=='exit' :
                break
            else :
                print("指令格式错误")
                time.sleep(0.5) 


if __name__== '__main__':
    an=ActionNode()