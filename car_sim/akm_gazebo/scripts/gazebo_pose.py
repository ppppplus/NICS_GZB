#!/usr/bin/env python
import rospy
import time
import math
import threading
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelStates

class GazeboPose():
    def __init__(self):
        rospy.init_node('gazebo_pose')
        # self.ros_spin_thread = threading.Thread(target=rospy.spin)
        # self.ros_spin_thread.setDaemon(True)
        # self.ros_spin_thread.start()
    
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
        rospy.spin()

    def pose_callback(self, model_states):
        name_list = model_states.name
        akm_index_list = []
        for i, name in enumerate(name_list):
            if 'AKM' in name:
                akm_index_list.append(i)
        for akm_index in akm_index_list:
            akm_name = name_list[akm_index]
            akm_pose = model_states.pose[akm_index]
            twist = self.Pose2Twist(akm_pose, akm_name)
            pub = rospy.Publisher('/'+akm_name+'/pose', TwistStamped, queue_size=1)
            pub.publish(twist)

    def Pose2Twist(self, akm_pose, akm_name):
        qx = akm_pose.orientation.x
        qy = akm_pose.orientation.y
        qw = akm_pose.orientation.w
        qz = akm_pose.orientation.z
        roll = math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))
        pitch = math.asin(2*(qw*qy-qz*qx))
        yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qz*qz+qy*qy))
        
        twist = TwistStamped()
        
        twist.twist.linear.x = akm_pose.position.x
        twist.twist.linear.y = akm_pose.position.y
        twist.twist.linear.z = akm_pose.position.z
        twist.twist.angular.x = roll
        twist.twist.angular.y = pitch
        twist.twist.angular.z = yaw
        return twist