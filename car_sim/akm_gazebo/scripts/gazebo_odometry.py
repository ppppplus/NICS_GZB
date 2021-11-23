#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros

class OdometryNode:

    def __init__(self, robot_name):
        # init internals
        self.robot_name = robot_name
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        self.tf_pub = tf2_ros.TransformBroadcaster()
        # Set publishers
        self.pub_odom = rospy.Publisher('~odom', Odometry, queue_size=1)
        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)
        # Set the update rate
        # rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz


        

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            arrayIndex = msg.name.index(self.robot_name+'::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = '/odom'
        cmd.child_frame_id = '/base_link'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        cmd.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
						0, 1e-3, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e3]

        cmd.twist.covariance = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


        self.pub_odom.publish(cmd)

        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    robot_name = rospy.get_param("robot_name", "AKM_1")
    rospy.init_node("gazebo_odometry_node")
    # rospy.loginfo("robotname:[%s]"%robot_name)
    node = OdometryNode(robot_name)
    rospy.spin()
