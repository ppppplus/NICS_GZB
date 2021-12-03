#!/usr/bin/env python
import rospy
import os
import sys
import math
from gazebo_msgs.srv import *

def set_model_state(name, x, y, theta):
    set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    objstate = SetModelStateRequest()
    objstate.model_state.model_name = name
    objstate.model_state.pose.position.x = float(x)
    objstate.model_state.pose.position.y = float(y)
    objstate.model_state.pose.position.z = 0.5
    ow, ox, oy, oz = RPY2Quar(float(theta))
    objstate.model_state.pose.orientation.x = ox
    objstate.model_state.pose.orientation.y = oy
    objstate.model_state.pose.orientation.z = oz
    objstate.model_state.pose.orientation.w = ow

    set_state_srv(objstate)

def RPY2Quar(theta):
    cy = math.cos(theta * 0.5)
    sy = math.sin(theta * 0.5)
    cp = math.cos(0)
    sp = math.sin(0)
    cr = math.cos(0)
    sr = math.sin(0)
 
    ow = cy * cp * cr + sy * sp * sr
    ox = cy * cp * sr - sy * sp * cr
    oy = sy * cp * sr + cy * sp * cr
    oz = sy * cp * cr - cy * sp * sr
    return ow, ox, oy, oz
        
if __name__ == '__main__':
    arg_num = len(sys.argv[1:-1])
    robot_name = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]
    theta = sys.argv[4] if arg_num > 2 else 0
    rospy.init_node("set_state_srv")
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state(robot_name, x, y, theta)
    
