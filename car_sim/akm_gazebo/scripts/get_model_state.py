#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import *

def get_model_state(model_name):
    get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = model_name
    objstate = get_state_srv(model)
    return objstate
def RPY2Quar(self):
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        cp = math.cos(0)
        sp = math.sin(0)
        cr = math.cos(0)
        sr = math.sin(0)
        
        self.ow = cy * cp * cr + sy * sp * sr
        self.ox = cy * cp * sr - sy * sp * cr
        self.oy = sy * cp * sr + cy * sp * cr
        self.oz = sy * cp * cr - cy * sp * sr
        
if __name__ == '__main__':
    state = get_model_state('cylinder1')
    print(state.pose.position.x)
    print(state.pose.position.y)
