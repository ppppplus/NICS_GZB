#!/usr/bin/env python
import rospy, math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)

def set_throttle_steer(data):

    global flag_move
    # K_vel = 0.355
    K_vel = 0.71
    # K_phi = 1.1847
    K_phi = 0.5

    pub_vel_left_rear_wheel = rospy.Publisher('left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('right_steering_hinge_position_controller/command', Float64, queue_size=1)
    v = data.linear.x*K_vel
    # steer = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)*K_phi
    steer = data.angular.z*K_phi
    throttle = v/0.1

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

def servo_commands():
    global wheelbase
    rospy.init_node('servo_commands', anonymous=True)
    wheelbase = rospy.get_param('wheelbase', 0.2)
    rospy.Subscriber("cmd_vel", Twist, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
