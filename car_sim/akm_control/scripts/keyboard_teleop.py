#!/usr/bin/env python
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import sys, select, termios, tty

banner = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
---------------------------
"""

keyBindings = {
  'u':(1,1),
  'i':(1,0),
  'o':(1,-1),
  'j':(0,1),
  'k':(0,0),
  'l':(0,-1),
  'm':(-1,1),
  ',':(-1,0),
  '.':(-1,-1)
}

speedBindings = {
  'q':(1.1,1.1),
  'z':(.9,.9),
  'w':(1.1,1),
  'x':(.9,1),
  'e':(1,1.1),
  'c':(1,.9),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 1
turn = 1
MAXTURN = 1.4

def vels(speed,turn):
  return "currently:\tlinear velocity %s\tangular velocity %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
  rospy.init_node('keyop')
  print(banner)
  x = 0
  th = 0
  status = 0

  try:
    while(1):
      key = getKey()
      if key in keyBindings.keys():
        x = keyBindings[key][0]
        th = keyBindings[key][1]
      elif key in speedBindings.keys():
        speed_mul = speedBindings[key][0]
        turn_mul = speedBindings[key][1]
        speed = speed*speed_mul
        turn = turn*turn_mul
        if turn > MAXTURN:
          turn = MAXTURN
          print("The set value of steering angle has reached the maximum!")
        print(vels(speed,turn))
      else:
        x = 0
        th = 0
        if (key == '\x03'):
          break
      msg = Twist()
      msg.linear.x = x*speed
      msg.linear.y = 0
      msg.linear.z = 0
      msg.angular.x = 0
      msg.angular.y = 0
      msg.angular.z = th*turn

      pub.publish(msg)

  except Exception as e:
    print(e)

  finally:
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
