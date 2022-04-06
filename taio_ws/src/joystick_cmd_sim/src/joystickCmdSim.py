#!/usr/bin/env python3
import math
import time

import rospy
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header


JOINT_PARAMS_PATH = "/home/taio/PycharmProjects/taio_ws/src/motors_watchdog/parameters/joint_params"

verbose = False
if verbose:
    def verboseprint(args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        # pub_error.publish(args)
        print(args)
else:
    verboseprint = lambda *a: None      # do-nothing function


def callback(msg):
   # print(msg)
   a = joints_pubs
   b=joints

   new_msg = JointState()
   new_msg.header = Header()
   new_msg.header.stamp = rospy.Time.now()
   new_msg.velocity = [0.0]
   new_msg.effort = [0]

   val0 = msg.axes[2]
   val1 = msg.axes[5]
   val2 = msg.axes[4]
   val3 = msg.axes[3]

   if val0 != 1.0:
      new_msg.name = "/soma/"+joints[0]
      pos = val0 * math.pi
      new_msg.position =  [pos]
      joints_pubs[joints[0]].publish(new_msg)
      verboseprint("moving joint: {} to pos: {}".format(joints[0],pos))

   if val1 != float(1.0):
      new_msg.name = "/soma/"+joints[1]
      pos = val1 * math.pi
      new_msg.position =  [pos]
      joints_pubs[joints[1]].publish(new_msg)
      verboseprint("moving joint: {} to pos: {}".format(joints[1],pos))

   if val2 != 0.0:
      new_msg.name = "/soma/"+joints[2]
      pos = val2 * math.pi
      new_msg.position =  [pos]
      joints_pubs[joints[2]].publish(new_msg)
      verboseprint("moving joint: {} to pos: {}".format(joints[2],pos))


   if val3 != 0.0:
      new_msg.name = "/soma/"+joints[3]
      pos = val3 * math.pi
      new_msg.position =  [pos]
      joints_pubs[joints[3]].publish(new_msg)
      verboseprint("moving joint: {} to pos: {}".format(joints[3],pos))
    # cmd_msg = Twist()
    # cmd_msg.linear.x = speed_factor * msg.axes[1]
    # cmd_msg.angular.z = speed_factor * msg.axes[2]
    # pub.publish(cmd_msg)

   val10 = msg.buttons[1]
   if val10 == 1.0:
      new_msg.name = "/soma/all"
      new_msg.position = [0]
      new_msg.velocity = [1.0]
      joints_pubs[2].publish(new_msg)
      verboseprint("Init all Joints")
    # cmd_msg = Twist()
    # cmd_msg.linear.x = speed_factor * msg.axes[1]
    # cmd_msg.angular.z = speed_factor * msg.axes[2]
    # pub.publish(cmd_msg)
   # time.sleep(0.1)


joints_pubs={}
joints={}
def init_motors(joint_params):
    parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)
    i = 0
    for joint in parsed_joint_params:
        joints_pubs[joint]=rospy.Publisher('/soma/'+joint, JointState, queue_size=10)
        joints[i]=joint
        i+=1

def joy_teleop():
    init_motors(JOINT_PARAMS_PATH)
    rospy.init_node('joy_teleop')
    # Get parameters from the server
    global speed_factor
    speed_factor = rospy.get_param('~speed_factor', 10.0)
    rospy.loginfo('Using speed_factor: [%.1f]' % speed_factor)
    # Susbscribe to the topic that contains the ps3 keys
    rospy.Subscriber('/joy', Joy, callback)
    # Keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    joy_teleop()
