#!/usr/bin/env python3
import math
import time
from collections import defaultdict

import rospy
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header
from dataclasses import dataclass


KNEE_FACTOR = 0.2

@dataclass
class dof_pos():
    act_pos: float
    des_pos: float
    delta: float

    def update_new_des(self):
        self.des_pos =+ self.delta


JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/src/parameters/joint_params"

verbose = True
if verbose:
    def verboseprint(args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        # pub_error.publish(args)
        print(args)
else:
    verboseprint = lambda *a: None      # do-nothing function




joints_pubs= defaultdict(lambda: defaultdict(list))
joints=[]
COMM_FREQ = 100  # Hz


def callback(msg):
   joints_pubs['left-knee']['point'].delta = msg.axes[3]/COMM_FREQ
   joints_pubs['left-hip-pitch']['point'].delta = msg.axes[4]/COMM_FREQ
   joints_pubs['left-ankle-pitch']['point'].delta = msg.axes[1]/COMM_FREQ
   joints_pubs['left-hip-roll']['point'].delta = msg.axes[0]/COMM_FREQ
   joints_pubs['left-hip-yaw']['point'].delta = msg.axes[6]/COMM_FREQ
   joints_pubs['left-ankle-pitch']['point'].delta = msg.axes[7]/COMM_FREQ
   # joints_pubs['left-ankle-pitch']['point'].delta = -(msg.buttons[0])/COMM_FREQ



def init_motors(joint_params):
    parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)
    for joint in parsed_joint_params:
        joints_pubs[joint]['name'] =  '/happiness/'+joint
        joints_pubs[joint]["pub"] = rospy.Publisher(joints_pubs[joint]['name'], JointState, queue_size=10)
        joints_pubs[joint]["point"] = dof_pos(act_pos=0, des_pos=0, delta=0)
        joints.append(joint)
        print(f"Joint {joint} was added to the Joystick Command")


def publish_all_motors( event=None):
    # for joint in self.joints:
    #     joint.send_position_and_update_status(joint.des_angle)  # TODO Demoif __name__ == '__main__':
    for joint in joints:
        new_msg = JointState()
        new_msg.header = Header()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.velocity = [0.0]
        new_msg.effort = [0]
        new_msg.name = joints_pubs[joint]['name']
        joints_pubs[joint]["point"].des_pos += joints_pubs[joint]["point"].delta
        new_msg.position = [joints_pubs[joint]["point"].des_pos]

        # joints_pubs[joint]["point"].act_pos = joints_pubs[joint]["point"].des_pos
        joints_pubs[joint]["pub"].publish(new_msg)
        # print(new_msg)
        rospy.sleep(0.0001)



JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/src/parameters/joint_params"
COMM_FREQ = 100  # Hz


if __name__ == '__main__':
    rospy.init_node('joy_teleop')

    init_motors(JOINT_PARAMS_PATH)
    # Get parameters from the server
    # global speed_factor
    # speed_factor = rospy.get_param('~speed_factor', 10.0)
    # rospy.loginfo('Using speed_factor: [%.1f]' % speed_factor)
    # # Susbscribe to the topic that contains the ps3 keys
    rospy.Subscriber('/joy', Joy, callback)
    rospy.Timer(rospy.Duration(1.0/ COMM_FREQ), publish_all_motors)

    # Keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("EXCEPTION EXCEPTION")
        pass