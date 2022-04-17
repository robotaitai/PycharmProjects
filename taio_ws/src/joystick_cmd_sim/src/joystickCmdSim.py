#!/usr/bin/env python3
import math
import time, json
from collections import defaultdict

import rospy
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header, String
from dataclasses import dataclass

Kp_VAL = 200
Kd_VAL = 0.1
VEL_VAL = 0
TOR_VAL = 0

KNEE_FACTOR = 0.2
SPEED_FACTOR = 2
global msg_id
msg_id = 0


@dataclass
class dof_pos():
    act_pos: float
    des_pos: float
    delta: float

    def update_new_des(self):
        self.des_pos =+ self.delta

JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/src/parameters/joint_params"


joints_pubs= defaultdict(lambda: defaultdict(list))
joints=[]
COMM_FREQ = 20  # Hz
motors_handler_pubs = rospy.Publisher("/happiness/motors_handler", String, queue_size=10)


def callback(msg):
   joints_pubs['left_knee']['point'].delta = msg.axes[3]/COMM_FREQ/SPEED_FACTOR
   joints_pubs['left_hip_pitch']['point'].delta = msg.axes[4]/COMM_FREQ/SPEED_FACTOR
   joints_pubs['left_ankle_pitch']['point'].delta = msg.axes[1]/COMM_FREQ/SPEED_FACTOR
   joints_pubs['left_hip_roll']['point'].delta = msg.axes[0]/COMM_FREQ/SPEED_FACTOR
   joints_pubs['left_hip_yaw']['point'].delta = msg.axes[6]/COMM_FREQ/SPEED_FACTOR
   joints_pubs['left_ankle_pitch']['point'].delta = msg.axes[7]/COMM_FREQ/SPEED_FACTOR
   # joints_pubs['left-ankle-pitch']['point'].delta = -(msg.buttons[0])/COMM_FREQ
   if msg.buttons[0] == 1:
       print("INIT MOTORS")
       motors_handler_pubs.publish("init")
   if msg.buttons[2] == 1:
       print("ZEROS MOTORS")
       motors_handler_pubs.publish("zero")
   if msg.buttons[1] == 1:
       print("STOP MOTORS")
       motors_handler_pubs.publish("stop")

def init_motors(joint_params):
    parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)
    for joint in parsed_joint_params:
        joints_pubs[joint]['name'] ='/happiness/'+joint
        joints_pubs[joint]["pub"] = rospy.Publisher(joints_pubs[joint]['name'], String, queue_size=10)
        joints_pubs[joint]["point"] = dof_pos(act_pos=0, des_pos=0, delta=0)
        joints.append(joint)
        print(f"Joint {joint} was added to the Joystick Command")

# def ros_command_(joint_name, position, velocity = VEL_VAL, torque=TOR_VAL, kp = Kp_VAL, kd = Kd_VAL):



def publish_all_motors( event=None):
    global msg_id
    msg_id += 1

    for joint in joints:
        joints_pubs[joint]["point"].des_pos += joints_pubs[joint]["point"].delta
        new_msg = {'message_id': msg_id ,'name': joints_pubs[joint]['name'], 'time_stamp': time.time(),
                                      'position':joints_pubs[joint]["point"].des_pos, 'velocity': VEL_VAL, 'torque': TOR_VAL,
                                                  'kp': Kp_VAL, 'kd': Kd_VAL}
        new_msg_json = json.dumps(new_msg)
        joints_pubs[joint]["pub"].publish(str(new_msg_json))
        del new_msg
        # print(new_msg)
        rospy.sleep(0.0001)



JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/src/parameters/joint_params"
COMM_FREQ = 100  # Hz


if __name__ == '__main__':
    rospy.init_node('joy_teleop')

    init_motors(JOINT_PARAMS_PATH)

    rospy.Subscriber('/joy', Joy, callback)
    rospy.Timer(rospy.Duration(1.0/ COMM_FREQ), publish_all_motors)

    # Keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("EXCEPTION EXCEPTION")
        pass