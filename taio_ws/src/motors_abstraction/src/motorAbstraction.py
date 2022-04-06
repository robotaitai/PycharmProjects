#!/usr/bin/env python3
import subprocess
from tokenize import String

import rospy
import yaml, time
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header
from dataclasses import dataclass
import canMotorController as mot_con
import numpy as np

KP_VALUE = 200
KD_VALUE = 5
SPEED_VALUE = 0
TORQUE_VALUE = 0
INIT_ANGLE = 0
CAN_BUS = 'can0'
verbose = False
JOINT_PARAMS_PATH = "/home/taio/PycharmProjects/taio_ws/src/motors_watchdog/parameters/joint_params"

# pub_error = rospy.Publisher("/soma/watchdog_error", String, queue_size=10)
COMM_FREQ = 40  # Hz

if verbose:
    def verboseprint(args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        # pub_error.publish(args)
        print(args)
else:
    verboseprint = lambda *a: None  # do-nothing function

@dataclass
class CanParams:
    can_id: hex
    can_socket: str

@dataclass
class MotorLimits:
    min_pos: float
    max_pos: float
    max_vel: float
    max_torque: float

@dataclass
class MotorStatus:
    act_pos: float
    act_vel: float
    act_torque: float

    des_pos: float
    mot_id: hex

    def update_status(self, new_pos, new_vel, new_torque):
        self.act_pos = new_pos
        self.act_vel = new_vel
        self.act_torque = new_torque


    def update_from_motor(self,status):
        self.mot_id, self.act_pos, self.act_vel ,self.act_torque = status


@dataclass
class RosParams:
    subscribing_to: str
    publishing_as: str
    publishing_err: str
    # publisher : rospy.Publisher(publishing_as, JointState, queue_size=10)
    # error_publisher : rospy.Publisher(publishing_err, String, queue_size=10)


class motor_abstractor:

    def __init__(self, parsed_joint_params, joint, new_angle=0):
        self.joint_name = joint
        self.can_params = CanParams(parsed_joint_params[joint]['can_id'],parsed_joint_params[joint]['can_socket'])

        # self.can_id = parsed_joint_params[joint]['can_id']
        self.motor_type = parsed_joint_params[joint]['motor_type']
        self.motor_controller = mot_con.CanMotorController(self.can_params.can_socket, self.can_params.can_id, self.motor_type)

        self.ros_params = RosParams("/soma/" + joint + "/limited", "/motors/" + joint + "/status", "/motors/" + joint + "/error")
        # self.subscribing_to = '/soma/' + joint + "/limited"
        # self.publishing_as = "/motors/" + joint + "/status"
        # self.publishing_err = "/motors/" + joint + "/error"

        self.publisher = rospy.Publisher(self.ros_params.publishing_as, JointState, queue_size=10)
        self.error_publisher = rospy.Publisher(self.ros_params.publishing_err, String, queue_size=10)
        self.subscriber = rospy.Subscriber(self.ros_params.subscribing_to, JointState, self.update_desired_angle)
        self.des_angle = new_angle
        print("Created new Motor for: {} CAN ID: {} Subs to: {} and Pubs to: {}".format(self.joint_name, self.can_id,
                                                                                        self.ros_params.subscribing_to,
                                                                                        self.ros_params.publishing_as))

    def send_position_and_update_status_demo(self, desired_position):
        '''
        Send desired position over can
        and update the received status
        '''
        verboseprint(
            "{} Sending pos: {} to Motor: {}".format(round(time.time() * 1000), desired_position, self.joint_name))
        can_id, pos, vel, curr = self.can_params.can_id, desired_position, desired_position, desired_position
        # TODO check if ID is correct
        new_status = JointState()
        new_status.header = Header()
        new_status.header.stamp = rospy.Time.now()
        new_status.name = self.joint_name + '/status'
        new_status.position.append(pos)
        new_status.velocity.append(vel)
        new_status.effort.append(curr)
        self.publisher.publish(new_status)

    def send_position_and_update_status(self, desired_position):
        '''
        Send desired position over can
        and update the received status
        '''
        motor_status = self.motor_controller.send_rad_command(desired_position, SPEED_VALUE, KP_VALUE, KD_VALUE,
                                                              TORQUE_VALUE)
        if motor_status == None:
            self.error_publisher.publish("TIMEOUT EXCEPTION: Got NONE message from motor")
        else:
            motor_id, pos, vel, curr = motor_status
            # TODO check if ID is correct
            new_status = JointState()
            new_status.header = Header()
            new_status.header.stamp = rospy.Time.now()
            new_status.name = self.joint_name + '/status'
            new_status.position.append(pos)
            new_status.velocity.append(vel)
            new_status.effort.append(curr)
            self.publisher.publish(new_status)

    def motor_init(self):
        can_id, pos, vel, curr = self.motor_controller.enable_motor()
        # TODO read status(?)

    def motor_stop(self):
        can_id, pos, vel, curr = self.motor_controller.disable_motor()

    def motor_set_zero(self):
        can_id, pos, vel, curr = self.motor_controller.set_zero_position()

    def publish_status(self, status):
        self.publisher.publish(status)

    def update_desired_angle(self, msg):
        self.des_angle = msg.position[0]



class motors_handler:

    def __init__(self, joint_params):
        self.joints = []
        parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)
        for joint in parsed_joint_params:
            if joint == 'logitech_controller':
                continue
            self.joints.append(motor_abstractor(parsed_joint_params, joint, INIT_ANGLE))

    def publish_all_motors(self, event=None):
        for joint in self.joints:
            joint.send_position_and_update_status_demo(joint.des_angle)  # TODO Demo
            rospy.sleep(0.0001)

    #TODO add



if __name__ == '__main__':
    rospy.init_node('motors_abstraction', anonymous=True)

    # rc = subprocess.call("/home/taio/PycharmProjects/taio_ws/src/motors_abstraction/src/setup_can_interfacem3n3t3.sh")
    motors_handler = motors_handler(JOINT_PARAMS_PATH)
    # rate = rospy.Rate(COMM_FREQ)
    rospy.Timer(rospy.Duration(1.0 / COMM_FREQ), motors_handler.publish_all_motors)
    try:

        # while not rospy.is_shutdown():
        #     verboseprint("----------------------------------")
        #     for joint in Joints:
        #         joint.publish_status(joint.send_position_and_update_status_demo(joint.des_angle)) #TODO Demo
        #         rospy.sleep(0.0001)
        #     rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("EXCEPTION EXCEPTION")
        pass
