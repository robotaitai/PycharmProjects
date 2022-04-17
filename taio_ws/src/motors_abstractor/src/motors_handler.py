import time

import yaml, rospy
import motors_abstractor
from std_msgs.msg import String


INIT_ANGLE = 0 #TODO
COMM_FREQ = 100  # Hz


class MotorsHandler:
    def __init__(self, joint_params):
        self.joints = {}
        self.dofs = []
        self.subscriber = rospy.Subscriber("/happiness/motors_handler", String, self.ros_actions)
        parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)
        for dof in parsed_joint_params:
            self.dofs.append(dof)
        for dof in self.dofs:
            self.joints[dof] = motors_abstractor.MotorsAbstractor(parsed_joint_params[dof], INIT_ANGLE)

    def init_motors(self):
        print("Init motors Command Received")
        for dof in self.dofs:
            self.joints[dof].motor_set_zero()
            time.sleep(0.002)
            self.joints[dof].motor_init()
        rospy.Timer(rospy.Duration(1.0 / COMM_FREQ), self.publish_all_motors)

    def set_zero_pos_motors(self):
        print("Zero motors Command Received")
        for dof in self.dofs:
            self.joints[dof].motor_set_zero()

    def disable_motors(self):
        print("Stop motors Command Received")
        for dof in self.dofs:
            self.joints[dof].motor_stop()


    def publish_all_motors(self, event=None):
        # for joint in self.joints:
        #     joint.send_position_and_update_status(joint.des_angle)  # TODO Demo
        for dof in self.dofs:
            self.joints[dof].send_position_and_update_status(self.joints[dof].des_angle)
            rospy.sleep(0.0001)

    def ros_actions(self, msg):
        print(msg)
        if msg == "init":
            self.init_motors()
        elif msg == "zero":
            self.set_zero_pos_motors()
        elif msg == "stop":
            self.disable_motors()

    def run(self):
        rospy.spin()