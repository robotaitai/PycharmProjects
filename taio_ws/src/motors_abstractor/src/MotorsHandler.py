import time

import yaml, rospy
import MotorsAbstractor

INIT_ANGLE = 0 #TODO


class MotorsHandler:

    def __init__(self, joint_params):
        self.joints = {}
        self.dofs = []
        parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)

        for dof in parsed_joint_params:
            if dof == 'logitech_controller':
                continue
            self.dofs.append(dof)

        for dof in self.dofs:
            self.joints[dof] = MotorsAbstractor.MotorsAbstractor(parsed_joint_params, dof, INIT_ANGLE)


    def init_motors(self):
        for dof in self.dofs:
            print(type(self.joints[dof]))
            self.joints[dof].motor_set_zero()
            time.sleep(0.002)
            self.joints[dof].motor_init()

    def publish_all_motors(self, event=None):
        # for joint in self.joints:
        #     joint.send_position_and_update_status(joint.des_angle)  # TODO Demo
        for dof in self.dofs:
            self.joints[dof].send_position_and_update_status(self.joints[dof].des_angle)
            rospy.sleep(0.0001)
