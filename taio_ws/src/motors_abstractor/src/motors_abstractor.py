import rospy, json, time
# import .utilities.utils

from utilities import utils
from sensor_msgs.msg import JointState
import can_motor_controller as mot_con
from std_msgs.msg import String, Header

import logging
logger = logging.getLogger(__name__)

class MotorsAbstractor:

    def __init__(self, parsed_joint_params, new_angle=0):
        self.joint_name = parsed_joint_params['name']
        # self.motor_name = parsed_joint_params['name']
        self.can_params = utils.CanParams(parsed_joint_params['can_id'],parsed_joint_params['can_socket'])
        self.motor_desire_command = utils.MotorCommand(new_angle,new_angle,new_angle,new_angle,new_angle) #TODO
        self.motor_status = utils.MotorStatus(new_angle,new_angle,new_angle,new_angle,self.can_params.can_id) #TODO
        self.motor_type = parsed_joint_params['motor_type']
        self.motor_params = utils.MotorParams(self.motor_type).get_motor_params()
        self.motor_controller = mot_con.CanMotorController(self.can_params, self.motor_params)
        '''
        ROS entities
        '''
        self.ros_params = utils.RosParams("/watchdog/" + self.joint_name, "/motors/" + self.joint_name + "/status", "/motors/" + self.joint_name + "/error")
        self.publisher = rospy.Publisher(self.ros_params.publishing_as, String, queue_size=10)
        self.error_publisher = rospy.Publisher(self.ros_params.publishing_err, String, queue_size=10)
        self.subscriber = rospy.Subscriber(self.ros_params.subscribing_to, String, self.update_desired_command)

        logger.info(f"New Motor: {self.joint_name} Subs to: {self.ros_params.subscribing_to} and Pubs to: {self.ros_params.publishing_as}")
        logger.info("Created new Motor for: {} CAN ID: {} Subs to: {} and Pubs to: {}".format(self.joint_name, self.can_params.can_id,
                                                                                        self.ros_params.subscribing_to,
                                                                                        self.ros_params.publishing_as))

    def send_position_and_update_status(self, desired_position, SPEED_VALUE =0, KP_VALUE=200, KD_VALUE=0.1, TORQUE_VALUE=0):
        '''
        Send desired position over can
        and update the received status
        '''
        new_status = self.motor_controller.send_rad_command(self.motor_desire_command.des_pos, self.motor_desire_command.des_vel,
                                                            self.motor_desire_command.des_kp, self.motor_desire_command.kd,
                                                              self.motor_desire_command.des_torque)
        if new_status == None:
            self.error_publisher.publish("TIMEOUT EXCEPTION: Got NONE message from motor")
        else:
            self.motor_status.update_from_motor(new_status)

            # motor_id, pos, vel, curr = motor_status
            # TODO check if ID is correct
            global msg_id
            msg_id += 1
            new_msg = {'message_id': msg_id, 'name': self.joint_name, 'time_stamp': time.time(),
                       'position': self.motor_status.act_pos, 'velocity': self.motor_status.act_vel, 'torque': self.motor_status.act_torque}
            new_msg_json = json.dumps(new_msg)

            self.publisher.publish(new_msg_json)

            return 1

    def motor_init(self):
        status = self.motor_controller.enable_motor()
        if status == None:
            logger.warning(f"Motor Init Failed for: {self.joint_name}")
            return 0
        else:
            self.motor_status.update_from_motor(status)
            return 1

    def motor_stop(self):
        status = self.motor_controller.disable_motor()
        if status == None:
            logger.warning(f"Motor Disable Failed for: {self.joint_name}")
            return 0
        else:
            self.motor_status.update_from_motor(status)
            return 1

    def motor_set_zero(self):
        status = self.motor_controller.set_zero_position()
        if status == None:
            logger.warning(f"Motor Zero Failed for: {self.joint_name}")

            return 0
        else:
            self.motor_status.update_from_motor(status)
            return 1

    def publish_status(self, status):
        self.publisher.publish(status)

    def update_desired_angle(self, msg):
        command = json.loads(msg.data)
        self.motor_desire_command.des_pos = command['position']
        self.motor_desire_command.des_vel = command['velocity']
        self.motor_desire_command.des_torque = command['torque']
        self.motor_desire_command.des_kp = command['kp']
        self.motor_desire_command.des_kd = command['kd']

    def update_from_motor(self,status):
        self.mot_id, self.act_pos, self.act_vel ,self.act_torque = status