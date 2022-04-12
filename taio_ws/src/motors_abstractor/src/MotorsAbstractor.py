import rospy
# import .utilities.utils

from utilities import utils
from sensor_msgs.msg import JointState
import CanMotorController as mot_con
from std_msgs.msg import String, Header


class MotorsAbstractor:

    def __init__(self, parsed_joint_params, joint, new_angle=0):
        self.joint_name = joint
        # self.motor_name = parsed_joint_params['name']
        self.can_params = utils.CanParams(parsed_joint_params[joint]['can_id'],parsed_joint_params[joint]['can_socket'])
        self.motor_status = utils.MotorStatus(new_angle,new_angle,new_angle,new_angle,self.can_params.can_id) #TODO
        self.motor_type = parsed_joint_params[joint]['motor_type']

        self.motor_controller = mot_con.CanMotorController(self.can_params.can_socket, self.can_params.can_id, self.motor_type)
        self.ros_params = utils.RosParams("/happiness/" + joint + "/limited", "/motors/" + joint + "/status", "/motors/" + joint + "/error")

        self.publisher = rospy.Publisher(self.ros_params.publishing_as, JointState, queue_size=10)
        self.error_publisher = rospy.Publisher(self.ros_params.publishing_err, String, queue_size=10)
        self.subscriber = rospy.Subscriber(self.ros_params.subscribing_to, JointState, self.update_desired_angle)
        print(f"New Motor: {self.joint_name} Subs to: {self.ros_params.subscribing_to} and Pubs to: {self.ros_params.publishing_as}")
        self.des_angle = new_angle
        print("Created new Motor for: {} CAN ID: {} Subs to: {} and Pubs to: {}".format(self.joint_name, self.can_params.can_id,
                                                                                        self.ros_params.subscribing_to,
                                                                                        self.ros_params.publishing_as))

    def send_position_and_update_status(self, desired_position, SPEED_VALUE =0, KP_VALUE=200, KD_VALUE=0.1, TORQUE_VALUE=0):
        '''
        Send desired position over can
        and update the received status
        '''
        new_status = self.motor_controller.send_rad_command(desired_position, SPEED_VALUE, KP_VALUE, KD_VALUE,
                                                              TORQUE_VALUE)
        if new_status == None:
            self.error_publisher.publish("TIMEOUT EXCEPTION: Got NONE message from motor")
        else:
            self.motor_status.update_from_motor(new_status)

            # motor_id, pos, vel, curr = motor_status
            # TODO check if ID is correct
            motor_status = JointState()
            motor_status.header = Header()
            motor_status.header.stamp = rospy.Time.now()
            motor_status.name = self.joint_name + '/status'
            motor_status.position.append(self.motor_status.act_pos)
            motor_status.velocity.append(self.motor_status.act_vel)
            motor_status.effort.append(self.motor_status.act_torque)
            self.publisher.publish(motor_status)
            return 1

    def motor_init(self):
        status = self.motor_controller.enable_motor()
        if status == None:
            print(f"Motor Init Failed for: {self.joint_name}")
            return 0
        else:
            self.motor_status.update_from_motor(status)
            return 1

    def motor_stop(self):
        status = self.motor_controller.disable_motor()
        if status == None:
            print(f"Motor Disable Failed for: {self.joint_name}")
            return 0
        else:
            self.motor_status.update_from_motor(status)
            return 1

    def motor_set_zero(self):
        status = self.motor_controller.set_zero_position()
        if status == None:
            print(f"Motor Zero Failed for: {self.joint_name}")

            return 0
        else:
            self.motor_status.update_from_motor(status)
            return 1

    def publish_status(self, status):
        self.publisher.publish(status)

    def update_desired_angle(self, msg):
        self.des_angle = msg.position[0]

    def update_from_motor(self,status):
        self.mot_id, self.act_pos, self.act_vel ,self.act_torque = status