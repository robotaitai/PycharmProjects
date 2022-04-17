import math, logging
from dataclasses import dataclass
import rospy, json, time
from std_msgs.msg import String

global msg_id
msg_id = 0

logger = logging.getLogger(__name__)


@dataclass
class MotorLimits:
    min_pos: float
    max_pos: float
    max_vel: float
    max_torque: float

@dataclass
class RosParams:
    subscribing_to: str
    publishing_as: str
    publishing_err: str


class MotorWatchdog:
    def __init__(self, motor_params):
        self.joint_name = motor_params['name']
        self.motor_limits = MotorLimits(motor_params['min_position'], motor_params['max_position'],motor_params['max_velocity'],motor_params['max_effort'])
        '''
        ROS entities
        '''
        self.ros_params = RosParams("/happiness/" + self.joint_name, "/watchdog/" + self.joint_name, "/watchdog/" + self.joint_name + "/error")
        self.publisher = rospy.Publisher(self.ros_params.publishing_as, String, queue_size=10)
        self.error_publisher = rospy.Publisher(self.ros_params.publishing_err, String, queue_size=10)
        self.subscriber = rospy.Subscriber(self.ros_params.subscribing_to, String, self.check_desired_range)



    def check_desired_range(self, msg):
        command = json.loads(msg.data)
        desired_position = command['position']
        desired_speed = command['velocity']
        desired_torque = command['torque']

        if desired_position < self.motor_limits.min_pos:
            logger.warning(
                "CAN Watchdog Alert for: {} Desired Pos: {} is Lower that the min allowed: {}".format(self.joint_name,
                                                                                                      desired_position,
                                                                                                      self.motor_limits.min_pos))
            desired_position = self.motor_limits.min_pos

        if desired_position > self.motor_limits.max_pos:
            logger.warning("CAN Watchdog Alert for: {}  Desired Pos: {} is Higher that the max allowed: {}".format(self.joint_name,
                                                                                                                   desired_position,
                                                                                                                   self.motor_limits.max_pos))
            desired_position = self.motor_limits.max_pos

        if abs(desired_speed) > self.motor_limits.max_vel:
            logger.warning("CAN Watchdog Alert for: {}  Desired Velocity: {} is Higher that the max allowed: {}".format(
                self.joint_name, desired_speed, self.motor_limits.max_vel))
            desired_speed = self.motor_limits.max_vel

        if desired_torque > self.motor_limits.max_torque:
            logger.warning("CAN Watchdog Alert for: {}  Desired Effort: {} is Higher that the max allowed: {}".format(
                self.joint_name, desired_torque, self.motor_limits.max_torque))
            desired_torque = self.motor_limits.max_torque
        #update data TODO why?
        self.des_pos_limited, self.des_vel_limited, self.des_tor_limited = desired_position, desired_speed, desired_torque

        """
        Publishing
        """
        global msg_id
        msg_id += 1
        new_msg = {'message_id': msg_id ,'name': self.joint_name, 'time_stamp': time.time(),
                                      'position':desired_position, 'velocity': desired_speed, 'torque': desired_torque,
                                                  'kp': command['kp'], 'kd':  command['kd']}
        new_msg_json = json.dumps(new_msg)

        self.publisher.publish(new_msg_json)


    def check_actual_range(self):
        actual_position = self.motor_status.act_pos
        actual_speed = self.motor_status.act_vel
        actual_torque = self.motor_status.act_torque

        self.overshooting = 0
        self.danger_velocity_zone = 0

        max_vel = math.sqrt(self.delta_from_ss*2*self.deceleration)
        self.max_velocity_allowed = max_vel
        # print(f'Velocity is: {actual_speed} allowed speed: {max_vel}')
        if abs(actual_speed) > max_vel:
            self.danger_velocity_zone = 1
            print(f"DANGER SPEED ZONE: max_vel: {max_vel}, actual vel: {actual_speed} and the distance: {self.delta_from_ss}")

        if actual_position < self.motor_limits.min_pos:
            self.overshooting = 1
            print("OVERSHOOT:Actual Pos: {} is Lower that the min allowed: {} with Velocity of: {}".format(actual_position,
                                                                                                      self.motor_limits.min_pos,
                                                                                                         actual_speed))

        if actual_position > self.motor_limits.max_pos:
            self.overshooting = 1
            logger.warning("OVERSHOOT:Actual Pos: {} is Higher that the max allowed: {} with Velocity of: {}".format(actual_position,
                                                                                                                   self.motor_limits.max_pos,
                                                                                                                   actual_speed))
        if actual_speed > self.motor_limits.max_vel:
            logger.warning("OVERSHOOT: Actual Velocity: {} is Higher that the max allowed: {}".format(actual_speed, self.motor_limits.max_vel))

        if actual_torque > self.motor_limits.max_torque:
            logger.warning("OVERSHOOT: Actual Effort: {} is Higher that the max allowed: {}".format(actual_torque, self.motor_limits.max_torque))


    def update_delta_pos_from_hard_stop(self):
        self.delta_from_ss = min([abs(self.motor_status.act_pos-self.motor_limits.min_pos) ,abs(self.motor_status.act_pos-self.motor_limits.max_pos)])

    def is_overshooting(self):
        return self.overshooting