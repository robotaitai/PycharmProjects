import time
from dataclasses import dataclass
from . import motors_params
import logging
logger = logging.getLogger(__name__)


maxRawPosition = 2 ** 16 - 1  # 16-Bits for Raw Position Values
maxRawVelocity = 2 ** 12 - 1  # 12-Bits for Raw Velocity Values
maxRawTorque = 2 ** 12 - 1  # 12-Bits for Raw Torque Values
maxRawKp = 2 ** 12 - 1  # 12-Bits for Raw Kp Values
maxRawKd = 2 ** 12 - 1  # 12-Bits for Raw Kd Values
maxRawCurrent = 2 ** 12 - 1  # 12-Bits for Raw Current Values


def float_to_uint(x, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    # Attempt to speedup by using pre-computation. Not used currently.
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2 ** numBits - 1
    return int(((x - offset) * (bitRange)) / span)


def uint_to_float(x_int, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2 ** numBits - 1
    return ((x_int * span) / (bitRange)) + offset


def waitOhneSleep(dt):
    startTime = time.time()
    while time.time() - startTime < dt:
        pass


@dataclass
class CanParams:
    can_id: hex
    can_socket: str

@dataclass
class MotorLimits:
    min_pos : float
    max_pos: float
    max_vel: float
    max_torque: float

@dataclass
class MotorCommand:
    des_pos: float
    des_vel: float
    des_torque: float
    des_kp: float
    des_kd: float


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
        correct_id =0 #TODO check correct Id
        correct_id, self.act_pos, self.act_vel ,self.act_torque = status


@dataclass
class RosParams:
    subscribing_to: str
    publishing_as: str
    publishing_err: str
    # publisher : rospy.Publisher(publishing_as, JointState, queue_size=10)
    # error_publisher : rospy.Publisher(publishing_err, String, queue_size=10)

class MotorParams:
    def __init__(self,motor_type):
        legitinate_motors = motors_params.legitimate_motors
        if motor_type not in legitinate_motors:
            logger.warning("Couldn't find any params for this motor")
            self.motor_params = None
        if motor_type == 'AK80_6_V1':
            self.motor_params = motors_params.AK80_6_V1_PARAMS
        elif motor_type == 'AK80_6_V1p1':
            self.motor_params = motors_params.AK80_6_V1p1_PARAMS
        elif motor_type == 'AK80_6_V2':
            self.motor_params = motors_params.AK80_6_V2_PARAMS
        elif motor_type == 'AK80_9_V1p1':
            self.motor_params = motors_params.AK80_9_V1p1_PARAMS
        elif motor_type == 'AK80_9_V2':
            self.motor_params = motors_params.AK80_9_V2_PARAMS
        elif motor_type == 'AK80_64_V2':
            self.motor_params = motors_params.AK80_64_V2_PARAMS
        elif motor_type == 'AK80_64_V3':
            self.motor_params = motors_params.AK80_64_V3_PARAMS
        elif motor_type == 'AK70_10':
            self.motor_params = motors_params.AK70_10_PARAMS
        logger.info(f"{motor_type} motor was configured")

    def get_motor_params(self):
        return self.motor_params