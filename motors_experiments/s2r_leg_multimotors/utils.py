import time
from dataclasses import dataclass

maxRawPosition = 2 ** 16 - 1  # 16-Bits for Raw Position Values
maxRawVelocity = 2 ** 12 - 1  # 12-Bits for Raw Velocity Values

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
