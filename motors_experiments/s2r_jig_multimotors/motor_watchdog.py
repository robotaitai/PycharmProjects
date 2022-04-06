import math
from dataclasses import dataclass
from taio_ws.src.motors_abstraction.src import canMotorController as mot_con

VERBOSE = True
if VERBOSE:
    def verbose_print(args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        print(args)
else:
    verbose_print = lambda *a: None      # do-nothing function


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
    #TODO check id

    def update_status(self, new_pos, new_vel, new_torque):
        self.act_pos = new_pos
        self.act_vel = new_vel
        self.act_torque = new_torque


    def update_from_motor(self,status):
        self.mot_id, self.act_pos, self.act_vel ,self.act_torque = status


class MotorWatchdog:
    def __init__(self, motor_params):
        self.motor_name = motor_params['name']
        #constraints
        self.motor_limits = MotorLimits(motor_params['min_position'],motor_params['max_position'],motor_params['max_velocity'],motor_params['max_effort'])
        # self.min_position = joint_params['min_position']
        # self.max_position = joint_params['max_position']
        # self.max_velocity = joint_params['max_velocity']
        # self.max_effort = joint_params['max_effort']

        #canparams
        self.can_params = CanParams(motor_params['can_id'],motor_params['can_socket'])
        # self.can_id = joint_params['can_id']
        # self.can_socket = joint_params['can_socket']
        #status
        self.motor_status = MotorStatus(0,0,0,0,self.can_params.can_id)
        # self.act_pos = 0
        # self.des_pos = 0
        # self.act_vel = 0
        # self.act_torque = 0
        #motor params
        self.motor_type = motor_params['motor_type']
        self.motor = mot_con.CanMotorController(self.can_params.can_socket, self.can_params.can_id, self.motor_type)

        self.deceleration = 60
        self.overshooting = 0
        self.danger_velocity_zone = 0
        self.max_velocity_allowed = 0
        self.delta_from_ss = 0.0


    def send_safe_rad_command(self, desired_position, SPEED_VALUE =0, KP_VALUE=100, KD_VALUE=0.1, TORQUE_VALUE=0):
        #watchdog
        safe_desired_position, safe_desired_speed, safe_desired_torque = self.check_desired_range(desired_position, SPEED_VALUE, TORQUE_VALUE)

        #send clipped parameters
        new_status = self.motor.send_rad_command(safe_desired_position, safe_desired_speed, KP_VALUE, KD_VALUE, safe_desired_torque)

        if new_status == None:
            print("TIMEOUT EXCEPTION: Got NONE message from motor")

        else:
            # motor_id, pos, vel, curr = motor_status
            #update the new status from the received data
            self.motor_status.update_from_motor(new_status)

            #update the distances from hard (sof) stops
            self.update_delta_pos_from_hard_stop() #TODO do we want to do anything?

            #Check if the range is in the proper limits
            self.check_actual_range()


    def enable_motor(self):
        self.motor.enable_motor()


    def disable_motor(self):
        self.motor.disable_motor()

    def set_zero_position(self):
        self.motor.set_zero_position()


    def check_desired_range(self, desired_position, desired_speed,  desired_torque):

        if desired_position < self.motor_limits.min_pos:
            verbose_print(
                "CAN Watchdog Alert for: {} Desired Pos: {} is Lower that the min allowed: {}".format(self.motor_name,
                                                                                                      desired_position,
                                                                                                      self.motor_limits.min_pos))
            desired_position = self.motor_limits.min_pos

        if desired_position > self.motor_limits.max_pos:
            verbose_print("CAN Watchdog Alert for: {}  Desired Pos: {} is Higher that the max allowed: {}".format(self.motor_name,
                                                                                                        desired_position,
                                                                                                        self.motor_limits.max_pos))
            desired_position = self.motor_limits.max_pos

        if abs(desired_speed) > self.motor_limits.max_vel:
            verbose_print("CAN Watchdog Alert for: {}  Desired Velocity: {} is Higher that the max allowed: {}".format(
                self.motor_name, desired_speed, self.motor_limits.max_vel))
            desired_speed = self.motor_limits.max_vel

        if desired_torque > self.motor_limits.max_torque:
            verbose_print("CAN Watchdog Alert for: {}  Desired Effort: {} is Higher that the max allowed: {}".format(
                self.motor_name, desired_torque, self.motor_limits.max_torque))
            desired_torque = self.motor_limits.max_torque

        return desired_position, desired_speed, desired_torque


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
            print("OVERSHOOT:Actual Pos: {} is Higher that the max allowed: {} with Velocity of: {}".format(actual_position,
                                                                                                                   self.motor_limits.max_pos,
                                                                                                                   actual_speed))
        if actual_speed > self.motor_limits.max_vel:
            verbose_print("OVERSHOOT: Actual Velocity: {} is Higher that the max allowed: {}".format(actual_speed, self.motor_limits.max_vel))

        if actual_torque > self.motor_limits.max_torque:
            verbose_print("OVERSHOOT: Actual Effort: {} is Higher that the max allowed: {}".format(actual_torque, self.motor_limits.max_torque))

    def update_delta_pos_from_hard_stop(self):
        self.delta_from_ss = min([abs(self.motor_status.act_pos-self.motor_limits.min_pos) ,abs(self.motor_status.act_pos-self.motor_limits.max_pos)])
        # print(f'min delta from edge: {self.delta_from_ss}')
        # if self.act_vel > 0:
        #     self.delta_from_ss = self.max_position - self.act_pos
        # else:
        #     self.delta_from_ss = self.act_pos - self.min_position

    def is_overshooting(self):
        return self.overshooting