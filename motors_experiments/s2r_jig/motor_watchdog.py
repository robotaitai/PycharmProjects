import math

from taio_ws.src.motors_abstraction.src import canMotorController as mot_con

VERBOSE = True

if VERBOSE:
    def verbose_print(args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        print(args)
else:
    verbose_print = lambda *a: None      # do-nothing function


class MotorWatchdog:
    def __init__(self, motor_params):
        self.motor_name = motor_params['name']
        self.min_position = motor_params['min_position']
        self.max_position = motor_params['max_position']
        self.max_velocity = motor_params['max_velocity']
        self.max_effort = motor_params['max_effort']
        self.can_id = motor_params['can_id']
        self.motor_type = motor_params['motor_type']
        self.motor = mot_con.CanMotorController('can0', self.can_id, self.motor_type)
        self.act_pos = 0
        self.des_pos = 0
        self.act_vel = 0
        self.act_torque = 0
        self.deceleration = 60
        self.overshooting = 0
        self.danger_velocity_zone = 0
        self.max_velocity_allowed = 0
        self.delta_from_ss = 0.0




    def send_safe_rad_command(self, desired_position, SPEED_VALUE =0, KP_VALUE=100, KD_VALUE=0.1, TORQUE_VALUE=0):

        safe_desired_position, safe_desired_speed, safe_desired_torque = self.check_desired_range(desired_position, SPEED_VALUE, TORQUE_VALUE)
        motor_status = self.motor.send_rad_command(safe_desired_position, safe_desired_speed, KP_VALUE, KD_VALUE, safe_desired_torque)

        if motor_status == None:
            print("TIMEOUT EXCEPTION: Got NONE message from motor")

        else:
            motor_id, pos, vel, curr = motor_status
            pos, vel, curr = self.check_actual_range(pos, vel, curr)
            self.act_pos = pos
            self.act_vel = vel
            self.act_torque = curr
            self.update_delta_pos_from_hard_stop()

        return motor_status

    def enable_motor(self):
        self.motor.enable_motor()

    def disable_motor(self):
        self.motor.disable_motor()


    def check_desired_range(self, desired_position, desired_speed,  desired_torque):

        if desired_position < self.min_position:
            verbose_print(
                "CAN Watchdog Alert for: {} Desired Pos: {} is Lower that the min allowed: {}".format(self.motor_name,
                                                                                                      desired_position,
                                                                                                      self.min_position))
            desired_position = self.min_position

        if desired_position > self.max_position:
            verbose_print("CAN Watchdog Alert for: {}  Desired Pos: {} is Higher that the max allowed: {}".format(self.motor_name,
                                                                                                        desired_position,
                                                                                                        self.max_position))
            desired_position = self.max_position

        if abs(desired_speed) > self.max_velocity:
            verbose_print("CAN Watchdog Alert for: {}  Desired Velocity: {} is Higher that the max allowed: {}".format(
                self.motor_name, desired_speed, self.max_velocity))
            desired_speed = self.max_velocity

        if desired_torque > self.max_effort:
            verbose_print("CAN Watchdog Alert for: {}  Desired Effort: {} is Higher that the max allowed: {}".format(
                self.motor_name, desired_torque, self.max_effort))
            desired_torque = self.max_effort

        return desired_position, desired_speed, desired_torque


    def check_actual_range(self, actual_position, actual_speed,  actual_torque):

        self.overshooting = 0
        self.danger_velocity_zone = 0

        max_vel = math.sqrt(self.delta_from_ss*2*self.deceleration)
        self.max_velocity_allowed = max_vel
        # print(f'Velocity is: {actual_speed} allowed speed: {max_vel}')
        if abs(actual_speed) > max_vel:
            self.danger_velocity_zone = 1
            print(f"DANGER SPEED ZONE: max_vel: {max_vel}, actual vel: {actual_speed} and the distance: {self.delta_from_ss}")

        if actual_position < self.min_position:
            self.overshooting = 1
            print("OVERSHOOT:Actual Pos: {} is Lower that the min allowed: {} with Velocity of: {}".format(actual_position,
                                                                                                      self.min_position,
                                                                                                         actual_speed))

        if actual_position > self.max_position:
            self.overshooting = 1
            print("OVERSHOOT:Actual Pos: {} is Higher that the min allowed: {} with Velocity of: {}".format(actual_position,
                                                                                                                   self.min_position,
                                                                                                                   actual_speed))
        if actual_speed > self.max_velocity:
            verbose_print("OVERSHOOT: Actual Velocity: {} is Higher that the max allowed: {}".format(actual_speed, self.max_velocity))

        if actual_torque > self.max_effort:
            verbose_print("OVERSHOOT: Actual Effort: {} is Higher that the max allowed: {}".format(actual_torque, self.max_effort))

        return actual_position, actual_speed, actual_torque

    def update_delta_pos_from_hard_stop(self):
        self.delta_from_ss = min([abs(self.act_pos-self.min_position) ,abs(self.act_pos-self.max_position)])
        # print(f'min delta from edge: {self.delta_from_ss}')
        # if self.act_vel > 0:
        #     self.delta_from_ss = self.max_position - self.act_pos
        # else:
        #     self.delta_from_ss = self.act_pos - self.min_position

    def is_overshooting(self):
        return self.overshooting