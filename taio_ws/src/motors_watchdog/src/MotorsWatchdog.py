import math

VERBOSE = True
if VERBOSE:
    def verbose_print(args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        print(args)
else:
    verbose_print = lambda *a: None      # do-nothing function


class MotorWatchdog:

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

    def is_overshooting(self):
        return self.overshooting