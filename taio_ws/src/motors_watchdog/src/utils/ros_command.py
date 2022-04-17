import genpy, ast

Kp_VAL = 200
Kd_VAL = 0.1
VEL_VAL = 0
TOR_VAL = 0


class RosCommand:
    def __init__(self, joint_name, position, velocity = VEL_VAL, torque=TOR_VAL, kp = Kp_VAL, kd = Kd_VAL):
        self.position = position
        self.velocity = velocity
        self.torque = torque
        self.joint_name = joint_name
        self.kp = kp
        self.kd = kd
        self.time_stamp = genpy.Time()

    def ros_command_str(self):
        new_msg = {'header': {'name':self.joint_name,'time_stamp': self.time_stamp}, "command":{'position':self.position, 'velocity':self.velocity, 'torque':self.torque, 'kp': self.kp, 'kd': self.kd}}
        return str(new_msg)

    def ros_command_dict(self, string_cmd):
        return ast.literal_eval(string_cmd)
