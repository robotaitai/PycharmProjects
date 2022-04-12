import can
# from src.motors_abstraction.src import utils
import utils
import MotorsParams
import math
from bitstring import BitArray


maxRawTorque = 2 ** 12 - 1  # 12-Bits for Raw Torque Values
maxRawKp = 2 ** 12 - 1  # 12-Bits for Raw Kp Values
maxRawKd = 2 ** 12 - 1  # 12-Bits for Raw Kd Values
maxRawCurrent = 2 ** 12 - 1  # 12-Bits for Raw Current Values
dt_sleep = 0.0001  # Time before motor sends a reply
msg_time_out = 0.01

VERBOSE = False
# SOCKET_ID = 'can5'
if VERBOSE:
    def verbose_print(args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        print(args)
else:
    verbose_print = lambda *a: None      # do-nothing function


class CanMotorController():
    """
    Class for creating a Mini-Cheetah Motor Controller over CAN. Uses SocketCAN driver for
    communication.
    """

    # Declare Socket as Class Attribute instead of member attribute so that it can be used across
    # multiple instances and check if the socket was declared earlier by an instance.

    can_socket_declared = False
    motor_socket = None
    can_sockets = {}

    def __init__(self, can_socket, motor_id, motor_type, socket_timeout=0.05):
        """
        Instantiate the class with socket name, motor ID, and socket timeout.
        Sets up the socket communication for rest of the functions.
        """
        self.motorParams = MotorsParams.AK80_64_V2_PARAMS  # default choice
        print('Using Motor Type: {}'.format(motor_type))
        assert motor_type in MotorsParams.legitimate_motors, 'Motor Type not in list of accepted motors.'
        if motor_type == 'AK80_6_V1':
            self.motorParams = MotorsParams.AK80_6_V1_PARAMS
        elif motor_type == 'AK80_6_V1p1':
            self.motorParams = MotorsParams.AK80_6_V1p1_PARAMS
        elif motor_type == 'AK80_6_V2':
            self.motorParams = MotorsParams.AK80_6_V2_PARAMS
        elif motor_type == 'AK80_9_V1p1':
            self.motorParams = MotorsParams.AK80_9_V1p1_PARAMS
        elif motor_type == 'AK80_9_V2':
            self.motorParams = MotorsParams.AK80_9_V2_PARAMS
        elif motor_type == 'AK80_64_V2':
            self.motorParams = MotorsParams.AK80_64_V2_PARAMS
        elif motor_type == 'AK80_64_V3':
            self.motorParams = MotorsParams.AK80_64_V3_PARAMS
        elif motor_type == 'AK70_10':
            self.motorParams = MotorsParams.AK70_10_PARAMS
        # print(self.motorParams)
        # can_socket = (can_socket,)
        self.motor_id = motor_id
        self.can_socket = can_socket
        # create a raw socket and bind it to the given CAN interface
        # if not CanMotorController.can_socket_declared:
        if self.can_socket not in CanMotorController.can_sockets:
            try:
                # CanMotorController.motor_socket =  can.interface.Bus(channel=self.can_socket, bustype='socketcan_native')
                CanMotorController.can_sockets[self.can_socket] =  can.interface.Bus(channel=self.can_socket, bustype='socketcan_native')

                # CanMotorController.motor_socket.flush_tx_buffer()

                # CanMotorController.motor_socket.bind(can_socket)
                # CanMotorController.motor_socket.settimeout(socket_timeout)
                print("Bound to: ", can_socket)
                CanMotorController.can_socket_declared = True
            except Exception as e:
                print("Unable to Connect to Socket Specified: ", can_socket)
                print("Error:", e)
        elif CanMotorController.can_socket_declared:
            pass
            # print("CAN Socket Already Available. Using: ", CanMotorController.motor_socket)

        # Initialize the command BitArrays for performance optimization
        self._p_des_BitArray = BitArray(uint=utils.float_to_uint(0, self.motorParams['P_MIN'],
                                                                 self.motorParams['P_MAX'], 16), length=16)
        self._v_des_BitArray = BitArray(uint=utils.float_to_uint(0, self.motorParams['V_MIN'],
                                                                 self.motorParams['V_MAX'], 12), length=12)
        self._kp_BitArray = BitArray(uint=0, length=12)
        self._kd_BitArray = BitArray(uint=0, length=12)
        self._tau_BitArray = BitArray(uint=0, length=12)
        self._cmd_bytes = BitArray(uint=0, length=64)
        self._recv_bytes = BitArray(uint=0, length=48)

    def close_bus(self):
        # print(CanMotorController.motor_socket.property)
        # CanMotorController.motor_socket.flush_tx_buffer()
        # print(CanMotorController.motor_socket.property)
        CanMotorController.can_sockets[self.can_socket].shutdown()
        # print(CanMotorController.motor_socket.property)


    def _send_can_frame(self, data):
        """
        Send raw CAN data frame (in bytes) to the motor.
        """
        # can_dlc = len(data)
        # can_msg = struct.pack(can_frame_fmt_send, self.motor_id, can_dlc, data)
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)

        try:
            # CanMotorController.motor_socket.send(can_msg)
            # CanMotorController.motor_socket.send(msg,timeout=msg_time_out)
            CanMotorController.can_sockets[self.can_socket].send(msg,timeout=msg_time_out)
            verbose_print(msg)
        except Exception as e:
            print("Unable to Send CAN Frame.")
            print("Error: ", e)

    def _recv_can_frame(self,timeout=msg_time_out):
        """
        Recieve a CAN frame and unpack it. Returns can_id, can_dlc (data length), data (in bytes)
        """
        try:
            # message = CanMotorController.motor_socket.recv(timeout=timeout)  # Wait until a message is received.
            message = CanMotorController.can_sockets[self.can_socket].recv(timeout=timeout)  # Wait until a message is received.
            if message == None:
                verbose_print("\r\n No message received, pass..")
                return message
            verbose_print(message)
            return [message.arbitration_id, message.dlc, message.data]
            # frame, addr = CanMotorController.motor_socket.recvfrom(14)
            # can_id, can_dlc, data = struct.unpack(can_frame_fmt_recv, frame)
            # return can_id, can_dlc, data[:can_dlc]
        except Exception as e:
            print("Unable to Receive CAN Franme.")
            print("Error: ", e)

    def enable_motor(self):
        """
        Sends the enable motor command to the motor.
        """
        try:
            self._send_can_frame(b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC')
            utils.waitOhneSleep(dt_sleep)
            msg = self._recv_can_frame()
            if msg == None:
                print("Nothing to parse")
                return msg
            else:
                motorStatusData = msg[2]
                rawMotorData = self.decode_motor_status(motorStatusData)
                motor_id, pos, vel, curr = self.convert_raw_to_physical_rad(rawMotorData[0], rawMotorData[1], rawMotorData[2], rawMotorData[3])
                print("Motor {} Enabled.".format(int(motor_id)))
                return motor_id, pos, vel, curr
        except Exception as e:
            print('\r\n `Error Enabling Motor.')
            print("Error: ", e)
            # return '-99','-99','-99'

    def disable_motor(self):
        """
        Sends the disable motor command to the motor.
        """
        try:
            self._send_can_frame(b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD')
            utils.waitOhneSleep(dt_sleep)
            msg = self._recv_can_frame()
            if msg == None:
                print("Nothing to parse")
                return msg
            else:
                motorStatusData = msg[2]
                rawMotorData = self.decode_motor_status(motorStatusData)
                motor_status = self.convert_raw_to_physical_rad(rawMotorData[0], rawMotorData[1], rawMotorData[2], rawMotorData[3])
                motor_id, pos, vel, curr = motor_status[0], motor_status[1], motor_status[2], motor_status[3]
                print("Motor Disabled.")
                return motor_id, pos, vel, curr
        except Exception as e:
            print('Error Disabling Motor!')
            print("Error: ", e)

    def set_zero_position(self):
        """
        Sends command to set current position as Zero position.
        """
        try:
            self._send_can_frame(b'\x7f\xff\x7f\xf0\x00\x00\x07\xff')
            utils.waitOhneSleep(0.001)

            self._send_can_frame(b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE')
            utils.waitOhneSleep(dt_sleep)
            msg = self._recv_can_frame()
            if msg == None:
                print("Nothing to parse")
                return msg
            else:
                motorStatusData = msg[2]
                rawMotorData = self.decode_motor_status(motorStatusData)
                motor_status = self.convert_raw_to_physical_rad(rawMotorData[0], rawMotorData[1], rawMotorData[2], rawMotorData[3])
                motor_id, pos, vel, curr = motor_status[0], motor_status[1], motor_status[2], motor_status[3]
                print("\r\nZero Position set.")
                return motor_id, pos, vel, curr
        except Exception as e:
            print('Error Setting Zero Position!')
            print("Error: ", e)

    def decode_motor_status(self, data_frame):
        '''
        Function to decode the motor status reply message into its constituent raw values.

        /// CAN Reply Packet Structure ///
        /// 16 bit position, between -4*pi and 4*pi
        /// 12 bit velocity, between -30 and + 30 rad/s
        /// 12 bit current, between -40 and 40;
        /// CAN Packet is 5 8-bit words
        /// Formatted as follows.  For each quantity, bit 0 is LSB
        /// 0: [position[15-8]]
        /// 1: [position[7-0]]
        /// 2: [velocity[11-4]]
        /// 3: [velocity[3-0], current[11-8]]
        /// 4: [current[7-0]]

        returns: the following raw values as (u)int: position, velocity, current
        '''

        # Convert the message from motor to a bit string as this is easier to deal with than hex
        # while seperating individual values.
        self._recv_bytes.bytes = data_frame
        dataBitArray = self._recv_bytes.bin

        # Separate motor satus values from the bit string.
        # Motor ID not considered necessary at the moment.
        motor_id = dataBitArray[:8]
        positionBitArray = dataBitArray[8:24]
        # print("\r\n pos from can: {}".format(positionBitArray))

        velocityBitArray = dataBitArray[24:36]
        currentBitArray = dataBitArray[36:48]

        # motor_id = int(motor_id, 2)
        positionRawValue = int(positionBitArray, 2)
        # print("\r\n pos int can: {}".format(positionRawValue))

        velocityRawValue = int(velocityBitArray, 2)
        currentRawValue = int(currentBitArray, 2)

        # TODO: Is it necessary/better to return motor_id?
        return motor_id, positionRawValue, velocityRawValue, currentRawValue
        # return positionRawValue, velocityRawValue, currentRawValue

    def convert_raw_to_physical_rad(self, motor_id, positionRawValue, velocityRawValue, currentRawValue):
        '''
        Function to convert the raw values from the motor to physical values:

        /// CAN Reply Packet Structure ///
        /// 16 bit position, between -4*pi and 4*pi
        /// 12 bit velocity, between -30 and + 30 rad/s
        /// 12 bit current, between -40 and 40;
        /// CAN Packet is 5 8-bit words
        /// Formatted as follows.  For each quantity, bit 0 is LSB
        /// 0: [position[15-8]]
        /// 1: [position[7-0]]
        /// 2: [velocity[11-4]]
        /// 3: [velocity[3-0], current[11-8]]
        /// 4: [current[7-0]]

        returns: position (radians), velocity (rad/s), current (amps)
        '''
        verbose_print("\r\npos raw val act: {} ".format(positionRawValue))

        physicalPositionRad = utils.uint_to_float(positionRawValue, self.motorParams['P_MIN'],
                                                  self.motorParams['P_MAX'], 16)
        physicalVelocityRad = utils.uint_to_float(velocityRawValue, self.motorParams['V_MIN'],
                                                  self.motorParams['V_MAX'], 12)
        physicalCurrent = utils.uint_to_float(currentRawValue, self.motorParams['T_MIN'],
                                              self.motorParams['T_MAX'], 12)

        # Correct Axis Direction
        physicalPositionRad = physicalPositionRad * self.motorParams['AXIS_DIRECTION']
        # print(physicalPositionRad)
        physicalVelocityRad = physicalVelocityRad * self.motorParams['AXIS_DIRECTION']
        physicalCurrent = physicalCurrent * self.motorParams['AXIS_DIRECTION']
        # print("\r\npos raw val rad: {} ".format(physicalPositionRad))
        motor_status = [motor_id, physicalPositionRad, physicalVelocityRad, physicalCurrent]
        return motor_status

    def convert_physical_rad_to_raw(self, p_des_rad, v_des_rad, kp, kd, tau_ff):

        # Correct the Axis Direction
        p_des_rad = p_des_rad * self.motorParams['AXIS_DIRECTION']
        v_des_rad = v_des_rad * self.motorParams['AXIS_DIRECTION']
        tau_ff = tau_ff * self.motorParams['AXIS_DIRECTION']

        rawPosition = utils.float_to_uint(p_des_rad, self.motorParams['P_MIN'],
                                          self.motorParams['P_MAX'], 16)
        rawVelocity = utils.float_to_uint(v_des_rad, self.motorParams['V_MIN'],
                                          self.motorParams['V_MAX'], 12)
        rawTorque = utils.float_to_uint(tau_ff, self.motorParams['T_MIN'], self.motorParams['T_MAX'], 12)

        rawKp = ((maxRawKp * kp) / self.motorParams['KP_MAX'])

        rawKd = ((maxRawKd * kd) / self.motorParams['KD_MAX'])
        # print("pos raw val des: {}".format(rawPosition))

        return int(rawPosition), int(rawVelocity), int(rawKp), int(rawKd), int(rawTorque)

    def _send_raw_command(self, p_des, v_des, kp, kd, tau_ff):
        """
        Package and send raw (uint) values of correct length to the motor.
        _send_raw_command(desired position, desired velocity, position gain, velocity gain,
                        feed-forward torque)
        Sends data over CAN, reads response, and returns the motor status data (in bytes).
        """

        self._p_des_BitArray.uint = p_des
        self._v_des_BitArray.uint = v_des
        self._kp_BitArray.uint = kp
        self._kd_BitArray.uint = kd
        self._tau_BitArray.uint = tau_ff

        cmd_BitArray = self._p_des_BitArray.bin + self._v_des_BitArray.bin + self._kp_BitArray.bin \
                       + self._kd_BitArray.bin + self._tau_BitArray.bin

        self._cmd_bytes.bin = cmd_BitArray

        try:
            self._send_can_frame(self._cmd_bytes.tobytes())
            utils.waitOhneSleep(dt_sleep)
            msg = self._recv_can_frame()
            if msg == None:
                print("Nothing to parse")
            return msg
        except Exception as e:
            print('Error Sending Raw Commands!')
            print("Error: ", e)
            # return '-99','-99','-90'

    def send_deg_command(self, p_des_deg, v_des_deg, kp, kd, tau_ff):
        """
        TODO: Add assert statements to validate input ranges.
        Function to send data to motor in physical units:
        send_deg_command(position (deg), velocity (deg/s), kp, kd, Feedforward Torque (Nm))
        Sends data over CAN, reads response, and prints the current status in deg, deg/s, amps.
        """
        # p_des_deg = p_des_deg/64/4
        p_des_rad = math.radians(p_des_deg)
        v_des_rad = math.radians(v_des_deg)

        motor_status = self.send_rad_command(p_des_rad, v_des_rad, kp, kd, tau_ff)
        if motor_status !=None:
            can_id, can_dlc, motorStatusData = motor_status
            rawMotorData = self.decode_motor_status(motorStatusData)

            motor_status = self.convert_raw_to_physical_rad(rawMotorData[0], rawMotorData[1], rawMotorData[2],
                                                            rawMotorData[3])
            motor_id, pos_rad, vel_rad, curr = motor_status[0], motor_status[1], motor_status[2], motor_status[3]
            pos = math.degrees(pos_rad)
            vel = math.degrees(vel_rad)
            motor_status = [motor_id, pos, vel, curr]

        return motor_status


    def send_rad_command(self, p_des_rad, v_des_rad, kp, kd, tau_ff):
        """
        TODO: Add assert statements to validate input ranges.
        Function to send data to motor in physical units:
        send_rad_command(position (rad), velocity (rad/s), kp, kd, Feedforward Torque (Nm))
        Sends data over CAN, reads response, and prints the current status in rad, rad/s, amps.
        """
        # Check for Torque Limits
        if (tau_ff < self.motorParams['T_MIN']):
            print('Torque Commanded lower than the limit. Clipping Torque...')
            print('Commanded Torque: {}'.format(tau_ff))
            print('Torque Limit: {}'.format(self.motorParams['T_MIN']))
            tau_ff = self.motorParams['T_MIN']
        elif (tau_ff > self.motorParams['T_MAX']):
            print('Torque Commanded higher than the limit. Clipping Torque...')
            print('Commanded Torque: {}'.format(tau_ff))
            print('Torque Limit: {}'.format(self.motorParams['T_MAX']))
            tau_ff = self.motorParams['T_MAX']

        rawPos, rawVel, rawKp, rawKd, rawTauff = self.convert_physical_rad_to_raw(p_des_rad, v_des_rad, kp, kd, tau_ff)
        # print("raw in: " + str(rawPos))
        motor_message  = self._send_raw_command(rawPos, rawVel, rawKp, rawKd, rawTauff)
        motor_status = motor_message
        if motor_message !=None:
            can_id, can_dlc, motorStatusData = motor_message
            rawMotorData = self.decode_motor_status(motorStatusData)
            motor_status = self.convert_raw_to_physical_rad(rawMotorData[0], rawMotorData[1], rawMotorData[2], rawMotorData[3])
        return motor_status


    def change_motor_constants(self, P_MIN_NEW, P_MAX_NEW, V_MIN_NEW, V_MAX_NEW, KP_MIN_NEW,
                               KP_MAX_NEW, KD_MIN_NEW, KD_MAX_NEW, T_MIN_NEW, T_MAX_NEW):
        """
        Function to change the global motor constants. Default values are for AK80-6 motor from
        CubeMars. For a differnt motor, the min/max values can be changed here for correct
        conversion.
        change_motor_params(P_MIN_NEW (radians), P_MAX_NEW (radians), V_MIN_NEW (rad/s),
                            V_MAX_NEW (rad/s), KP_MIN_NEW, KP_MAX_NEW, KD_MIN_NEW, KD_MAX_NEW,
                            T_MIN_NEW (Nm), T_MAX_NEW (Nm))
        """
        self.motorParams['P_MIN'] = P_MIN_NEW
        self.motorParams['P_MAX'] = P_MAX_NEW
        self.motorParams['V_MIN'] = V_MIN_NEW
        self.motorParams['V_MAX'] = V_MAX_NEW
        self.motorParams['KP_MIN'] = KP_MIN_NEW
        self.motorParams['KP_MAX'] = KP_MAX_NEW
        self.motorParams['KD_MIN'] = KD_MIN_NEW
        self.motorParams['KD_MAX'] = KD_MAX_NEW
        self.motorParams['T_MIN'] = T_MIN_NEW
        self.motorParams['T_MAX'] = T_MAX_NEW
