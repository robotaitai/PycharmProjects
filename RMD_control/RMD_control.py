#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  RMD_control.py
#  
#  Copyright 2022  <pi@raspberrypi>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

import can, time, os, sys, curses, math
from bitstring import BitArray

MOTOR_ID = 0x142

screen = curses.initscr()
curses.cbreak()
screen.keypad(1)

RMD_X8_POSITION_CTRL_4_CMD = 0xA4
_recv_bytes = BitArray(uint=0, length=64)
# Working parameters for AK80-9 V2.0 firmware

RMD_X6_PARAMS = {
    "P_MIN": -180.0,
    "P_MAX": 180.0,
    "V_MIN": -25.64,
    "V_MAX": 25.64,
    "KP_MIN": 0.0,
    "KP_MAX": 500.0,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -18.0,
    "T_MAX": 18.0,
    "AXIS_DIRECTION": -1,
    "IS_AK": True
}

maxRawPosition = 2 ** 16 - 1  # 16-Bits for Raw Position Values
maxRawVelocity = 2 ** 12 - 1  # 12-Bits for Raw Velocity Values
maxRawTorque = 2 ** 12 - 1  # 12-Bits for Raw Torque Values
maxRawKp = 2 ** 12 - 1  # 12-Bits for Raw Kp Values
maxRawKd = 2 ** 12 - 1  # 12-Bits for Raw Kd Values
maxRawCurrent = 2 ** 12 - 1  # 12-Bits for Raw Current Values
dt_sleep = 0.0001  # Time before motor sends a reply

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


class position_control_1_cmd:

    def __init__(self, can_id, speed_limit):

        self.can_id = can_id
        self.pos_des =0
        self.pos_des_raw =0
        self.pos_act =0
        self.speed_limit = int(speed_limit)
        self.cmd_byte = RMD_X8_POSITION_CTRL_4_CMD
        self.spin_direction = CLOCKWISE
        self.speed_limited_low = self.speed_limit & 0xFF
        self.speed_limited_high = self.speed_limit>>8 & 0xFF
        self.pos_ctrl_low = 0x00
        self.pos_ctrl_high = 0x00
        self.data6 = 0x00
        self.data7 = 0x00
        self._empty_bytes = BitArray(uint=0,length=8)

        self.motorParams = RMD_X6_PARAMS
        self._p_des_BitArray = BitArray(uint=float_to_uint(0, self.motorParams['P_MIN'],
                                                               self.motorParams['P_MAX'], 32), length=32)
        self._v_des_BitArray = BitArray(uint=float_to_uint(0, self.motorParams['V_MIN'],
                                                               self.motorParams['V_MAX'], 16), length=16)
        self._cmd_bytes = BitArray(uint=0, length=64)
        

    def update_pos(self, new_pos, dir):
        # convert rpm to dps
        speed_limited = self.speed_limit
        # speed_limited = int(self.speed_limit * 360)
        # speed_limited = speed_limited / 60

        # Convert 1degree/LSB to 0.01degree/LSB
        #new_pos = int(new_pos * 100)
        #new_pos = new_pos * 6
        self.spin_direction = dir

        self.speed_limited_low = int(speed_limited) & 0xFF
        self.speed_limited_high = int(speed_limited)>>8 & 0xFF

        self.pos_ctrl_lowest = int(new_pos) & 0xFF
        self.pos_ctrl_low = int(new_pos)>>8 & 0xFF
        self.pos_ctrl_high = int(new_pos)>>16 & 0xFF
        self.pos_ctrl_highest = int(new_pos)>>24 & 0xFF

    def parse_msg(self):
        data0 = self.cmd_byte
        data1 = 0x00
        data2 = self.speed_limited_low
        data3 = self.speed_limited_high
        data4 = self.pos_ctrl_lowest
        data5 = self.pos_ctrl_low
        data6 = self.pos_ctrl_high
        data7 = self.pos_ctrl_highest
        return [data0,data1,data2,data3,data4,data5,data6,data7]
        

def send_deg_command(bus, mot):
    """
    TODO: Add assert statements to validate input ranges.
    Function to send data to motor in physical units:
    send_deg_command(position (deg), velocity (deg/s), kp, kd, Feedforward Torque (Nm))
    Sends data over CAN, reads response, and prints the current status in deg, deg/s, amps.
    """
    p_des_rad = math.radians(mot.pos_des)*188
    print('\r\n pos in rad: {}'.format(p_des_rad))
    pos_rad= send_rad_command(bus, mot, p_des_rad)
    
    pos = math.degrees(pos_rad)
    return pos
    
    
    
    
def send_rad_command(bus, mot, p_des_rad):
    """
    TODO: Add assert statements to validate input ranges.
    Function to send data to motor in physical units:
    send_rad_command(position (rad), velocity (rad/s), kp, kd, Feedforward Torque (Nm))
    Sends data over CAN, reads response, and prints the current status in rad, rad/s, amps.
    """
    # Check for Torque Limits
    #if (tau_ff < self.motorParams['T_MIN']):
    #    print('Torque Commanded lower than the limit. Clipping Torque...')
    #    print('Commanded Torque: {}'.format(tau_ff))
    #    print('Torque Limit: {}'.format(self.motorParams['T_MIN']))
    #    tau_ff = self.motorParams['T_MIN']
    #elif (tau_ff > self.motorParams['T_MAX']):
    #    print('Torque Commanded higher than the limit. Clipping Torque...')
    #    print('Commanded Torque: {}'.format(tau_ff))
    #    print('Torque Limit: {}'.format(self.motorParams['T_MAX']))
    #    tau_ff = self.motorParams['T_MAX']

    mot.pos_des_raw = convert_physical_rad_to_raw(mot, p_des_rad)
    print('\r\n pos in rad physical: {}'.format(mot.pos_des_raw))
    motorStatusData = send_and_recieve(bus, mot)

    positionRawValue, velocityRawValue, torqueBitArray, temperatureRawValue = decode_motor_status(motorStatusData)
    pos = convert_raw_to_physical_rad(mot, positionRawValue)

    return pos

def convert_raw_to_physical_rad(mot, positionRawValue):
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

    physicalPositionRad = uint_to_float(positionRawValue, mot.motorParams['P_MIN'],
                                        mot.motorParams['P_MAX'], 16)

    # Correct Axis Direction
    physicalPositionRad = physicalPositionRad * mot.motorParams['AXIS_DIRECTION']


    return physicalPositionRad


def convert_physical_rad_to_raw(mot, p_des_rad):

    # Correct the Axis Direction

    rawPosition = float_to_uint(p_des_rad, mot.motorParams['P_MIN'],
                                mot.motorParams['P_MAX'], 16)


    return int(rawPosition)
        
        
        
        
def send_and_recieve(bus,mot):
    
    mot._p_des_BitArray.uint = mot.pos_des_raw
    mot._v_des_BitArray.uint = mot.speed_limit
    
    cmd_BitArray = bin(int(mot.cmd_byte))[2:]+ mot._empty_bytes.bin + mot._v_des_BitArray.bin + mot._p_des_BitArray.bin[24:32] + mot._p_des_BitArray.bin[16:24] + mot._p_des_BitArray.bin[8:16] + mot._p_des_BitArray.bin[:8]
    
    mot._cmd_bytes.bin = cmd_BitArray

    try:
        msg = can.Message(arbitration_id=mot.can_id, data=mot._cmd_bytes.tobytes(), is_extended_id=False)        

        bus.send(msg)
        print('\r\n sent: {}'.format(msg))

        time.sleep(0.001)
        msg = bus.recv()
        #print("\r\n recieved: {}".format(msg))
        real_pos, vel, curr, temp = decode_motor_status(msg.data)
        print("\r\n Current Position: {}, Velocity: {}, Torque: {}".format(real_pos, vel, curr))
        can_id, can_dlc, data = msg.arbitration_id, msg.dlc, msg.data
        return data
    except Exception as e:
        print("Unable to Receive CAN Franme.")
        print("Error: ", e)


def send_and_recieve_cmd(bus, mot):
    msg = can.Message(arbitration_id=mot.can_id, data=mot.data, is_extended_id=False)        
    print('\r\n sent: {}'.format(msg))
    bus.send(msg)
    time.sleep(0.001)
    msg = bus.recv()  # Wait until a message is received.
    print("\r\n recieved: {}".format(msg))
    return msg.data



def decode_motor_status(data):
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
    _recv_bytes.bytes = data
    dataBitArray = _recv_bytes.bin

    temperatureBitArray = dataBitArray[8:16]
    
    torqueLowBitArray = dataBitArray[16:24]
    torqueHighBitArray = dataBitArray[24:32]
    
    velocityLowBitArray = dataBitArray[32:40]
    velocityHighBitArray = dataBitArray[40:48]
    
    positionLowBitArray = dataBitArray[48:56]
    positionHighBitArray = dataBitArray[56:64]
    
    positionBitArray = positionHighBitArray + positionLowBitArray
    velocityBitArray = velocityHighBitArray + velocityLowBitArray
    torqueBitArray = torqueHighBitArray + torqueLowBitArray
    
    
    # Separate motor satus values from the bit string.
    # Motor ID not considered necessary at the moment.
    # motor_id = dataBitArray[:8]
    #positionBitArray = dataBitArray[8:24]
    #velocityBitArray = dataBitArray[24:36]
    #currentBitArray = dataBitArray[36:48]

    # motor_id = int(motor_id, 2)
    positionRawValue = int(positionBitArray, 2)
    velocityRawValue = int(velocityBitArray, 2)
    torqueBitArray = int(torqueBitArray, 2)
    
    temperatureRawValue = int(temperatureBitArray,2)

    # TODO: Is it necessary/better to return motor_id?
    # return motor_id, positionRawValue, velocityRawValue, currentRawValue
    return positionRawValue, velocityRawValue, torqueBitArray, temperatureRawValue   
    
STEP_VALUE = 0.1
RMD_SPEED_LIMITED = 514
CLOCKWISE = 0x00
COUNTER_CLOCKWISE = 0x01
MAX_ENCODER_VALUE =  2**16

def _get_message(msg):
    return msg

def main(args):
    mot = position_control_1_cmd(can_id=MOTOR_ID,speed_limit=RMD_SPEED_LIMITED)

    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    mot.data=[0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    send_and_recieve_cmd(bus,mot)
    mot.data=[0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    send_and_recieve_cmd(bus,mot)
    mot.data=[0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    send_and_recieve_cmd(bus,mot)
    
    zero = 0
    new_pos = int(0)
    arrow_key =''
    buffer = can.BufferedReader()


    while arrow_key != ord('q'):
        arrow_key = screen.getch()

        if arrow_key == ord('t'):
            mot.data=[0x9c,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
            send_and_recieve_cmd(bus,mot)
            
        if arrow_key == ord('y'):
            mot.data=[0xa4,0x00,0x3e,0x80,0xa0,0x8c,0x00,0x00]
            new_pos=0
            send_and_recieve_cmd(bus,mot)   
        if arrow_key == ord('w'):
            mot.data=[0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
            send_and_recieve_cmd(bus,mot)

        if arrow_key == ord('s'):
            mot.data=[0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
            send_and_recieve_cmd(bus,mot)

        if arrow_key == ord('d'):
            new_pos += STEP_VALUE
            #if pos >MAX_ENCODER_VALUE:
            #    pos = max(pos - MAX_ENCODER_VALUE-1,0)
            #if pos < 0:
            #    pos = 0
            mot.cmd_byte = RMD_X8_POSITION_CTRL_4_CMD
            mot.pos_des = new_pos
            motor_pos = send_deg_command(bus, mot)
            print('\r\n pos: {}'.format(motor_pos))
            #mot.update_pos(pos, CLOCKWISE)
            #mot.data = mot.parse_msg()
            #msg_bck = send_and_recieve(bus,data,mot)
            #real_pos, vel, curr, temp = decode_motor_status(msg_bck)
            #print("\r\n Current Position: {}, Velocity: {}, Torque: {}".format(real_pos, vel, curr)) 
        if arrow_key == ord('a'):
            new_pos -= STEP_VALUE
            #if pos >= MAX_ENCODER_VALUE:
            #    pos = MAX_ENCODER_VALUE-1
            #if pos < 0:
            #    pos = min(pos + MAX_ENCODER_VALUE-1, MAX_ENCODER_VALUE-1)
            mot.cmd_byte = RMD_X8_POSITION_CTRL_4_CMD
            mot.pos_des = new_pos
            motor_pos = send_deg_command(bus, mot)
            print('\r\n pos: {}'.format(motor_pos))
            #print("\r\n Current Position: {}, Velocity: {}, Torque: {}".format(real_pos, vel, curr))    
        print("\r\n" + str(new_pos))
        #screen.refresh()
        # time.sleep(0.1)

    return 0

if __name__ == '__main__':
    os.system('sudo /sbin/ip link set can0 down')
    os.system("sudo ifconfig can0 txqueuelen 1000")
    os.system('sudo /sbin/ip link set can0 up type can bitrate 1000000')
    sys.exit(main(sys.argv))
