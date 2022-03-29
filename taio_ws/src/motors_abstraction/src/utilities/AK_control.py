
import cProfile
import curses
import os
import time

import numpy as np
import yaml

import canMotorController as mot_con

screen = curses.initscr()
# curses.cbreak()
# screen.keypad(10)
# curses.halfdelay(5)           # How many tenths of a second are waited, from 1 to 255
# curses.noecho()

ADD_SECOND_MOTOR = False
KP_VALUE = 200
KD_VALUE = 5
SPEED_VALUE = 0
TORQUE_VALUE = 0

r_motor_id = 0x03
l_motor_id = 0x09
c_motor_id = 0x08


JOINT_PARAMS_PATH = "/home/taio/PycharmProjects/taio_ws/src/motors_watchdog/parameters/joint_params"
Joints = {}


def init_motors_can_interface(joint_params):
    parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)
    # print(parsed_joint_params)
    # TODO take care of default config and error in the yaml
    print(parsed_joint_params)
    for joint in parsed_joint_params:
        print("\r\nConnecting joint: {}".format(joint))
        if joint == 'all_joints':
            return 0
        can_id = parsed_joint_params[joint]['can_id']
        new_motor =  mot_con.CanMotorController('can0', can_id, 'AK80_9_V2')
        Joints[joint] = new_motor



# time.sleep(5)
DELTA_T = 0.1

def main():
    # l_motor_id = 0x03
    # init_motors_can_interface(JOINT_PARAMS_PATH)

    # r_motor_controller = Joints['left_knee']
    # l_motor_controller = Joints['right_knee']
    r_motor_controller = mot_con.CanMotorController('can0', r_motor_id, 'AK80_64_V2')
    time.sleep(DELTA_T)
    if ADD_SECOND_MOTOR:
        l_motor_controller = mot_con.CanMotorController('can0', l_motor_id, 'AK80_9_V2')
        c_motor_controller = mot_con.CanMotorController('can0', c_motor_id, 'AK80_9_V2')

    startTime = time.perf_counter()

    pos, vel, curr = r_motor_controller.enable_motor()
    if ADD_SECOND_MOTOR:
        time.sleep(DELTA_T)
        pos, vel, curr = l_motor_controller.enable_motor()
        pos, vel, curr = c_motor_controller.enable_motor()
    # if len(sys.argv) != 2:
    #     print('Provide CAN device name (can0, slcan0 etc.)')
    #     sys.exit(0)

    # print("\r\nUsing Socket {} for can communication".format('can0'))
    # print(type((sys.argv[1],)))

    # l_motor_controller = mot_con.CanMotorController('can0', l_motor_id, 'AK80_6_V1')

    # r_motor_controller = mot_con.CanMotorController('can0', r_motor_id,'AK80_9_V2')

    # pos, vel, curr = r_motor_controller.enable_motor()
    # print("Init Motor Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))

    # pos, vel, curr = r_motor_controller.set_zero_position()
    endTime = time.perf_counter()

    print("\r\nTime for One Command: {}".format(endTime - startTime))

    # time.sleep(3)

    # while abs(np.rad2deg(pos)) > 0.5:
    # pos, vel, curr = l_motor_controller.set_zero_position()
    # print("Zero Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))
    # pos, vel, curr = r_motor_controller.send_deg_command(0, 90, 0, 2, 0)

    zero = 0
    new_pos = int(0)
    arrow_key = ''
    STEP_VALUE = 1

    while arrow_key != ord('q'):

        print("--------------------------------------------")
        arrow_key =  ord('d')

        # arrow_key = screen.getch()
        # pos, vel, curr = r_motor_controller.enable_motor()

        if arrow_key == ord('t'):
            # pos, vel, curr = r_motor_controller.decode_motor_status()
            pass
        if arrow_key == ord('u'):
            # pos, vel, curr = r_motor_controller.decode_motor_status()
            STEP_VALUE += 1
            print("step is: {}".format(STEP_VALUE))
        if arrow_key == ord('j'):
            # pos, vel, curr = r_motor_controller.decode_motor_status()
            STEP_VALUE -=1
            print("step is: {}".format(STEP_VALUE))

        if arrow_key == ord('y'):
            pos, vel, curr = r_motor_controller.set_zero_position()
            if ADD_SECOND_MOTOR:

                time.sleep(DELTA_T)
                pos, vel, curr = l_motor_controller.set_zero_position()
                pos, vel, curr = c_motor_controller.set_zero_position()

            new_pos = pos

        if arrow_key == ord('w'):
            pos, vel, curr = r_motor_controller.enable_motor()
            if ADD_SECOND_MOTOR:
                time.sleep(DELTA_T)
                pos, vel, curr = l_motor_controller.enable_motor()
                time.sleep(DELTA_T)
                pos, vel, curr = c_motor_controller.enable_motor()

        if arrow_key == ord('s'):
            pos, vel, curr = r_motor_controller.disable_motor()
            time.sleep(DELTA_T)
            pos, vel, curr = c_motor_controller.disable_motor()
        if arrow_key == ord('d'):
            new_pos += STEP_VALUE
            new_pos = new_pos%360
            # if pos >MAX_ENCODER_VALUE:
            #    pos = max(pos - MAX_ENCODER_VALUE-1,0)
            # if pos < 0:
            #    pos = 0
            pos, vel, curr = r_motor_controller.send_deg_command(new_pos, SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            if ADD_SECOND_MOTOR:
                time.sleep(DELTA_T)
                pos, vel, curr = l_motor_controller.send_deg_command(new_pos, SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
                time.sleep(DELTA_T)
                pos, vel, curr = c_motor_controller.send_deg_command(new_pos, SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)

            # print('\r\n mot pos: {}, vel: {}, torqe: {} '.format(pos, vel, curr))

        if arrow_key == ord('a'):
            new_pos -= STEP_VALUE
            new_pos = new_pos%360

            # if pos >= MAX_ENCODER_VALUE:
            #    pos = MAX_ENCODER_VALUE-1
            # if pos < 0:
            #    pos = min(pos + MAX_ENCODER_VALUE-1, MAX_ENCODER_VALUE-1)
            pos, vel, curr = r_motor_controller.send_deg_command(new_pos, SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            if ADD_SECOND_MOTOR:

                time.sleep(DELTA_T)
                pos, vel, curr = l_motor_controller.send_deg_command(new_pos, SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
                time.sleep(DELTA_T)
                pos, vel, curr = c_motor_controller.send_deg_command(new_pos, SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            # print("\r\n Current Position: {}, Velocity: {}, Torque: {}".format(real_pos, vel, curr))
        print('\r\n mot pos: {}, des pos: {} '.format(np.rad2deg(pos), new_pos))
        screen.refresh()
        # time.sleep(0.1)

    return 0

def main1():
    # Motor ID
    r_motor_id = 0x05
    # l_motor_id = 0x03

    # if len(sys.argv) != 2:
    #     print('Provide CAN device name (can0, slcan0 etc.)')
    #     sys.exit(0)

    print("Using Socket {} for can communication".format('can0'))
    # print(type((sys.argv[1],)))

    # l_motor_controller = mot_con.CanMotorController('can0', l_motor_id, 'AK80_6_V1')
    r_motor_controller = mot_con.CanMotorController('can0', r_motor_id, 'AK80_64_V2')

    startTime = time.perf_counter()

    # pos, vel, curr = l_motor_controller.enable_motor()

    # pos, vel, curr = motor_controller.send_deg_command(0, 0, 0, 0, 0)
    # print("Initial Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))

    pos, vel, curr = r_motor_controller.enable_motor()

    # pos, vel, curr = motor_controller.send_deg_command(0, 0, 0, 0, 0)
    print("Initial Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),
                                                                  curr))
    endTime = time.perf_counter()

    print("Time for One Command: {}".format(endTime - startTime))

    time.sleep(1)

    # while abs(np.rad2deg(pos)) > 0.5:
    # pos, vel, curr = l_motor_controller.set_zero_position()
    # print("Zero Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))
    pos, vel, curr = r_motor_controller.set_zero_position()
    print("Zero Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))

    time.sleep(1)

    # Moving 180 degrees
    # pos, vel, curr = l_motor_controller.send_deg_command(40, 90, 0, 2, 10)
    # print("Moving Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))

    pos, vel, curr = r_motor_controller.send_deg_command(100, 90, 10, 2, 10)
    print("Moving Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
    time.sleep(2)
    # Send zeros
    # pos, vel, curr = l_motor_controller.send_deg_command(0, 50, 0, 0, 10)
    # print("Reached Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
    pos, vel, curr = r_motor_controller.send_deg_command(0, 50, 10, 2, 10)
    print("Reached Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
    time.sleep(1)

    # pos, vel, curr = l_motor_controller.disable_motor()
    # print("Final Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))
    pos, vel, curr = r_motor_controller.disable_motor()
    print("Final Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))
    # Use a breakpoint in the code line below to debug your script.


def setZeroPosition(motor, initPos):

    pos = initPos

    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),
                                                                curr))

#
# def main2():
#     r_motor_controller.enable_motor()
#     steps_array = np.linspace(1000, 1000, 1)
#     startdtTest = time.time()
#     # steps_array = np.linspace(0, 5, 6)
#     for i in steps_array:
#         print("Starting Profiler for {} Commands".format(i))
#         cmdString ="motor_send_n_commands({})".format(int(i))
#         profiler = cProfile.Profile()
#         profiler.run(cmdString)
#         profiler.print_stats()
#
#     enddtTest = time.time()
#     r_motor_controller.disable_motor()
#     dt = (enddtTest - startdtTest) / 1000
#     cmd_freq = 1 / dt
#     print("Dt = {}".format(dt))
#     print("Command Frequency: {} Hz".format(cmd_freq))
#

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
