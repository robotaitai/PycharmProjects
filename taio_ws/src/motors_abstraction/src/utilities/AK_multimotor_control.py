
import cProfile
import curses
import os
import time, subprocess
import numpy as np
import yaml

import canMotorController as mot_con
import csv, sched

EXPERIENCE_NAME = 'hip_pitch_200Hz_kp_50_kd_5'
ADD_SECOND_MOTOR = False
KP_VALUE = 50
KD_VALUE = 5
SPEED_VALUE = 0
TORQUE_VALUE = 0

r_motor_id = 0x03


JOINT_PARAMS_PATH = "/home/taio/PycharmProjects/taio_ws/src/motors_watchdog/parameters/joint_params"
Joints = {}
DELTA_T = 0.001
STEP_VALUE = 0.025


with open('../src/hip_pitch_200Hz_kp_50_kd_5.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    line_count = 0
    for row in spamreader:
        if line_count == 0:
            des_pos_dirty = row[2:]
        if line_count == 1:
            sim_pos_dirty = row[2:]
        if line_count == 2:
            sim_vel_dirty = row[2:]
        if line_count == 3:
            sim_torque_dirty = row[2:]
        line_count += 1
des_pos=[]

"""
Clear nitsan's data
"""
i=1
des_pos=[]
for i in range(len(des_pos_dirty)-1):
    b= float(des_pos_dirty[i].replace(" ", "").replace("]", ""))
    des_pos.append(b)

i=1
sim_pos=[]
for i in range(len(sim_pos_dirty)-1):
    b= float(sim_pos_dirty[i].replace(" ", "").replace("]", ""))
    sim_pos.append(b)

i=1
sim_vel=[]
for i in range(len(sim_vel_dirty)-1):
    b= float(sim_vel_dirty[i].replace(" ", "").replace("]", ""))
    sim_vel.append(b)

i=1
sim_torque=[]
for i in range(len(sim_torque_dirty)-1):
    b= float(sim_torque_dirty[i].replace(" ", "").replace("]", ""))
    sim_torque.append(b)

"""
Create new CSV
"""
Header = ['Desired Potision','Sim Position','Sim Velocity','Sim Torque','Real Position','Real Velocity','Real Torque']

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
        motor_type = parsed_joint_params[joint]['motor_type']

        new_motor =  mot_con.CanMotorController('can0', can_id, motor_type)
        Joints[joint] = new_motor


def main():
    g_motor_controller = mot_con.CanMotorController('can0', r_motor_id, 'AK80_64_V2')
    init_motors_can_interface(JOINT_PARAMS_PATH)

    r_knee_motor_controller = Joints['right_knee']
    time.sleep(STEP_VALUE)

    l_knee_motor_controller = Joints['left_knee']
    time.sleep(STEP_VALUE)

    r_ankle_motor_controller = Joints['right_ankle']
    time.sleep(STEP_VALUE)

    l_ankle_motor_controller = Joints['left_ankle']
    time.sleep(STEP_VALUE)

    # r_heap_roll_motor_controller = Joints['right_heap_roll']
    # time.sleep(STEP_VALUE)

    l_heap_roll_motor_controller = Joints['left_heap_roll']
    time.sleep(STEP_VALUE)

    startTime = time.time()

    motor_id, pos, vel, curr = r_knee_motor_controller.enable_motor()
    time.sleep(DELTA_T)
    motor_id, pos, vel, curr = l_knee_motor_controller.enable_motor()
    time.sleep(DELTA_T)
    motor_id, pos, vel, curr = r_ankle_motor_controller.enable_motor()
    time.sleep(DELTA_T)
    motor_id, pos, vel, curr = l_ankle_motor_controller.enable_motor()
    time.sleep(DELTA_T)
    # motor_id, pos, vel, curr = r_heap_roll_motor_controller.enable_motor()
    # time.sleep(DELTA_T)
    motor_id, pos, vel, curr = l_heap_roll_motor_controller.enable_motor()
    time.sleep(DELTA_T)

    # pos, vel, curr = r_motor_controller.set_zero_position()


    endTime = time.perf_counter()

    print("\r\nTime for One Command: {}".format(endTime - startTime))

    zero = 0
    new_pos = int(0)
    arrow_key = ''
    i=0
    data = []
    while i<len(des_pos)-800:
        # print("\r\n time: {}".format((time.time()-startTime)))
        # print("\r\n time: {}".format(time.time()))
        if (time.time()-startTime) >= STEP_VALUE:
            # print(des_pos[i])
            # print("-----------------------------------------------------------------------")
            motor_id, pos, vel, tor = r_knee_motor_controller.send_deg_command((des_pos[i]), SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            time.sleep(DELTA_T)
            motor_id, pos, vel, tor = l_knee_motor_controller.send_deg_command((des_pos[i]), SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            time.sleep(DELTA_T)
            motor_id, pos, vel, tor = r_ankle_motor_controller.send_deg_command((des_pos[i]), SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            time.sleep(DELTA_T)
            motor_id, pos, vel, tor = l_ankle_motor_controller.send_deg_command((des_pos[i]), SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            time.sleep(DELTA_T)
            # motor_id, pos, vel, tor = r_heap_roll_motor_controller.send_deg_command((des_pos[i]), SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            # time.sleep(DELTA_T)
            motor_id, pos, vel, tor = l_heap_roll_motor_controller.send_deg_command((des_pos[i]), SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            time.sleep(DELTA_T)

            data.append([des_pos[i],sim_pos[i],sim_vel[i],sim_torque[i],pos,vel,tor])
            startTime = time.time()
            i+=1
            print("iteration: {} out of; {}".format(i,len(des_pos)))

    motor_id, pos, vel, curr = r_knee_motor_controller.disable_motor()
    time.sleep(DELTA_T)
    motor_id, pos, vel, curr = l_knee_motor_controller.disable_motor()
    time.sleep(DELTA_T)
    motor_id, pos, vel, curr = r_ankle_motor_controller.disable_motor()
    time.sleep(DELTA_T)
    motor_id, pos, vel, curr = l_ankle_motor_controller.disable_motor()
    time.sleep(DELTA_T)
    # motor_id, pos, vel, curr = r_heap_roll_motor_controller.disable_motor()
    # time.sleep(DELTA_T)
    motor_id, pos, vel, curr = l_heap_roll_motor_controller.disable_motor()
    time.sleep(DELTA_T)
    with open('output'+ EXPERIENCE_NAME+'.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)

        # write the header
        writer.writerow(Header)

        # write multiple rows
        writer.writerows(data)

    exit()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
