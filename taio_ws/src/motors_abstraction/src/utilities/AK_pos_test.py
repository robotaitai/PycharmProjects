
import cProfile
import curses
import os
import time, subprocess
import numpy as np
import yaml
from taio_ws.src.motors_abstraction.src import canMotorController as mot_con
import csv, sched

EXPERIENCE_NAME = 'hip_pitch_200Hz_kp_50_kd_5'
ADD_SECOND_MOTOR = False
KP_VALUE = 50/9
KD_VALUE = 5/9
SPEED_VALUE = 0
TORQUE_VALUE = 0

r_motor_id = 0x08
MOTOR_TYPE = 'AK80_9_V2'
CAN_BUS = 'can0'

JOINT_PARAMS_PATH = "/home/taio/PycharmProjects/taio_ws/src/motors_watchdog/parameters/joint_params"
Joints = {}
DELTA_T = 0.01
STEP_VALUE = 0.025

pwd ='m3n3t3'
cmd = 'ls'
rc = subprocess.call('echo {} | sudo -S {}'.format(pwd, cmd), shell=True)
rc = subprocess.call("/home/taio/PycharmProjects/taio_ws/src/motors_abstraction/src/utilities/setup_can_interface.sh")


with open('/home/taio/PycharmProjects/taio_ws/src/motors_abstraction/src/utilities/hip_pitch_200Hz_kp_50_kd_5.csv', newline='') as csvfile:
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

def main():
    r_motor_controller = mot_con.CanMotorController(CAN_BUS, r_motor_id, MOTOR_TYPE)
    time.sleep(STEP_VALUE)

    startTime = time.time()

    if r_motor_controller.enable_motor() != None:
        mot_id, pos, vel, curr = r_motor_controller.enable_motor()
    time.sleep(STEP_VALUE)
    # pos, vel, curr = r_motor_controller.set_zero_position()


    endTime = time.perf_counter()

    print("\r\nTime for One Command: {}".format(endTime - startTime))

    zero = 0
    new_pos = int(0)
    arrow_key = ''
    i=0
    data = []
    while i<len(des_pos)-900:
        # print("\r\n time: {}".format((time.time()-startTime)))
        # print("\r\n time: {}".format(time.time()))
        if (time.time()-startTime) >= STEP_VALUE:
            # print(des_pos[i])
            print("-----------------------------------------------------------------------")
            # pos, vel, curr = r_motor_controller.enable_motor()
            r_motor_controller_status = r_motor_controller.send_rad_command((des_pos[i]), SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            if r_motor_controller_status != None:
                mot_id, pos, vel, curr = r_motor_controller_status
                print("final position: {}".format(pos))
            # time.sleep(DELTA_T)
                data.append([des_pos[i],sim_pos[i],sim_vel[i],sim_torque[i],pos,vel,curr])
                startTime = time.time()
                i+=1
                print("iteration: {} out of; {}".format(i,len(des_pos)))

    mot_id, pos, vel, curr = r_motor_controller.disable_motor()

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
