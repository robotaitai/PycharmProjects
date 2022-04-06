
import cProfile, curses, math, os, time, subprocess
import numpy as np
import matplotlib as mpl
mpl.use("TkAgg")

import matplotlib.pyplot as plt

import yaml, json
from taio_ws.src.motors_abstraction.src import canMotorController as mot_con
import motor_watchdog as new_mot
import Jetson.GPIO as GPIO

import csv, sched

'''
EXPERIENCE PARAMS
'''
EXPERIENCE_NAME = 'full_test_a'
EXPERIENCE_NAME_MICRO = '_delete_'
EXPERIENCE_NUM = '2'
file_params = EXPERIENCE_NAME.split('_')

DELTA_T = 0.01
STEP_VALUE = 0.025
"""
MOTORS PARAMS
"""
kp = KP_VALUE = 10
kd = KD_VALUE = 1.0

# kp = KP_VALUE = float(file_params[2][2:])
# kd = KD_VALUE = float(file_params[3][2:])
SPEED_VALUE = 6
TORQUE_VALUE = 0
r_motor_id = '007'
MOTOR_TYPE = 'AK80_64_V2'
CAN_BUS = 'can0'
JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/parameters/joint_params"
Joints = {}

output_path = f"/mnt/nvme0n1p1/PycharmProjects/motors_experiments/s2r_jig/output/output_{EXPERIENCE_NAME + EXPERIENCE_NAME_MICRO + EXPERIENCE_NUM}_kp_{kp}_kd_{kd}"
print(output_path)
"""
ENV PARAMs
"""
pwd ='m3n3t3'
cmd = 'ls'
log_plot = False
JOINT_PARAMS_PATH = "motor_params"

f = open('files/'+EXPERIENCE_NAME+'.json')
data = json.load(f)
data = data['exp_0']
j = 0
tot_iter = len(data['dof_target'])



def main():
    parsed_joint_params = yaml.load(open(JOINT_PARAMS_PATH), Loader=yaml.FullLoader)

    # rc = subprocess.call('echo {} | sudo -S {}'.format(pwd, cmd), shell=True)
    # rc = subprocess.run(["sudo", "/home/taio/setup_can_interface.sh"], shell=True)
    rc = subprocess.run(["sudo",'-E', "/home/taio/setup_can_interface.sh"], shell=True)

    time.sleep(1)
    r_motor_controller = new_mot.MotorWatchdog(parsed_joint_params['left-ankle-roll'])
    time.sleep(STEP_VALUE)

    startTime = time.time()

    if r_motor_controller.enable_motor() != None:
        mot_id, pos, vel, curr = r_motor_controller.enable_motor()
    time.sleep(STEP_VALUE)
    # pos, vel, curr = r_motor_controller.set_zero_position()


    endTime = time.perf_counter()
    # if pos > 0:
    #     d_d = 1/10
    # else:
    #     d_d = -1/10
    # while abs(pos) >= 1/50:
    #     new_pos = pos + d_d
    #     print(new_pos)
    #     r_motor_controller_status = r_motor_controller.send_rad_command(new_pos, SPEED_VALUE, KP_VALUE,
    #                                                                     KD_VALUE, TORQUE_VALUE)
    #     mot_id, pos, vel, curr = r_motor_controller_status
    #     time.sleep(0.1)


    print("\r\nTime for One Command: {}".format(endTime - startTime))
    r_motor_controller_status = r_motor_controller.send_safe_rad_command(0, SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
    print("Motor in Zero")
    time.sleep(3)
    des_pos_array = data['dof_target']
    output_data =[]
    log={}
    log['dof_target'] = []
    log['dof_pos'] = []
    log['dof_vel'] = []
    log['torque_action'] = []
    log['real_pos']=[]
    log['real_vel']=[]
    log['real_torque']=[]
    log['hh']=[]
    log['overshooting']=[]
    log['danger_vel']=[]
    i=0
    while i < tot_iter:
        # print("\r\n time: {}".format((time.time()-startTime)))

        # print("\r\n time: {}".format(time.time()))
        if (time.time()-startTime) >= STEP_VALUE:
            # print(des_pos[i])
            # print("-----------------------------------------------------------------------")
            # pos, vel, curr = r_motor_controller.enable_motor()
            # if r_motor_controller.danger_velocity_zone:
            #     r_motor_controller_status = r_motor_controller.send_safe_rad_command(0, -1 * r_motor_controller.max_velocity_allowed, 0, 5, TORQUE_VALUE)
            # else:
            #     r_motor_controller_status = r_motor_controller.send_safe_rad_command((des_pos_array[i]), SPEED_VALUE, KP_VALUE, KD_VALUE, TORQUE_VALUE)
            r_motor_controller_status = r_motor_controller.send_safe_rad_command((des_pos_array[i]), SPEED_VALUE,
                                                                                 KP_VALUE, KD_VALUE, TORQUE_VALUE)
            if r_motor_controller_status != None:
                mot_id, pos, vel, curr = r_motor_controller_status
                # print("final position: {}\n".format(math.degrees(pos)))
                # print("final position: {}\n".format(math.degrees(pos)))
                # print("final position: {}\n".format(math.degrees(pos)))

                log['dof_target'].append(data['dof_target'][i])
                log['dof_pos'].append(data['dof_pos'][i])
                log['dof_vel'].append(data['dof_vel'][i])
                log['torque_action'].append(data['torque_action'][i])
                log['real_pos'].append(pos)
                log['real_vel'].append(vel)
                log['real_torque'].append(curr)
                log['hh'].append(r_motor_controller.delta_from_ss)
                log['overshooting'].append(r_motor_controller.overshooting)
                log['danger_vel'].append(r_motor_controller.danger_velocity_zone)
            time.sleep(DELTA_T)

            # output_data.append([float(data['dof_target'][i]), float(data['dof_pos'][i]), float(data['dof_vel'][i]), float(data['torque_action'][i]), pos, vel, curr])
            startTime = time.time()
            i+=5
            # print("iteration: {} out of; {}".format(i,tot_iter))

    status = r_motor_controller.disable_motor()

    """
    Save Json
    """
    with open(output_path+'.json', 'w') as outfile:
        json.dump(log, outfile)

    print("JSON saved")

    """
    Create new CSV
    """
    Header = ['Desired Potision', 'Sim Position', 'Sim Velocity', 'Sim Torque', 'Real Position', 'Real Velocity',
              'Real Torque','hh','overshooting','danger_vel']

    with open(output_path+'.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)

        # # write the header
        # writer.writerow(Header)
        writer.writerow(log.keys())
        writer.writerows(zip(*log.values()))

    """
    Create new Plots
    """
    with open(output_path + '.json', "w") as f:
        json.dump(log, f)

    if log_plot:
        t = range(tot_iter)
        # t = range(iter)

        plt.figure()
        plt.title(f"Dof position (kp:{kp}, kd:{kd})")
        plt.plot(t, log["dof_target"], t, log["dof_pos"], t, log["real_pos"])
        plt.xlabel("step")
        plt.ylabel("dof_pos[rad]")
        plt.legend(["target", "sim", "real"])
        # plt.show()

        plt.figure()
        plt.title(f"Dof velocity (kp:{kp}, kd:{kd})")
        plt.plot(t, log["dof_vel"], t, log["real_vel"])
        plt.xlabel("step")
        plt.ylabel("dof_vel[rad/s]")
        plt.legend(["sim", "real"])

        # plt.show()

        plt.figure()
        plt.title(f"Torques (kp:{kp}, kd:{kd})")
        # if controller:
        plt.plot( t, data["torque_action"][:tot_iter], t, log["real_torque"])
        plt.legend(["sim", "real"])
        # else:
        #     plt.plot(t, log["torque_sensor"])
        #     plt.legend(["sensor"])
        plt.xlabel("step")
        plt.ylabel("torque[Nm]")
        plt.show()



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
