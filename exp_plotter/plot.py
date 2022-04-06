# This is a sample Python script.

import json

import matplotlib as mpl
# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.
import matplotlib.pyplot as plt

'''
EXPERIENCE PARAMS   
'''
EXPERIENCE_NAME = 'output_chirp_test_kp_100_kd_0.1'
EXPERIENCE_NAME_MICRO = 'fing_hh_equation'
EXPERIENCE_NUM = '1'
file_params = EXPERIENCE_NAME.split('_')
DELTA_T = 0.01
STEP_VALUE = 0.0005


kp = KP_VALUE = 100
# kp = KP_VALUE = float(file_params[2][2:])
# kd = KD_VALUE = float(file_params[3][2:])
kd = KD_VALUE = 0.5

output_path = f"output/output_{EXPERIENCE_NAME + EXPERIENCE_NAME_MICRO + EXPERIENCE_NUM}_kp_{kp}_kd_{kd}"

f = open('/Users/taio/PycharmProjects/motors_experiments/s2r_jig/'+output_path+'.json')
log = json.load(f)

j = 0
tot_iter = len(log['dof_target'])

mpl.use("Qt5Agg")

def print_hi(name):
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
    plt.plot(t, log["torque_action"][:tot_iter], t, log["real_torque"])
    plt.legend(["sim", "real"])
    # else:
    #     plt.plot(t, log["torque_sensor"])
    #     plt.legend(["sensor"])
    plt.xlabel("step")
    plt.ylabel("torque[Nm]")
    plt.show()
    print(f'Hi, {name}')  # Press ⌘F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
