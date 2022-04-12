#!/usr/bin/env python3
import time

import MotorsHandler
import rospy

JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/src/parameters/joint_params"
COMM_FREQ = 40  # Hz


if __name__ == '__main__':
    rospy.init_node('motors_abstractor', anonymous=True)

    # rc = subprocess.call("/home/taio/PycharmProjects/taio_ws/src/motors_abstraction/src/setup_can_interfacem3n3t3.sh")
    motors_handler = MotorsHandler.MotorsHandler(JOINT_PARAMS_PATH)
    time.sleep(1)
    motors_handler.init_motors()
    # rate = rospy.Rate(COMM_FREQ)
    rospy.Timer(rospy.Duration(1.0  / COMM_FREQ), motors_handler.publish_all_motors)
    try:

        # while not rospy.is_shutdown():
        #     verboseprint("----------------------------------")
        #     for joint in Joints:
        #         joint.publish_status(joint.send_position_and_update_status_demo(joint.des_angle)) #TODO Demo
        #         rospy.sleep(0.0001)
        #     rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("EXCEPTION EXCEPTION")
        pass
