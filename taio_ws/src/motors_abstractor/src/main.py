#!/usr/bin/env python
import time, logging,yaml
import motors_handler
import rospy




JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/src/parameters/joint_params"
logger = logging.getLogger(__name__)


if __name__ == '__main__':


    logger.addHandler(logging.StreamHandler())
    logger.info("Starting motors abstractor node")
    rospy.init_node('motors_abstractor', anonymous=True)

    motors_handler = motors_handler.MotorsHandler(JOINT_PARAMS_PATH)
    time.sleep(0.1)
    motors_handler.init_motors()
    # rate = rospy.Rate(COMM_FREQ)
    try:
        motors_handler.run()
    except rospy.ROSInterruptException:
        logger.error("EXCEPTION EXCEPTION")
        pass
