#!/usr/bin/env python
import time, logging, yaml
import rospy
import watchdog_handler
from hydra import compose, initialize
from omegaconf import OmegaConf

logger = logging.getLogger(__name__)
JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/src/parameters/joint_params"

if __name__ == '__main__':

    # default = "../config/wd_config.yaml"
    # # Configuring hydra
    # yaml_path_split = default.split("/")
    #
    logger.addHandler(logging.StreamHandler())
    logger.info("Starting motors watchdog node")
    rospy.init_node('motors_watchdog', anonymous=True)


    watchdog_handler = watchdog_handler.WatchdogHandler(JOINT_PARAMS_PATH)
    time.sleep(0.1)
    # watchdog_handler.init_motors()
    # rate = rospy.Rate(COMM_FREQ)
    try:
        watchdog_handler.run()
    except rospy.ROSInterruptException:
        logger.error("EXCEPTION EXCEPTION")
        pass
