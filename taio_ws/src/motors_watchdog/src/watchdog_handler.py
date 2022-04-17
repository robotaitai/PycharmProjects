from std_msgs.msg import String
import motor_watchdog
import yaml, rospy

import logging
logger = logging.getLogger(__name__)

class WatchdogHandler:
    def __init__(self, joint_params):
        self.joints = []
        self.subscriber = rospy.Subscriber("/happiness/motors_watchdog", String, self.ros_actions)
        parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)
        for dof in parsed_joint_params:
            self.joints.append(motor_watchdog.MotorWatchdog(parsed_joint_params[dof]))

    def ros_actions(self, msg):
        print(msg)


    def run(self):
        rospy.spin()

