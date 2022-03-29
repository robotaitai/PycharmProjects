#! /usr/bin/env python3

import rospy, yaml,unittest,rostest, random
from std_msgs.msg import Header

from sensor_msgs.msg import JointState

JOINT_PARAMS_PATH = "/home/taio/taio_ws/src/motors_watchdog/parameters/joint_params"

class myTestCase(unittest.TestCase):


    def test_watchdog_unit(self):

        test_values = {}
        parsed_joint_params = yaml.load(open(JOINT_PARAMS_PATH), Loader=yaml.FullLoader)
        for joint in parsed_joint_params:
            print(joint)
            pub = rospy.Publisher("/soma/" + joint, JointState, queue_size=10)
            # rospy.Subscriber('/soma/' + joint+"/limited", JointState, msg_validator)
            new_msg = JointState()
            new_msg.header = Header()
            new_msg.header.stamp = rospy.Time.now()
            new_msg.name = "/soma/"+joint
            value = random.random()
            test_values[joint] = value
            new_msg.position = [value * 90]
            new_msg.velocity = [value * 90]
            new_msg.effort = [value * 90]
            pub.publish(new_msg)
            joint_state = rospy.get_param("/soma/"+joint)
            print(joint_state)
            self.assertIsNotNone(joint_state)


        joint_state = rospy.get_param("/soma/left_bow", 1)
        self.assertIsNotNone(joint_state)

if __name__ == '__main__':
    rostest.rosrun('test_unit', 'runTest', myTestCase)
