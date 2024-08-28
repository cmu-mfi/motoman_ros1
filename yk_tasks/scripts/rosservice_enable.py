#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger
import time

def enable_robot_service(ns):
    rospy.wait_for_service(f'/{ns}/robot_enable')
    try:
        enable_robot = rospy.ServiceProxy(f'/{ns}/robot_enable', Trigger)
        resp = enable_robot()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('enable_robot_client')
    ns = rospy.get_param('enable_robot/namespace')
    rospy.loginfo(f"Requesting {ns}/robot_enable service...")
    time.sleep(3)
    enable_robot_service(ns)
