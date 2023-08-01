#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
import time

def disable_robot_service(ns):
    rospy.wait_for_service(f'/{ns}/robot_disable')
    try:
        enable_robot = rospy.ServiceProxy(f'/{ns}/robot_disable', Trigger)
        resp = enable_robot()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('disable_robot_client')
    ns = rospy.get_param('enable_robot/namespace')
    rospy.loginfo(f"Requesting {ns}/robot_disable service...")
    #time.sleep(3)
    disable_robot_service(ns)
