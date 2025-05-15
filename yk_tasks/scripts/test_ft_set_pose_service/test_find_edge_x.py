import rospy
import sys
import copy

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
from yk_msgs.srv import SetPoseFT, SetPoseFTRequest, SetPoseFTResponse

ROBOT_NAMESPACE = "yk_destroyer"
XTRAVEL = 0.025 # m
FORCE_DIFF = 0.05 # N
SPEED_FACTOR = 0.002

def find_edge_x():
    print("JOG THE ROBOT TO AN EDGE OFFSET")
    
    x_dir_input = input("Enter the x direction (+ or -): ")
    while x_dir_input not in ["+", "-"]:
        print("Invalid input. Please enter + or -.")
        x_dir_input = input("Enter the x direction (+ or -): ")
        
    if x_dir_input == "+":
        x_dir = 1
    elif x_dir_input == "-":
        x_dir = -1
        
    # Get the current pose, force readings of the robot
    current_pose = rospy.wait_for_message(f'/{ROBOT_NAMESPACE}/tool0_pose', PoseStamped)
    current_pose = current_pose.pose
    
    # Create a service proxy to call the find_edge_x service
    rospy.wait_for_service(f'/{ROBOT_NAMESPACE}/yk_set_pose_ft')
    set_pose_ft = rospy.ServiceProxy(f'/{ROBOT_NAMESPACE}/yk_set_pose_ft', SetPoseFT)
    
    # Create a request object
    request = SetPoseFTRequest()
    request.pose = current_pose
    request.pose.position.x = current_pose.position.x + (x_dir * XTRAVEL)
    request.use_force = [True, False, False]
    
    request.force_thresholds[0] = FORCE_DIFF
    request.max_velocity_scaling_factor = SPEED_FACTOR
    request.max_acceleration_scaling_factor = SPEED_FACTOR
        
    # Call the service and get the response
    try:
        response = set_pose_ft(request)
    except rospy.ServiceException as e:
        print("Service call failed")
        return None
    
    return response.pose.position.x if response.is_force_termination else None
    

if __name__ == '__main__':
    try:
        rospy.init_node('yk_find_edge_x_client')
        result = find_edge_x()
        print("Result: ", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)