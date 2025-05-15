import rospy
import sys
import copy
# Brings in the SimpleActionClient
import actionlib

from geometry_msgs.msg import Pose
import yk_msgs.msg
import yk_msgs.srv
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def interpolate_quaternions(q_start, q_end, num_waypoints):
    key_times = [0, 1]
    key_rots = R.from_quat([q_start, q_end])

    # Set up SLERP
    slerp = Slerp(key_times, key_rots)

    # Generate interpolation times
    times = np.linspace(0, 1, num_waypoints)

    # Interpolate
    interp_rots = slerp(times)

    return interp_rots.as_quat()

def exec_traj_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/yk_builder/yk_execute_cartesian_trajectory_ft', yk_msgs.msg.ExecuteCartesianTrajectoryFTAction)


    # Waits until the action server has started up and started
    # listening for goals.
    print("Waiting for action server to start...")
    client.wait_for_server()
    # Creates a goal to send to the action server.
    waypoints = []
    start_pose = Pose()
    start_pose.position.x = 0.0
    start_pose.position.y = -0.27
    start_pose.position.z = 0.647
    start_pose.orientation.x = 1.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 0.0
    waypoints.append(start_pose)
    
    sub_client = rospy.ServiceProxy('/yk_builder/yk_set_pose', yk_msgs.srv.SetPose)
    sub_client.wait_for_service()
    sub_goal = yk_msgs.srv.SetPoseRequest()
    sub_goal.pose = start_pose
    sub_client.call(sub_goal)    
    
    # Add more waypoints as needed
    next_pose = copy.deepcopy(start_pose)
    quat_list = interpolate_quaternions(
        [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w],
        [-0.707, 0.707, 0.0, 0.0],
        100
    )
    for i in range(1, 100):
        # QUARTERNION WAYPOINTS 90 DEGREES
        next_pose = copy.deepcopy(next_pose)
        next_pose.orientation.x = quat_list[i][0]
        next_pose.orientation.y = quat_list[i][1]
        next_pose.orientation.z = quat_list[i][2]
        next_pose.orientation.w = quat_list[i][3]
        waypoints.append(next_pose)
    
    # Create the goal message
    goal = yk_msgs.msg.ExecuteCartesianTrajectoryFTGoal()
    goal.waypoints = waypoints
    # goal.force_target = [0.921, 2.69, 23.68]
    # goal.torque_target = [0.15, 0.12, 0.05]
    goal.force_target = [0, 0, 10]
    goal.torque_target = [-0.3, 0.49, -2.0]
    goal.max_velocity_scaling_factor = 0.3
    goal.max_acceleration_scaling_factor = 0.3
    goal.force_tolerance = 7
    # goal.blend_radius = 0.002
    breakpoint()
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    
    # ROS SERVICE CLIENT CALL
    sub_client.call(sub_goal)

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('yk_execute_trajectory_client')
        result = exec_traj_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)