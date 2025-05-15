import rospy
import sys
import copy
# Brings in the SimpleActionClient
import actionlib

from geometry_msgs.msg import Pose
import yk_msgs.msg

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
    start_pose.position.x = 0.29
    start_pose.position.y = -0.120
    start_pose.position.z = 0.394
    start_pose.orientation.x = 0.500
    start_pose.orientation.y = -0.500
    start_pose.orientation.z = 0.500
    start_pose.orientation.w = 0.500
    waypoints.append(start_pose)
    
    # Add more waypoints as needed
    next_pose = copy.deepcopy(start_pose)
    for i in range(1, 11):
        next_pose = copy.deepcopy(next_pose)
        next_pose.position.y += 0.01
        # next_pose.position.x += 0.01
        waypoints.append(next_pose)
    
    # Create the goal message
    goal = yk_msgs.msg.ExecuteCartesianTrajectoryFTGoal()
    goal.waypoints = waypoints
    # goal.force_target = [0.921, 2.69, 23.68]
    # goal.torque_target = [0.15, 0.12, 0.05]
    goal.force_target = [-1, 0, 0]
    goal.max_velocity_scaling_factor = 0.3
    goal.max_acceleration_scaling_factor = 0.3
    goal.force_tolerance = 4
    goal.force_adj_speed_factor = 0.01
    # goal.blend_radius = 0.002
    breakpoint()
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

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