#ifndef _YK_INTERFACE_
#define _YK_INTERFACE_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <yk_msgs/GoToPoseAction.h>
#include <yk_msgs/GoToJointsAction.h>
#include <yk_msgs/ExecuteCartesianTrajectoryAction.h>
#include <yk_msgs/GetPose.h>
#include <yk_msgs/GetPoseStamped.h>
#include <yk_msgs/SetPose.h>
#include <yk_msgs/SetJoints.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/kinematic_constraints/utils.h>
#include <std_srvs/Trigger.h>
#include <string.h>

class YK_Interface
{
private:
	ros::ServiceServer getPose_server_;
	ros::ServiceServer getPoseStamped_server_;
	ros::ServiceServer setPose_server_;
	ros::ServiceServer setJoints_server_;
	ros::ServiceServer executeTrajectory_server_;
	ros::ServiceServer stopTrajectory_server_;
	tf::TransformListener listener_;
	geometry_msgs::PoseStamped last_pose_;
	sensor_msgs::JointState last_joints_;
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<yk_msgs::GoToPoseAction> go_to_pose_as_;
	yk_msgs::GoToPoseFeedback go_to_pose_feedback_;
	yk_msgs::GoToPoseResult go_to_pose_result_;

	actionlib::SimpleActionServer<yk_msgs::GoToJointsAction> go_to_joints_as_;
	yk_msgs::GoToJointsFeedback go_to_joints_feedback_;
	yk_msgs::GoToJointsResult go_to_joints_result_;

	actionlib::SimpleActionServer<yk_msgs::ExecuteCartesianTrajectoryAction> execute_cartesian_trajectory_as_;
	yk_msgs::ExecuteCartesianTrajectoryFeedback execute_cartesian_trajectory_feedback_;
	yk_msgs::ExecuteCartesianTrajectoryResult execute_cartesian_trajectory_result_;

	moveit::planning_interface::MoveGroupInterface move_group_;
	
	double max_velocity_scaling_factor_;
	double max_acceleration_scaling_factor_;

public:
	YK_Interface(std::string group_name, ros::NodeHandle &nh_);

	/**
	 * @brief ROS service callback for "yk_get_pose" to get current pose of the manipulator
	*/
	bool getPose(yk_msgs::GetPose::Request &req, yk_msgs::GetPose::Response &res);

	// Redundant function. Changed getPose to have Header stamp.
	bool getPoseStamped(yk_msgs::GetPoseStamped::Request &req, yk_msgs::GetPoseStamped::Response &res);

	/**
	 * @brief ROS service callback for "yk_set_pose" to set the pose of the manipulator
	 * 
	 * @param req requested pose of the manipulator
	 * @param res current pose of the mauipulator at the end of the operation
	 * @return true: if operation was successful
	 * @return false: invalid/erroneous response
	*/
	bool setPose(yk_msgs::SetPose::Request &req, yk_msgs::SetPose::Response &res);

	bool setJoints(yk_msgs::SetJoints::Request &req, yk_msgs::SetJoints::Response &res);

	bool executeTrajectory(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res);

	bool stopTrajectory(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	void goToJointsCallback(const yk_msgs::GoToJointsGoalConstPtr &goal);

	void goToPoseCallback(const yk_msgs::GoToPoseGoalConstPtr &goal);
	
	void executeCartesianTrajectoryCallback(const yk_msgs::ExecuteCartesianTrajectoryGoalConstPtr & goal);

};

#endif
