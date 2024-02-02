#ifndef _YK_INTERFACE_
#define _YK_INTERFACE_

#include <ros/ros.h>
#include <yk_tasks/SetPose.h>
#include <yk_tasks/GetPose.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit/kinematic_constraints/utils.h>
#include <string.h>

class YK_Interface
{
private:
	ros::ServiceServer setPose_server_;
	ros::ServiceServer getPose_server_;
	tf::TransformListener listener_;
	geometry_msgs::PoseStamped last_pose_;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface move_group_;
	
	double max_velocity_scaling_factor_;
	double max_acceleration_scaling_factor_;

	

public:
	YK_Interface(std::string group_name, ros::NodeHandle &nh_);

	/**
	 * @brief ROS service callback for "yk_set_pose" to set the pose of the manipulator
	 * 
	 * @param req requested pose of the manipulator
	 * @param res current pose of the mauipulator at the end of the operation
	 * @return true: if operation was successful
	 * @return false: invalid/erroneous response
	*/
	bool setPose(yk_tasks::SetPose::Request &req, yk_tasks::SetPose::Response &res);

	/**
	 * @brief ROS service callback for "yk_get_pose" to get current pose of the manipulator
	*/
	bool getPose(yk_tasks::GetPose::Request &req, yk_tasks::GetPose::Response &res);
};

#endif