#include <yk_interface.h>

YK_Interface::YK_Interface(std::string group_name, ros::NodeHandle &nh_)
	: move_group_(group_name)
{
	ROS_INFO_STREAM("planning frame.." << move_group_.getPlanningFrame());
	ROS_INFO_STREAM("pose reference frame.."<< move_group_.getPoseReferenceFrame());
	ROS_INFO_STREAM("End Effector.." << move_group_.getEndEffector());
	ROS_INFO_STREAM("End Effector Link.." << move_group_.getEndEffectorLink());

	// SERVICE SERVERS
	setPose_server_ = nh_.advertiseService("yk_set_pose", &YK_Interface::setPose, this);
	getPose_server_ = nh_.advertiseService("yk_get_pose", &YK_Interface::getPose, this);
}

bool YK_Interface::setPose(yk_tasks::SetPose::Request &req, yk_tasks::SetPose::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_INFO_STREAM("Let's Move!!");
	// moveit::planning_interface::MoveGroupInterface move_group."manipulator"); //move_group name to be changes
	move_group_.setPoseReferenceFrame(req.base_frame);
	ROS_INFO_STREAM("Planning Frame.." << move_group_.getPlanningFrame());
	move_group_.setPoseTarget(req.pose);
	moveit_msgs::MoveItErrorCodes ret_val = move_group_.move();
	// return value is of type MoveitErrorCodes message

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		ret_val = move_group_.move();
		ROS_INFO_STREAM("Timed out! Retrying..");
	}

	last_pose_ = move_group_.getCurrentPose();
	ROS_INFO_STREAM(last_pose_);

	res.pose = last_pose_.pose;
	async_spinner.stop();

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		ROS_INFO_STREAM("Move failed due to ERROR CODE=" << ret_val);
		return false;
	}

	return true;
}

bool YK_Interface::getPose(yk_tasks::GetPose::Request &req,
			 yk_tasks::GetPose::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();
	ROS_INFO("getPoseInfo");

	last_pose_ = move_group_.getCurrentPose();
	ROS_INFO_STREAM(last_pose_);

	res.pose = last_pose_.pose;

	async_spinner.stop();
	return true;
}