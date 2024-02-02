#include <yk_interface.h>

YK_Interface::YK_Interface(std::string group_name, ros::NodeHandle &nh_)
	: move_group_(group_name)
{
	ROS_INFO_STREAM("planning frame.." << move_group_.getPlanningFrame());
	ROS_INFO_STREAM("pose reference frame.." << move_group_.getPoseReferenceFrame());
	ROS_INFO_STREAM("End Effector.." << move_group_.getEndEffector());
	ROS_INFO_STREAM("End Effector Link.." << move_group_.getEndEffectorLink());

	max_velocity_scaling_factor_ = 0.1;
	max_acceleration_scaling_factor_ = 0.3;
	move_group_.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
	move_group_.setMaxVelocityScalingFactor(max_velocity_scaling_factor_);

	// SERVICE SERVERS
	setPose_server_ = nh_.advertiseService("yk_set_pose", &YK_Interface::setPose, this);
	getPose_server_ = nh_.advertiseService("yk_get_pose", &YK_Interface::getPose, this);
}

bool YK_Interface::setPose(yk_tasks::SetPose::Request &req, yk_tasks::SetPose::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!";
	std::cout << "Moving to: " << req.pose;
	// moveit::planning_interface::MoveGroupInterface move_group."manipulator"); //move_group name to be changes

	//******************************************************************
	/**
	 * Approach 1: Using plan_kinematics_path service
	 * ISSUE: Planning errors due to exceeding vel/acc joint limits
	 *
	moveit_msgs::MotionPlanRequest mp_req;
	moveit_msgs::MotionPlanResponse mp_res;

	mp_req.pipeline_id = "pilz_industrial_motion_planner";
	mp_req.planner_id = "LIN";
	mp_req.group_name = "manipulator";
	mp_req.num_planning_attempts = 5;
	mp_req.max_velocity_scaling_factor = 0.3;
	mp_req.max_acceleration_scaling_factor = 0.3;

	float pos_tolerance = 0.001;
	float orient_tolerance = 0.01;
	geometry_msgs::PoseStamped target_pose_stamped;
	target_pose_stamped.header.frame_id = req.base_frame;
	target_pose_stamped.pose = req.pose;
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), target_pose_stamped, pos_tolerance, orient_tolerance);
	mp_req.goal_constraints.push_back(pose_goal);

	ros::ServiceClient plan_kinematics_path_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
	plan_kinematics_path_client.waitForExistence();
	moveit_msgs::GetMotionPlan srv;
	srv.request.motion_plan_request = mp_req;

	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;

	while (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		if (plan_kinematics_path_client.call(srv))
		{
			mp_res = srv.response.motion_plan_response;
			if (mp_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				std::cerr << "Planning failed due to ERROR CODE = " << mp_res.error_code.val << std::endl;
			}
			else
			{
				std::cout << "Planning successful\n";
				moveit::planning_interface::MoveGroupInterface::Plan plan;
				plan.trajectory_ = mp_res.trajectory;
				ret_val = move_group_.execute(plan);
			}
		}
		else
		{
			ROS_ERROR("Failed to call service plan_kinematics_path");
			return false;
		}
	}
	*/	

	/**
	 * Approach 2: using setPlannerId to use PILZ LIN planner
	 * ERROR: "Cannot find planning configuration for group 'manipulator' using planner 'LIN'. Will use defaults instead."
	 * ISSUE:  ompl_planning.yaml file needs to have "LIN" planner config. PILZ is not part of OMPL.
	/*
	move_group_.setPoseTarget(req.pose);
	move_group_.setPoseReferenceFrame(req.base_frame);
	move_group_.setPlannerId("LIN");
	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		move_group_.setStartStateToCurrentState();
		ret_val = move_group_.move();
	}
	*/

	// Approach 3: using computeCartesianPath
	/**
	 * ERROR: Aborts with TIMEOUT. "Validation failed: Missing valid timestamp data for trajectory pt 1"
	geometry_msgs::Pose start_pose = move_group_.getCurrentPose().pose;
	geometry_msgs::Pose target_pose = req.pose;
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(start_pose);
	waypoints.push_back(target_pose);

	moveit_msgs::RobotTrajectory trajectory;
	double fraction = move_group_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

	if (fraction > 0.95)
	{
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;
		move_group_.execute(plan);
	}
	else
	{
		ROS_ERROR("Failed to compute cartesian path");
		return false;
	}
	*/

	/**
	 * Approach 4
	 * ISSUE: Joint space trajectory. Does weird trajectory sometimes.
	 */
	move_group_.setPoseReferenceFrame(req.base_frame);
	std::cout<<"Planning Frame.." << move_group_.getPlanningFrame();

	move_group_.setPoseTarget(req.pose);
	std::cout<<"TargetPose set";

	moveit_msgs::MoveItErrorCodes ret_val = move_group_.move();
	// return value is of type MoveitErrorCodes message

	std::cout<<"Return Value"<<ret_val;

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		ret_val = move_group_.move();

	}
	
	//******************************************************************

	last_pose_ = move_group_.getCurrentPose();
	std::cout << last_pose_;

	res.pose = last_pose_.pose;
	async_spinner.stop();

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::cerr << "Move failed due to ERROR CODE=" << ret_val;
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
