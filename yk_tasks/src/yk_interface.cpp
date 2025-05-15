#include <yk_interface.h>

YK_Interface::YK_Interface(std::string group_name, ros::NodeHandle &nh_)
	: move_group_(group_name),
	go_to_pose_as_(nh_, "yk_go_to_pose", boost::bind(&YK_Interface::goToPoseCallback, this, _1), false),
	go_to_pose_async_as_(nh_, "yk_go_to_pose_async", boost::bind(&YK_Interface::goToPoseAsyncCallback, this, _1), false),
	go_to_joints_as_(nh_, "yk_go_to_joints", boost::bind(&YK_Interface::goToJointsCallback, this, _1), false),
	execute_cartesian_trajectory_as_(nh_, "yk_execute_cartesian_trajectory", boost::bind(&YK_Interface::executeCartesianTrajectoryCallback, this, _1), false),
	execute_cartesian_trajectory_ft_as_(nh_, "yk_execute_cartesian_trajectory_ft", boost::bind(&YK_Interface::executeCartesianTrajectoryFTCallback, this, _1), false)
{
	ROS_INFO_STREAM("planning frame.." << move_group_.getPlanningFrame());
	ROS_INFO_STREAM("pose reference frame.." << move_group_.getPoseReferenceFrame());
	ROS_INFO_STREAM("End Effector.." << move_group_.getEndEffector());
	ROS_INFO_STREAM("End Effector Link.." << move_group_.getEndEffectorLink());

	max_velocity_scaling_factor_ = 0.1;
	max_acceleration_scaling_factor_ = 0.1;
	move_group_.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
	move_group_.setMaxVelocityScalingFactor(max_velocity_scaling_factor_);

	//START ACTION SERVERS
	go_to_pose_as_.start();
	go_to_pose_async_as_.start();
	go_to_joints_as_.start();
	execute_cartesian_trajectory_as_.start();
	execute_cartesian_trajectory_ft_as_.start();
	
	//ROSTOPIC SUBSCRIBERS
	fts_sub_ = nh_.subscribe("fts", 1, &YK_Interface::ftsCallback, this);
	// robot_status_sub_ = nh_.subscribe("robot_status", 1, &YK_Interface::checkMoving, this);
	joint_state_sub_ = nh_.subscribe("joint_states", 1, &YK_Interface::checkMoving, this);
	trajectory_status_sub_ = nh_.subscribe("execute_trajectory/status", 1, &YK_Interface::checkTrajStatus, this);

	//SERVICE SERVERS
	getPose_server_ = nh_.advertiseService("yk_get_pose", &YK_Interface::getPose, this);
	getPoseStamped_server_ = nh_.advertiseService("yk_get_pose_stamped", &YK_Interface::getPoseStamped, this);
	setJoints_server_ = nh_.advertiseService("yk_set_joints", &YK_Interface::setJoints, this);
	setPose_server_ = nh_.advertiseService("yk_set_pose", &YK_Interface::setPose, this);
	setPoseFT_server_ = nh_.advertiseService("yk_set_pose_ft", &YK_Interface::setPoseFT, this);
	executeTrajectory_server_ = nh_.advertiseService("yk_execute_trajectory", &YK_Interface::executeTrajectory, this);
	stopTrajectory_server_ = nh_.advertiseService("yk_stop_trajectory", &YK_Interface::stopTrajectory, this);
	executeCartesianTraj_server_ = nh_.advertiseService("yk_execute_cartesian_trajectory", &YK_Interface::executeCartesianTrajectory, this);
	executeCartesianTrajAsync_server_ = nh_.advertiseService("yk_execute_cartesian_trajectory_async", &YK_Interface::executeCartesianTrajectory, this);
}

bool YK_Interface::setPose(yk_msgs::SetPose::Request &req, yk_msgs::SetPose::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!";
	std::cout << "New Move to: " << req.pose;
	// moveit::planning_interface::MoveGroupInterface move_group."manipulator"); //move_group name to be changes

	//******************************************************************
	/**
	 * Approach 1: Using plan_kinematics_path service
	 * ISSUE: Planning errors due to exceeding vel/acc joint limits
	 */
	moveit_msgs::MotionPlanRequest mp_req;
	moveit_msgs::MotionPlanResponse mp_res;

	mp_req.pipeline_id = "pilz_industrial_motion_planner";
	mp_req.planner_id = req.traj_type.compare("PTP") == 0 ? "PTP" : "LIN";
	mp_req.group_name = "manipulator";
	mp_req.num_planning_attempts = 5;
	if (req.max_velocity_scaling_factor == 0.0)
	{
		mp_req.max_velocity_scaling_factor = 0.3;
	}
	else
	{
		mp_req.max_velocity_scaling_factor = req.max_velocity_scaling_factor;
	}
	if (req.max_acceleration_scaling_factor == 0.0)
	{
		mp_req.max_acceleration_scaling_factor = 0.3;
	}
	else
	{
		mp_req.max_acceleration_scaling_factor = req.max_acceleration_scaling_factor;
	}

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

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		if (plan_kinematics_path_client.call(srv))
		{
			mp_res = srv.response.motion_plan_response;
			if (mp_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				std::cerr << "Planning failed due to ERROR CODE = " << mp_res.error_code.val << std::endl;
				ret_val.val = mp_res.error_code.val;
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
	 *
	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;

	while(ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
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
			ret_val = move_group_.execute(plan);
		}
		else
		{
			ROS_ERROR("Failed to compute cartesian path");
			return false;
		}
	}
	*/
	/**
	 * Approach 4
	 * ISSUE: Joint space trajectory. Does weird trajectory sometimes.
	 *
	move_group_.setPlannerId("BiTRRT");
	std::cout<<"\nPrinting Planner Params\n";
	std::map<std::string,std::string> planner_params = move_group_.getPlannerParams("BiTRRT");
	for (const auto& pair : planner_params) {
        std::cout << "Key: " << pair.first << ", Value: " << pair.second << std::endl;
    }
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
	*/
	//******************************************************************

	res.pose = move_group_.getCurrentPose().pose;
	std::cout << res.pose;

	async_spinner.stop();

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::cerr << "Move failed due to ERROR CODE=" << ret_val;
		return false;
	}

	return true;
}

bool YK_Interface::setJoints(yk_msgs::SetJoints::Request &req, yk_msgs::SetJoints::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!";
	std::cout << "New Move to: " << req.state;

	/**
	 * Approach 4
	 * ISSUE: Joint space trajectory. Does weird trajectory sometimes.
	 */
	move_group_.setPlannerId("BiTRRT");
	std::cout<<"\nPrinting Planner Params\n";

	move_group_.setJointValueTarget(req.state);
	std::cout<<"Target Joints set";

	moveit_msgs::MoveItErrorCodes ret_val = move_group_.move();
	// return value is of type MoveitErrorCodes message

	std::cout<<"Return Value"<<ret_val;

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		ret_val = move_group_.move();

	}
	
	//******************************************************************

	last_joints_.name = move_group_.getJoints();
	last_joints_.position = move_group_.getCurrentJointValues();

	res.state = last_joints_;
	async_spinner.stop();

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::cerr << "Move failed due to ERROR CODE=" << ret_val;
		return false;
	}

	return true;
}


bool YK_Interface::getPose(yk_msgs::GetPose::Request &req,
						   yk_msgs::GetPose::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	res.pose = move_group_.getCurrentPose().pose;

	async_spinner.stop();
	return true;
}

bool YK_Interface::getPoseStamped(yk_msgs::GetPoseStamped::Request &req,
						   		  yk_msgs::GetPoseStamped::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	res.pose = move_group_.getCurrentPose();

	async_spinner.stop();
	return true;
}

bool YK_Interface::executeTrajectory(moveit_msgs::ExecuteKnownTrajectory::Request &req, 
									 moveit_msgs::ExecuteKnownTrajectory::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	move_group_.stop();

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = req.trajectory;

	moveit_msgs::MoveItErrorCodes ret_val = move_group_.execute(plan);

	res.error_code = ret_val;
	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::cerr << "Move failed due to ERROR CODE=" << res.error_code;
		return true;
	}

	async_spinner.stop();
	return true;
}

bool YK_Interface::stopTrajectory(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	move_group_.stop();
	res.success = true;
	res.message = "Trajectory stopped";

	async_spinner.stop();
	return true;
}


void YK_Interface::goToPoseCallback(const yk_msgs::GoToPoseGoalConstPtr &goal)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!";
	std::cout << "New Move to: " << goal->pose;
	// moveit::planning_interface::MoveGroupInterface move_group."manipulator"); //move_group name to be changes

	//******************************************************************
	/**
	 * Approach 1: Using plan_kinematics_path service
	 * ISSUE: Planning errors due to exceeding vel/acc joint limits
	 */
	moveit_msgs::MotionPlanRequest mp_req;
	moveit_msgs::MotionPlanResponse mp_res;

	mp_req.pipeline_id = "pilz_industrial_motion_planner";
	mp_req.planner_id = goal->traj_type.compare("PTP") == 0 ? "PTP" : "LIN";
	mp_req.group_name = "manipulator";
	mp_req.num_planning_attempts = 5;
	if (goal->max_velocity_scaling_factor == 0.0)
	{
		mp_req.max_velocity_scaling_factor = 0.3;
	}
	else
	{
		mp_req.max_velocity_scaling_factor = goal->max_velocity_scaling_factor;
	}
	if (goal->max_acceleration_scaling_factor == 0.0)
	{
		mp_req.max_acceleration_scaling_factor = 0.3;
	}
	else
	{
		mp_req.max_acceleration_scaling_factor = goal->max_acceleration_scaling_factor;
	}

	float pos_tolerance = 0.001;
	float orient_tolerance = 0.01;
	geometry_msgs::PoseStamped target_pose_stamped;
	target_pose_stamped.header.frame_id = goal->base_frame;
	target_pose_stamped.pose = goal->pose;
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), target_pose_stamped, pos_tolerance, orient_tolerance);
	mp_req.goal_constraints.push_back(pose_goal);

	ros::ServiceClient plan_kinematics_path_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
	plan_kinematics_path_client.waitForExistence();
	moveit_msgs::GetMotionPlan srv;
	srv.request.motion_plan_request = mp_req;

	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		if (plan_kinematics_path_client.call(srv))
		{
			mp_res = srv.response.motion_plan_response;
			if (mp_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				std::cerr << "Planning failed due to ERROR CODE = " << mp_res.error_code.val << std::endl;
				go_to_pose_feedback_.feedback = "Planning failed";
				go_to_pose_as_.publishFeedback(go_to_pose_feedback_);
				ret_val.val = mp_res.error_code.val;
			}
			else
			{
				std::cout << "Planning successful\n";
				go_to_pose_feedback_.feedback = "Planning Successful";
				go_to_pose_as_.publishFeedback(go_to_pose_feedback_);
				moveit::planning_interface::MoveGroupInterface::Plan plan;
				plan.trajectory_ = mp_res.trajectory;
				ret_val = move_group_.execute(plan);
			}
		}
		else
		{
			go_to_pose_result_.pose = move_group_.getCurrentPose().pose;
			ROS_ERROR("Failed to call service plan_kinematics_path");
			go_to_pose_feedback_.feedback = "Failed to call service plan_kinematics_path";
			go_to_pose_as_.publishFeedback(go_to_pose_feedback_);
			go_to_pose_as_.setAborted(go_to_pose_result_);
			return;
		}
	}	

	go_to_pose_result_.pose = move_group_.getCurrentPose().pose;	

	std::cout << go_to_pose_result_.pose;

	async_spinner.stop();
	// set the action state to succeeded

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::cerr << "Move failed due to ERROR CODE=" << ret_val;
		go_to_pose_feedback_.feedback = "Move failed";
		go_to_pose_as_.publishFeedback(go_to_pose_feedback_);
		go_to_pose_as_.setAborted(go_to_pose_result_);
		return;
	}

	go_to_pose_as_.setSucceeded(go_to_pose_result_);

	return;
}


void YK_Interface::goToPoseAsyncCallback(const yk_msgs::GoToPoseGoalConstPtr &goal)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!";
	std::cout << "New Move to: " << goal->pose;
	// moveit::planning_interface::MoveGroupInterface move_group."manipulator"); //move_group name to be changes

	//******************************************************************
	/**
	 * Approach 1: Using plan_kinematics_path service
	 * ISSUE: Planning errors due to exceeding vel/acc joint limits
	 */
	moveit_msgs::MotionPlanRequest mp_req;
	moveit_msgs::MotionPlanResponse mp_res;

	mp_req.pipeline_id = "pilz_industrial_motion_planner";
	mp_req.planner_id = goal->traj_type.compare("PTP") == 0 ? "PTP" : "LIN";
	mp_req.group_name = "manipulator";
	mp_req.num_planning_attempts = 5;
	if (goal->max_velocity_scaling_factor == 0.0)
	{
		mp_req.max_velocity_scaling_factor = 0.3;
	}
	else
	{
		mp_req.max_velocity_scaling_factor = goal->max_velocity_scaling_factor;
	}
	if (goal->max_acceleration_scaling_factor == 0.0)
	{
		mp_req.max_acceleration_scaling_factor = 0.3;
	}
	else
	{
		mp_req.max_acceleration_scaling_factor = goal->max_acceleration_scaling_factor;
	}

	float pos_tolerance = 0.001;
	float orient_tolerance = 0.01;
	geometry_msgs::PoseStamped target_pose_stamped;
	target_pose_stamped.header.frame_id = goal->base_frame;
	target_pose_stamped.pose = goal->pose;
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), target_pose_stamped, pos_tolerance, orient_tolerance);
	mp_req.goal_constraints.push_back(pose_goal);

	ros::ServiceClient plan_kinematics_path_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
	plan_kinematics_path_client.waitForExistence();
	moveit_msgs::GetMotionPlan srv;
	srv.request.motion_plan_request = mp_req;

	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		if (plan_kinematics_path_client.call(srv))
		{
			mp_res = srv.response.motion_plan_response;
			if (mp_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				std::cerr << "Planning failed due to ERROR CODE = " << mp_res.error_code.val << std::endl;
				go_to_pose_async_feedback_.feedback = "Planning failed";
				go_to_pose_async_as_.publishFeedback(go_to_pose_async_feedback_);
				ret_val.val = mp_res.error_code.val;
			}
			else
			{
				std::cout << "Planning successful\n";
				go_to_pose_async_feedback_.feedback = "Planning Successful";
				go_to_pose_async_as_.publishFeedback(go_to_pose_async_feedback_);
				moveit::planning_interface::MoveGroupInterface::Plan plan;
				plan.trajectory_ = mp_res.trajectory;
				ret_val = move_group_.asyncExecute(plan);
			}
		}
		else
		{
			go_to_pose_async_result_.pose = move_group_.getCurrentPose().pose;
			ROS_ERROR("Failed to call service plan_kinematics_path");
			go_to_pose_async_feedback_.feedback = "Failed to call service plan_kinematics_path";
			go_to_pose_async_as_.publishFeedback(go_to_pose_async_feedback_);
			go_to_pose_async_as_.setAborted(go_to_pose_async_result_);
			return;
		}
	}	

	go_to_pose_async_result_.pose = move_group_.getCurrentPose().pose;
	std::cout << go_to_pose_async_result_.pose ;

	async_spinner.stop();
	// set the action state to succeeded

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::cerr << "Move failed due to ERROR CODE=" << ret_val;
		go_to_pose_async_feedback_.feedback = "Move failed";
		go_to_pose_async_as_.publishFeedback(go_to_pose_async_feedback_);
		go_to_pose_async_as_.setAborted(go_to_pose_async_result_);
		return;
	}

	go_to_pose_async_as_.setSucceeded(go_to_pose_async_result_);

	return;
}


void YK_Interface::goToJointsCallback(const yk_msgs::GoToJointsGoalConstPtr &goal)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!";
	std::cout << "New Move to: " << goal->state;

	/**
	 * Approach 4
	 * ISSUE: Joint space trajectory. Does weird trajectory sometimes.
	 */
	move_group_.setPlannerId("BiTRRT");
	std::cout<<"\nPrinting Planner Params\n";

	move_group_.setMaxAccelerationScalingFactor(goal->max_acceleration_scaling_factor);
	move_group_.setMaxVelocityScalingFactor(goal->max_velocity_scaling_factor);

	go_to_joints_feedback_.feedback = "Printing Planner Params";
	go_to_joints_as_.publishFeedback(go_to_joints_feedback_);

	move_group_.setJointValueTarget(goal->state);
	std::cout<<"Target Joints set";
	go_to_joints_feedback_.feedback = "Target Joints set";
	go_to_joints_as_.publishFeedback(go_to_joints_feedback_);

	moveit_msgs::MoveItErrorCodes ret_val = move_group_.move();
	// return value is of type MoveitErrorCodes message

	std::cout<<"Return Value"<<ret_val;

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		ret_val = move_group_.move();
	}
	
	//******************************************************************

	last_joints_.name = move_group_.getJoints();
	last_joints_.position = move_group_.getCurrentJointValues();

	go_to_joints_result_.state = last_joints_;
	async_spinner.stop();

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		std::cerr << "Move failed due to ERROR CODE=" << ret_val;
		go_to_joints_feedback_.feedback = "Move failed";
		go_to_joints_as_.publishFeedback(go_to_joints_feedback_);
		go_to_joints_as_.setAborted(go_to_joints_result_);
		return;
	}

	go_to_joints_as_.setSucceeded(go_to_joints_result_);
	return;
}

void YK_Interface::executeCartesianTrajectoryCallback(const yk_msgs::ExecuteCartesianTrajectoryGoalConstPtr &goal)
{
	// ros::AsyncSpinner async_spinner(1);
	// async_spinner.start();

	std::cout << "Let's Move!!\n";
	std::cout << "New Trajectory to End Waypoint: " << goal->waypoints[1] << std::endl;
	std::cout << "Number of waypoints: " << goal->waypoints.size();

	execute_cartesian_trajectory_feedback_.feedback = "Planning Cartesian Trajectory";
	execute_cartesian_trajectory_feedback_.wp_pct = 0.0;
	execute_cartesian_trajectory_feedback_.wp_id = 0;
	execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
	
	//******************************************************************
	/*
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	// optional arg: avoid_collision: bool
	// optional arg: planning_options: moveit::planning_interface::MoveGroupInterface::PlanningOptions
	double fraction = move_group_.computeCartesianPath(goal->waypoints,
													   goal->eef_step,
													   goal->jump_threshold,
													   plan.trajectory_);
	moveit_msgs::MoveItErrorCodes ret_val;

	if (fraction < 0.99) {
		std::cerr << "Planning failed due to ???. Fraction of planning successful = " << fraction  * 100.0 << std::endl;
		execute_cartesian_trajectory_feedback_.feedback = "Planning failed";
		execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
		ret_val.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
	} else {
		std::cout << "Planning successful\n";
		execute_cartesian_trajectory_feedback_.feedback = "Planning Successful";
		execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
		ret_val = move_group_.asyncExecute(plan);
	}
	*/
	
	planAndExecuteCartesianPath_(goal->waypoints, goal->eef_step, goal->jump_threshold, 
								goal->max_velocity_scaling_factor, goal->max_acceleration_scaling_factor,
								goal->blend_radius, true);
	//******************************************************************

	while(execute_cartesian_trajectory_feedback_.wp_id < goal->waypoints.size()-1)
	{
		if(execute_cartesian_trajectory_as_.isPreemptRequested() || !ros::ok())
		{
			execute_cartesian_trajectory_feedback_.feedback = "Preempted";
			execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
			execute_cartesian_trajectory_as_.setPreempted(execute_cartesian_trajectory_result_);
			return;
		}

		int next_wp_id = execute_cartesian_trajectory_feedback_.wp_id + 1;
		geometry_msgs::Pose next_wp = goal->waypoints[next_wp_id];

		double position_error, orientation_error;
		calculatePoseError_(move_group_.getCurrentPose().pose, next_wp, position_error, orientation_error);

		float pos_tolerance = 0.001;
		float orient_tolerance = 0.01;

		if (position_error < pos_tolerance || orientation_error < orient_tolerance)
		{
			execute_cartesian_trajectory_feedback_.feedback = "Waypoint reached: " + std::to_string(next_wp_id);
			execute_cartesian_trajectory_feedback_.wp_id = next_wp_id;
		}
		
		execute_cartesian_trajectory_feedback_.wp_pct = 
			(float)execute_cartesian_trajectory_feedback_.wp_id/(float)goal->waypoints.size();
	
		execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
	}

	// set the action state to succeeded
	execute_cartesian_trajectory_feedback_.feedback = "Trajectory execution completed";
	std::cout << execute_cartesian_trajectory_feedback_.feedback << std::endl;
	execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
	
	execute_cartesian_trajectory_result_.pose = move_group_.getCurrentPose().pose;
	execute_cartesian_trajectory_as_.setSucceeded(execute_cartesian_trajectory_result_);
}

bool YK_Interface::executeCartesianTrajectory(yk_msgs::ExecuteCartesianTrajectory::Request &req, 
											  yk_msgs::ExecuteCartesianTrajectory::Response &res)
{

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!\n";
	std::cout << "Number of waypoints: " << req.poses.size() << std::endl;

	// Approach 1: Use computeCartesianPath
	// Issue: Robot doesn't follow the trajectory, and instead moves in a straight line to the last waypoint.
	/*
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	// optional arg: avoid_collision: bool
	// optional arg: planning_options: moveit::planning_interface::MoveGroupInterface::PlanningOptions
	double fraction = move_group_.computeCartesianPath(req.poses,
													   req.eef_step,
													   req.jump_threshold,
													   plan.trajectory_);
													   
	bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
	moveit_msgs::MoveItErrorCodes ret_val;

	if (success) {
		std::cerr << "Planning failed due to ???. Fraction of planning successful = " << fraction  * 100.0 << std::endl;
		ret_val.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
	} else {
		std::cout << "Planning successful\n";
		ret_val = move_group_.execute(plan);
	}
	*/

	// Approach 2: Use plan_sequence_path service
	// Issue: Robot doesn't follow the trajectory, and instead moves in a straight line to the last waypoint.
	
	moveit_msgs::MotionPlanRequest mp_req_base;
	moveit_msgs::MotionSequenceRequest ms_req;
	moveit_msgs::MotionSequenceResponse ms_res;

	mp_req_base.pipeline_id = "pilz_industrial_motion_planner";
	mp_req_base.planner_id = "LIN";
	mp_req_base.group_name = "manipulator";
	mp_req_base.num_planning_attempts = 5;
	mp_req_base.max_velocity_scaling_factor = req.max_velocity_scaling_factor == 0.0 ? 0.3 : req.max_velocity_scaling_factor;
	mp_req_base.max_acceleration_scaling_factor = req.max_acceleration_scaling_factor == 0.0 ? 0.3 : req.max_acceleration_scaling_factor;

	float pos_tolerance = 0.001;
	float orient_tolerance = 0.01;
	float blend_radius = req.blend_radius;

	moveit::core::RobotStatePtr robot_state_ptr(move_group_.getCurrentState());
	const robot_state::JointModelGroup *robot_joint_group_ptr = robot_state_ptr->getJointModelGroup("manipulator");
	std::vector<geometry_msgs::Pose> waypoints = req.poses;

	for (int i = 0; i < waypoints.size(); i++)
	{
		moveit_msgs::MotionSequenceItem item;
		item.blend_radius = blend_radius;
   
		if (i == waypoints.size() - 1)
			 item.blend_radius = 0.0;

		item.req = mp_req_base;
		geometry_msgs::PoseStamped waypoint_pose;
		waypoint_pose.header.frame_id = "base_link";
		waypoint_pose.pose = waypoints[i];
		moveit_msgs::Constraints goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), waypoint_pose, pos_tolerance, orient_tolerance);
		item.req.goal_constraints.push_back(goal);
   
		if (i != waypoints.size() - 1)
			 item.req.start_state = moveit_msgs::RobotState();
   
		ms_req.items.push_back(item);
	}

	std::cout<<"Number of waypoints: "<<ms_req.items.size()<<std::endl;
	ros::ServiceClient plan_sequence_path_client = nh_.serviceClient<moveit_msgs::GetMotionSequence>("plan_sequence_path");
	plan_sequence_path_client.waitForExistence();
	moveit_msgs::GetMotionSequence srv;
	srv.request.request = ms_req;

	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
	
	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		if (plan_sequence_path_client.call(srv))
		{
			ms_res = srv.response.response;
			if (ms_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				std::cerr << "Planning failed due to ERROR CODE = " << ms_res.error_code.val << std::endl;
				ret_val.val = ms_res.error_code.val;
			}
			else
			{
				std::cout << "Planning successful\n";
				std::vector<moveit_msgs::RobotTrajectory> trajectories = ms_res.planned_trajectories;
				std::cout<<"Number of planned trajectories: "<<trajectories.size()<<std::endl;
				for (int i = 0; i < trajectories.size(); i++)
				{
					moveit::planning_interface::MoveGroupInterface::Plan plan;
					plan.trajectory_ = trajectories[i];
					ret_val = move_group_.execute(plan);
					if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
					{
						std::cerr << "Move failed due to ERROR CODE=" << ret_val;
						return false;
					}
				}
			}
		}
		else
		{
			ROS_ERROR("Failed to call service plan_sequence_path");
			return false;
		}
	}		
	

	// Approach 3: Using plan_kinematics_path service sequentially
	// Issue: Start position of next trajectory is not always the end position of the previous trajectory.
	/*
	std::vector<moveit_msgs::RobotTrajectory> trajectories;
	std::vector<geometry_msgs::Pose> waypoints = req.poses;
	moveit::core::RobotStatePtr robot_state_ptr(move_group_.getCurrentState());
	const robot_state::JointModelGroup *robot_joint_group_ptr = robot_state_ptr->getJointModelGroup("manipulator");

	ros::ServiceClient plan_kinematics_path_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
	plan_kinematics_path_client.waitForExistence();
	moveit_msgs::GetMotionPlan srv;
	
	moveit_msgs::MoveItErrorCodes ret_val;

	for( int i = 0; i < waypoints.size(); i++)
	{
		moveit_msgs::MotionPlanRequest mp_req;
		moveit_msgs::MotionPlanResponse mp_res;

		mp_req.pipeline_id = "pilz_industrial_motion_planner";
		mp_req.planner_id = "LIN";
		mp_req.group_name = "manipulator";

		robot_state_ptr->setFromIK(robot_joint_group_ptr, waypoints[i-1]);
		robot_state_ptr->update();
		mp_req.start_state.joint_state.name = robot_state_ptr->getVariableNames();
		double* positions = robot_state_ptr->getVariablePositions();
		mp_req.start_state.joint_state.position.assign(positions, positions + robot_state_ptr->getVariableCount());
		// moveit::core::robotStateToRobotStateMsg(*robot_state_ptr, mp_req.start_state);

		mp_req.num_planning_attempts = 5;
		mp_req.max_velocity_scaling_factor = req.max_velocity_scaling_factor == 0.0 ? 0.3 : req.max_velocity_scaling_factor;
		mp_req.max_acceleration_scaling_factor = req.max_acceleration_scaling_factor == 0.0 ? 0.3 : req.max_acceleration_scaling_factor;

		float pos_tolerance = 0.001;
		float orient_tolerance = 0.01;
		geometry_msgs::PoseStamped target_pose_stamped;
		target_pose_stamped.header.frame_id = "base_link";
		target_pose_stamped.pose = waypoints[i];
		moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), target_pose_stamped, pos_tolerance, orient_tolerance);
		mp_req.goal_constraints.push_back(pose_goal);

		srv.request.motion_plan_request = mp_req;
		plan_kinematics_path_client.call(srv);
		mp_res = srv.response.motion_plan_response;

		if (mp_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			ret_val.val = mp_res.error_code.val;
			std::cerr << "Planning failed due to ERROR CODE = " << mp_res.error_code.val << std::endl;
			return false;
		}
		else
		{
			std::cout << "Planning successful\n";
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = mp_res.trajectory;
			trajectories.push_back(mp_res.trajectory);
		}
	}

	for (int i=0; i<trajectories.size(); i++)
	{
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectories[i];
		ret_val = move_group_.execute(plan);
		std::cout<<"Trajectory "<<i<<" executed\n";		// std::cout<<"Current Pose: \n"<<move_group_.getCurrentPose().pose.position<<std::endl;
		if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			std::cerr << "Move failed due to ERROR CODE=" << ret_val;
			return false;
		}
	}
	*/

	res.pose = move_group_.getCurrentPose().pose;
	std::cout << res.pose;

	async_spinner.stop();

	return true;	

}

bool YK_Interface::executeCartesianTrajectoryAsync(yk_msgs::ExecuteCartesianTrajectory::Request &req, 
											  yk_msgs::ExecuteCartesianTrajectory::Response &res)
{

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!\n";
	std::cout << "Number of waypoints: " << req.poses.size() << std::endl;

	// Approach 1: Use computeCartesianPath
	// Issue: Robot doesn't follow the trajectory, and instead moves in a straight line to the last waypoint.
	/*
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	// optional arg: avoid_collision: bool
	// optional arg: planning_options: moveit::planning_interface::MoveGroupInterface::PlanningOptions
	double fraction = move_group_.computeCartesianPath(req.poses,
													   req.eef_step,
													   req.jump_threshold,
													   plan.trajectory_);
													   
	bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
	moveit_msgs::MoveItErrorCodes ret_val;

	if (success) {
		std::cerr << "Planning failed due to ???. Fraction of planning successful = " << fraction  * 100.0 << std::endl;
		ret_val.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
	} else {
		std::cout << "Planning successful\n";
		ret_val = move_group_.execute(plan);
	}
	*/

	// Approach 2: Use plan_sequence_path service
	// Issue: Robot doesn't follow the trajectory, and instead moves in a straight line to the last waypoint.
	
	moveit_msgs::MotionPlanRequest mp_req_base;
	moveit_msgs::MotionSequenceRequest ms_req;
	moveit_msgs::MotionSequenceResponse ms_res;

	mp_req_base.pipeline_id = "pilz_industrial_motion_planner";
	mp_req_base.planner_id = "LIN";
	mp_req_base.group_name = "manipulator";
	mp_req_base.num_planning_attempts = 5;
	mp_req_base.max_velocity_scaling_factor = req.max_velocity_scaling_factor == 0.0 ? 0.3 : req.max_velocity_scaling_factor;
	mp_req_base.max_acceleration_scaling_factor = req.max_acceleration_scaling_factor == 0.0 ? 0.3 : req.max_acceleration_scaling_factor;

	float pos_tolerance = 0.001;
	float orient_tolerance = 0.01;
	float blend_radius = req.blend_radius;

	moveit::core::RobotStatePtr robot_state_ptr(move_group_.getCurrentState());
	const robot_state::JointModelGroup *robot_joint_group_ptr = robot_state_ptr->getJointModelGroup("manipulator");
	std::vector<geometry_msgs::Pose> waypoints = req.poses;

	for (int i = 0; i < waypoints.size(); i++)
	{
		moveit_msgs::MotionSequenceItem item;
		item.blend_radius = blend_radius;
   
		if (i == waypoints.size() - 1)
			 item.blend_radius = 0.0;

		item.req = mp_req_base;
		geometry_msgs::PoseStamped waypoint_pose;
		waypoint_pose.header.frame_id = "base_link";
		waypoint_pose.pose = waypoints[i];
		moveit_msgs::Constraints goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), waypoint_pose, pos_tolerance, orient_tolerance);
		item.req.goal_constraints.push_back(goal);
   
		if (i != waypoints.size() - 1)
			 item.req.start_state = moveit_msgs::RobotState();
   
		ms_req.items.push_back(item);
	}

	std::cout<<"Number of waypoints: "<<ms_req.items.size()<<std::endl;
	ros::ServiceClient plan_sequence_path_client = nh_.serviceClient<moveit_msgs::GetMotionSequence>("plan_sequence_path");
	plan_sequence_path_client.waitForExistence();
	moveit_msgs::GetMotionSequence srv;
	srv.request.request = ms_req;

	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
	
	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		if (plan_sequence_path_client.call(srv))
		{
			ms_res = srv.response.response;
			if (ms_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				std::cerr << "Planning failed due to ERROR CODE = " << ms_res.error_code.val << std::endl;
				ret_val.val = ms_res.error_code.val;
			}
			else
			{
				std::cout << "Planning successful\n";
				std::vector<moveit_msgs::RobotTrajectory> trajectories = ms_res.planned_trajectories;
				std::cout<<"Number of planned trajectories: "<<trajectories.size()<<std::endl;
				if (trajectories.size() == 1)
				{
					moveit::planning_interface::MoveGroupInterface::Plan plan;
					plan.trajectory_ = trajectories[0];
					ret_val = move_group_.asyncExecute(plan);
					if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
					{
						std::cerr << "Move failed due to ERROR CODE=" << ret_val;
						return false;
					}
				}
				else {
					ROS_ERROR_STREAM("Trajectory execution not supported for multiple trajectories. Use yk_execute_cartesian_trajectory service");
					return false;
				}
			}
		}
		else
		{
			ROS_ERROR("Failed to call service plan_sequence_path");
			return false;
		}
	}		
	

	// Approach 3: Using plan_kinematics_path service sequentially
	// Issue: Start position of next trajectory is not always the end position of the previous trajectory.
	/*
	std::vector<moveit_msgs::RobotTrajectory> trajectories;
	std::vector<geometry_msgs::Pose> waypoints = req.poses;
	moveit::core::RobotStatePtr robot_state_ptr(move_group_.getCurrentState());
	const robot_state::JointModelGroup *robot_joint_group_ptr = robot_state_ptr->getJointModelGroup("manipulator");

	ros::ServiceClient plan_kinematics_path_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
	plan_kinematics_path_client.waitForExistence();
	moveit_msgs::GetMotionPlan srv;
	
	moveit_msgs::MoveItErrorCodes ret_val;

	for( int i = 0; i < waypoints.size(); i++)
	{
		moveit_msgs::MotionPlanRequest mp_req;
		moveit_msgs::MotionPlanResponse mp_res;

		mp_req.pipeline_id = "pilz_industrial_motion_planner";
		mp_req.planner_id = "LIN";
		mp_req.group_name = "manipulator";

		robot_state_ptr->setFromIK(robot_joint_group_ptr, waypoints[i-1]);
		robot_state_ptr->update();
		mp_req.start_state.joint_state.name = robot_state_ptr->getVariableNames();
		double* positions = robot_state_ptr->getVariablePositions();
		mp_req.start_state.joint_state.position.assign(positions, positions + robot_state_ptr->getVariableCount());
		// moveit::core::robotStateToRobotStateMsg(*robot_state_ptr, mp_req.start_state);

		mp_req.num_planning_attempts = 5;
		mp_req.max_velocity_scaling_factor = req.max_velocity_scaling_factor == 0.0 ? 0.3 : req.max_velocity_scaling_factor;
		mp_req.max_acceleration_scaling_factor = req.max_acceleration_scaling_factor == 0.0 ? 0.3 : req.max_acceleration_scaling_factor;

		float pos_tolerance = 0.001;
		float orient_tolerance = 0.01;
		geometry_msgs::PoseStamped target_pose_stamped;
		target_pose_stamped.header.frame_id = "base_link";
		target_pose_stamped.pose = waypoints[i];
		moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), target_pose_stamped, pos_tolerance, orient_tolerance);
		mp_req.goal_constraints.push_back(pose_goal);

		srv.request.motion_plan_request = mp_req;
		plan_kinematics_path_client.call(srv);
		mp_res = srv.response.motion_plan_response;

		if (mp_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			ret_val.val = mp_res.error_code.val;
			std::cerr << "Planning failed due to ERROR CODE = " << mp_res.error_code.val << std::endl;
			return false;
		}
		else
		{
			std::cout << "Planning successful\n";
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = mp_res.trajectory;
			trajectories.push_back(mp_res.trajectory);
		}
	}

	for (int i=0; i<trajectories.size(); i++)
	{
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectories[i];
		ret_val = move_group_.execute(plan);
		std::cout<<"Trajectory "<<i<<" executed\n";		// std::cout<<"Current Pose: \n"<<move_group_.getCurrentPose().pose.position<<std::endl;
		if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			std::cerr << "Move failed due to ERROR CODE=" << ret_val;
			return false;
		}
	}
	*/

	res.pose = move_group_.getCurrentPose().pose;
	std::cout << res.pose;

	async_spinner.stop();

	return true;	

}

void YK_Interface::ftsCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
	eef_force_[0] = msg->wrench.force.x;
	eef_force_[1] = msg->wrench.force.y;
	eef_force_[2] = msg->wrench.force.z;

	eef_torque_[0] = msg->wrench.torque.x;
	eef_torque_[1] = msg->wrench.torque.y;
	eef_torque_[2] = msg->wrench.torque.z;
}

bool YK_Interface::setPoseFT(yk_msgs::SetPoseFT::Request &req,
							 yk_msgs::SetPoseFT::Response &res)
{
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	std::cout << "Let's Move!!";
	std::cout << "New Move to: " << req.pose;

	move_group_.stop();
	ros::Duration(1.0).sleep();

	// 1. PLAN TRAJECTORY
	//******************************************************************
	moveit_msgs::MotionPlanRequest mp_req;
	moveit_msgs::MotionPlanResponse mp_res;

	mp_req.pipeline_id = "pilz_industrial_motion_planner";
	mp_req.planner_id = "LIN";
	mp_req.group_name = "manipulator";
	mp_req.num_planning_attempts = 5;
	if (req.max_velocity_scaling_factor == 0.0)
	{
		mp_req.max_velocity_scaling_factor = 0.3;
	}
	else
	{
		mp_req.max_velocity_scaling_factor = req.max_velocity_scaling_factor;
	}
	if (req.max_acceleration_scaling_factor == 0.0)
	{
		mp_req.max_acceleration_scaling_factor = 0.3;
	}
	else
	{
		mp_req.max_acceleration_scaling_factor = req.max_acceleration_scaling_factor;
	}

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
	moveit::planning_interface::MoveGroupInterface::Plan plan;

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
			plan.trajectory_ = mp_res.trajectory;
		}
	}
	else
	{
		ROS_ERROR("Failed to call service plan_kinematics_path");
		return false;
	}

	// 2. FORCE TORQUE THRESHOLDS
	//******************************************************************
	for (int i=0; i<3; i++)
	{
		if (req.use_force[i])
			ROS_INFO_STREAM("Using force threshold for axis " << i << " with value " << req.force_thresholds[i]);			
		if (req.use_torque[i])
			ROS_INFO_STREAM("Using torque threshold for axis " << i << " with value " << req.torque_thresholds[i]);
	}
	ros::spinOnce();
	double start_force[3] = {eef_force_[0], eef_force_[1], eef_force_[2]};
	double start_torque[3] = {eef_torque_[0], eef_torque_[1], eef_torque_[2]};

	// 3. EXECUTE TRAJECTORY
	//******************************************************************
	move_group_.asyncExecute(plan);

	ROS_INFO_STREAM("MOVEMENT STARTED");

	// MONITOR FORCE/TORQUE
	//******************************************************************
	ros::spinOnce();
	bool ft_stop = false;
	bool target_reached = false;
	while(!ft_stop && !target_reached)
	{	
		ros::spinOnce();
		for(int i=0; !ft_stop && i<3; i++)
		{
			if(req.use_force[i] && 
				abs(eef_force_[i]-start_force[i]) > abs(req.force_thresholds[i]))
			{
				ROS_WARN_STREAM("Force threshold exceeded for axis " << i << " with value " << eef_force_[i]);
				move_group_.stop();
				ft_stop = true;
			}
			if(req.use_torque[i] && 
				abs(eef_torque_[i]-start_torque[i]) > abs(req.torque_thresholds[i]))
			{
				ROS_WARN_STREAM("Torque threshold exceeded for axis " << i << " with value " << eef_torque_[i]);
				move_group_.stop();
				ft_stop = true;
			}
		}

		geometry_msgs::Pose current_pose = move_group_.getCurrentPose().pose;
		double position_error, orientation_error;
		calculatePoseError_(current_pose, req.pose, position_error, orientation_error);
		float pos_tolerance = 0.001;
		float orient_tolerance = 0.01;
		if (position_error < pos_tolerance && orientation_error < orient_tolerance)
		{
			ROS_INFO_STREAM("Target pose reached");
			target_reached = true;
		}

	}

	res.is_force_termination = !target_reached;
	res.pose = move_group_.getCurrentPose().pose;
	ROS_INFO_STREAM("Current Pose: " << res.pose);

	for (int i=0; i<3; i++)
	{
		res.force[i] = eef_force_[i]-start_force[i];
		res.torque[i] = eef_torque_[i]-start_torque[i];
	}
	async_spinner.stop();

	return true;

}

void YK_Interface::checkMoving(const sensor_msgs::JointState::ConstPtr &msg)
{
	// ROS_INFO_STREAM("Checking if moving"<<msg->velocity.size());
	int size = msg->velocity.size();
	double velocity_sum=0;
	for (int i=0; i<size; i++)
	{
		velocity_sum += abs(msg->velocity[i]);
	}
	
	if (velocity_sum > 0.05)
	{
		is_moving_ = true;
		// ROS_INFO_STREAM("MOVING");
	}
	else
	{
		is_moving_ = false;
	}
}

/*
void YK_Interface::checkMoving(const industrial_msgs::RobotStatus::ConstPtr &msg)
{
	if (msg->in_motion.val)
	{
		is_moving_ = true;
	}
	else
	{
		is_moving_ = false;
	}
}*/

void YK_Interface::calculatePoseError_(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, 
									   double& position_error, double& orientation_error)
{
	// calculate position error
	position_error = sqrt(pow(pose1.position.x - pose2.position.x, 2) +
						  pow(pose1.position.y - pose2.position.y, 2) +
						  pow(pose1.position.z - pose2.position.z, 2));

	// calculate orientation error

	Eigen::Quaterniond q1 = Eigen::Quaterniond(pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
	Eigen::Quaterniond q2 = Eigen::Quaterniond(pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);
	q1.normalize();
	q2.normalize();

	double dot = q1.dot(q2);

    // Clamp the dot product to avoid domain errors in acos due to numerical issues
    dot = std::max(-1.0, std::min(1.0, dot));

    // Rotational error (angle) in radians
    orientation_error = 2.0 * std::acos(std::abs(dot));
}

bool YK_Interface::planAndExecuteCartesianPath_(
	const std::vector<geometry_msgs::Pose> &waypoints, 
	double eef_step, double jump_threshold, 
	double max_velocity_scaling_factor, double max_acceleration_scaling_factor,
	double blend_radius, bool async)
{

	// Approach 1: Use computeCartesianPath
	// Issue: Robot doesn't follow the trajectory, and instead moves in a straight line to the last waypoint.
	/*
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	// optional arg: avoid_collision: bool
	// optional arg: planning_options: moveit::planning_interface::MoveGroupInterface::PlanningOptions
	double fraction = move_group_.computeCartesianPath(req.poses,
													   req.eef_step,
													   req.jump_threshold,
													   plan.trajectory_);
													   
	bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
	moveit_msgs::MoveItErrorCodes ret_val;

	if (success) {
		std::cerr << "Planning failed due to ???. Fraction of planning successful = " << fraction  * 100.0 << std::endl;
		ret_val.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
	} else {
		std::cout << "Planning successful\n";
		ret_val = move_group_.execute(plan);
	}
	*/

	// Approach 2: Use plan_sequence_path service
	// Issue: Robot doesn't follow the trajectory, and instead moves in a straight line to the last waypoint.
	
	moveit_msgs::MotionPlanRequest mp_req_base;
	moveit_msgs::MotionSequenceRequest ms_req;
	moveit_msgs::MotionSequenceResponse ms_res;

	mp_req_base.pipeline_id = "pilz_industrial_motion_planner";
	mp_req_base.planner_id = "LIN";
	mp_req_base.group_name = "manipulator";
	mp_req_base.num_planning_attempts = 5;
	mp_req_base.max_velocity_scaling_factor = max_velocity_scaling_factor == 0.0 ? 0.3 : max_velocity_scaling_factor;
	mp_req_base.max_acceleration_scaling_factor = max_acceleration_scaling_factor == 0.0 ? 0.3 : max_acceleration_scaling_factor;

	float pos_tolerance = 0.001;
	float orient_tolerance = 0.01;

	moveit::core::RobotStatePtr robot_state_ptr(move_group_.getCurrentState());
	const robot_state::JointModelGroup *robot_joint_group_ptr = robot_state_ptr->getJointModelGroup("manipulator");

	for (int i = 0; i < waypoints.size(); i++)
	{
		moveit_msgs::MotionSequenceItem item;
		item.blend_radius = blend_radius;
   
		if (i == waypoints.size() - 1)
			 item.blend_radius = 0.0;

		item.req = mp_req_base;
		geometry_msgs::PoseStamped waypoint_pose;
		waypoint_pose.header.frame_id = "base_link";
		waypoint_pose.pose = waypoints[i];
		moveit_msgs::Constraints goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), waypoint_pose, pos_tolerance, orient_tolerance);
		item.req.goal_constraints.push_back(goal);
   
		if (i != waypoints.size() - 1)
			 item.req.start_state = moveit_msgs::RobotState();
   
		ms_req.items.push_back(item);
	}

	std::cout<<"Number of waypoints: "<<ms_req.items.size()<<std::endl;
	ros::ServiceClient plan_sequence_path_client = nh_.serviceClient<moveit_msgs::GetMotionSequence>("plan_sequence_path");
	plan_sequence_path_client.waitForExistence();
	moveit_msgs::GetMotionSequence srv;
	srv.request.request = ms_req;

	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
	
	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		if (plan_sequence_path_client.call(srv))
		{
			ms_res = srv.response.response;
			if (ms_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				std::cerr << "Planning failed due to ERROR CODE = " << ms_res.error_code.val << std::endl;
				ret_val.val = ms_res.error_code.val;
			}
			else
			{
				std::cout << "Planning successful\n";
				std::vector<moveit_msgs::RobotTrajectory> trajectories = ms_res.planned_trajectories;
				std::cout<<"Number of planned trajectories: "<<trajectories.size()<<std::endl;
				for (int i = 0; i < trajectories.size(); i++)
				{
					moveit::planning_interface::MoveGroupInterface::Plan plan;
					plan.trajectory_ = trajectories[i];
					ret_val = async ? move_group_.asyncExecute(plan) : move_group_.execute(plan);
					if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
					{
						std::cerr << "Move failed due to ERROR CODE=" << ret_val;
						return false;
					}
				}
			}
		}
		else
		{
			ROS_ERROR("Failed to call service plan_sequence_path");
			return false;
		}
	}		
	

	// Approach 3: Using plan_kinematics_path service sequentially
	// Issue: Start position of next trajectory is not always the end position of the previous trajectory.
	/*
	std::vector<moveit_msgs::RobotTrajectory> trajectories;
	std::vector<geometry_msgs::Pose> waypoints = req.poses;
	moveit::core::RobotStatePtr robot_state_ptr(move_group_.getCurrentState());
	const robot_state::JointModelGroup *robot_joint_group_ptr = robot_state_ptr->getJointModelGroup("manipulator");

	ros::ServiceClient plan_kinematics_path_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
	plan_kinematics_path_client.waitForExistence();
	moveit_msgs::GetMotionPlan srv;
	
	moveit_msgs::MoveItErrorCodes ret_val;

	for( int i = 0; i < waypoints.size(); i++)
	{
		moveit_msgs::MotionPlanRequest mp_req;
		moveit_msgs::MotionPlanResponse mp_res;

		mp_req.pipeline_id = "pilz_industrial_motion_planner";
		mp_req.planner_id = "LIN";
		mp_req.group_name = "manipulator";

		robot_state_ptr->setFromIK(robot_joint_group_ptr, waypoints[i-1]);
		robot_state_ptr->update();
		mp_req.start_state.joint_state.name = robot_state_ptr->getVariableNames();
		double* positions = robot_state_ptr->getVariablePositions();
		mp_req.start_state.joint_state.position.assign(positions, positions + robot_state_ptr->getVariableCount());
		// moveit::core::robotStateToRobotStateMsg(*robot_state_ptr, mp_req.start_state);

		mp_req.num_planning_attempts = 5;
		mp_req.max_velocity_scaling_factor = req.max_velocity_scaling_factor == 0.0 ? 0.3 : req.max_velocity_scaling_factor;
		mp_req.max_acceleration_scaling_factor = req.max_acceleration_scaling_factor == 0.0 ? 0.3 : req.max_acceleration_scaling_factor;

		float pos_tolerance = 0.001;
		float orient_tolerance = 0.01;
		geometry_msgs::PoseStamped target_pose_stamped;
		target_pose_stamped.header.frame_id = "base_link";
		target_pose_stamped.pose = waypoints[i];
		moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), target_pose_stamped, pos_tolerance, orient_tolerance);
		mp_req.goal_constraints.push_back(pose_goal);

		srv.request.motion_plan_request = mp_req;
		plan_kinematics_path_client.call(srv);
		mp_res = srv.response.motion_plan_response;

		if (mp_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			ret_val.val = mp_res.error_code.val;
			std::cerr << "Planning failed due to ERROR CODE = " << mp_res.error_code.val << std::endl;
			return false;
		}
		else
		{
			std::cout << "Planning successful\n";
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = mp_res.trajectory;
			trajectories.push_back(mp_res.trajectory);
		}
	}

	for (int i=0; i<trajectories.size(); i++)
	{
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectories[i];
		ret_val = move_group_.execute(plan);
		std::cout<<"Trajectory "<<i<<" executed\n";		// std::cout<<"Current Pose: \n"<<move_group_.getCurrentPose().pose.position<<std::endl;
		if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			std::cerr << "Move failed due to ERROR CODE=" << ret_val;
			return false;
		}
	}
	*/
	return true;
}

void YK_Interface::executeCartesianTrajectoryFTCallback(const yk_msgs::ExecuteCartesianTrajectoryFTGoalConstPtr &goal)
{
	// ros::AsyncSpinner async_spinner(1);
	// async_spinner.start();

	std::cout << "Let's Move!!\n";
	std::cout << "New Trajectory to End Waypoint: " << goal->waypoints[1] << std::endl;
	std::cout << "Number of waypoints: " << goal->waypoints.size();

	execute_cartesian_trajectory_ft_feedback_.feedback = "Planning Cartesian Trajectory";
	execute_cartesian_trajectory_ft_feedback_.wp_pct = 0.0;
	execute_cartesian_trajectory_ft_feedback_.wp_id = 0;
	execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);
	std::vector<geometry_msgs::Pose> goal_waypoints = goal->waypoints;
	
	planAndExecuteCartesianPath_(goal_waypoints, goal->eef_step, goal->jump_threshold, 
								goal->max_velocity_scaling_factor, goal->max_acceleration_scaling_factor,
								goal->blend_radius, true);

	float pos_tolerance = goal->position_tolerance == 0.0 ? 0.002 : goal->position_tolerance;
	float orient_tolerance = goal->orient_tolerance == 0.0 ? 0.01 : goal->orient_tolerance;
	float force_tolerance = goal->force_tolerance == 0.0 ? 2 : goal->force_tolerance;
	double ft_correction_speed_factor = goal->force_adj_speed_factor == 0.0 ? 0.005 : goal->force_adj_speed_factor;
	float K = 0.001;	
	float timeout_before_planning = 0.5;
	Eigen::Vector3d last_force_diff(0.0, 0.0, 0.0);
	double position_error, orientation_error;
	std::vector<geometry_msgs::Pose> new_waypoints;
	int next_wp_id = 0;
	long aborted_traj_time = ros::Time::now().toSec();
	
	while(execute_cartesian_trajectory_ft_feedback_.wp_id < goal_waypoints.size()-1)
	{
		ros::spinOnce();
		// CHECK IF PREEMPTED
		if(execute_cartesian_trajectory_as_.isPreemptRequested() || !ros::ok())
		{
			move_group_.stop();
			execute_cartesian_trajectory_ft_feedback_.feedback = "Preempted";
			execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);
			execute_cartesian_trajectory_ft_as_.setPreempted(execute_cartesian_trajectory_ft_result_);
			return;
		}

		// CHECK IF TRAJ ABORTED

		if(traj_status_.status != traj_status_.ABORTED)
			aborted_traj_time = ros::Time::now().toSec();
		
		if(ros::Time::now().toSec() - aborted_traj_time > 2)
		{
			ROS_WARN_STREAM("Trajectory aborted. Retrying after"<<ros::Time::now().toSec() - aborted_traj_time <<" seconds");
			execute_cartesian_trajectory_ft_feedback_.feedback = "Trajectory aborted. Retrying.";
			execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);
			
			move_group_.stop();
			ros::Duration(timeout_before_planning).sleep();
			new_waypoints.clear();
			for(int i = next_wp_id; i < goal_waypoints.size(); i++)
			{
				new_waypoints.push_back(goal_waypoints[i]);
			}
			bool ret = false;
			while (!ret)
				ret = planAndExecuteCartesianPath_(new_waypoints, goal->eef_step, goal->jump_threshold, 
									goal->max_velocity_scaling_factor, goal->max_acceleration_scaling_factor,
									goal->blend_radius, true);
			
		}

		next_wp_id = execute_cartesian_trajectory_ft_feedback_.wp_id + 1;
		geometry_msgs::Pose next_wp = goal_waypoints[next_wp_id];	

		calculatePoseError_(move_group_.getCurrentPose().pose, next_wp, position_error, orientation_error);
		execute_cartesian_trajectory_ft_feedback_.position_error = position_error;
		execute_cartesian_trajectory_ft_feedback_.orientation_error = orientation_error;
		execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);

		if (position_error < pos_tolerance && orientation_error < orient_tolerance)
		{
			execute_cartesian_trajectory_ft_feedback_.feedback = "Waypoint reached: " + std::to_string(next_wp_id);
			execute_cartesian_trajectory_ft_feedback_.wp_id = next_wp_id;
		}
		
		execute_cartesian_trajectory_ft_feedback_.wp_pct = 
			(float)execute_cartesian_trajectory_ft_feedback_.wp_id/(float)goal_waypoints.size();

		double force_target[3] = {goal->force_target[0], goal->force_target[1], goal->force_target[2]};
		Eigen::Vector3d force_target_eigen(force_target[0], force_target[1], force_target[2]);
		Eigen::Vector3d eef_force_eigen(eef_force_[0], eef_force_[1], eef_force_[2]);

		//PROJECT EEF FORCE ONTO TARGET FORCE
		Eigen::Vector3d force_target_eigen_unit = force_target_eigen.normalized();

		Eigen::Vector3d eef_force_eigen_proj = eef_force_eigen.dot(force_target_eigen_unit) * force_target_eigen_unit;
		double force_error = abs(eef_force_eigen_proj.norm()  - force_target_eigen.norm());

		execute_cartesian_trajectory_ft_feedback_.force_error = force_error;

		if (force_error > force_tolerance)
		{
			execute_cartesian_trajectory_ft_feedback_.feedback = "Force threshold exceeded";
			execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);
			
			Eigen::Vector3d force_diff = eef_force_eigen_proj - force_target_eigen;
			Eigen::Vector3d force_diff_unit = force_diff.normalized();
		
			std::cout << "===============================" << std::endl;
			std::cout << "Force diff: " << force_diff_unit.transpose() << std::endl;
			std::cout << "===============================" << std::endl;		
			
			// CONVERT THE VECTOR TO BASE LINK FRAME
			geometry_msgs::Pose T_base2eef_ = move_group_.getCurrentPose().pose;
			Eigen::Quaterniond q(T_base2eef_.orientation.w, T_base2eef_.orientation.x, 
								T_base2eef_.orientation.y, T_base2eef_.orientation.z);
			Eigen::Matrix3d R = q.toRotationMatrix();
			std::cout << "Rotation matrix:\n " << R << std::endl;
			Eigen::Vector3d force_diff_base = R * force_diff_unit;
			double force_diff_base_unit[3] = {force_diff_base[0], force_diff_base[1], force_diff_base[2]};
			std::cout << "Force diff base: " << force_diff_base.transpose() << std::endl;
									
			//MOVE UNTIL FORCE ERROR IS BELOW TOLERANCE
			geometry_msgs::Pose start_pose = move_group_.getCurrentPose().pose;

			move_group_.stop();
			//GET 50 POINTS IN FORCE DIFF BASE UNIT DIRECTION
			std::vector<geometry_msgs::Pose> error_waypoints(100);
			for(int i=0; i<100; i++)
			{
				error_waypoints[i] = start_pose;
				error_waypoints[i].position.x += (i+1) * K * force_diff_base_unit[0];
				error_waypoints[i].position.y += (i+1) * K * force_diff_base_unit[1];
				error_waypoints[i].position.z += (i+1) * K * force_diff_base_unit[2];
			}
			execute_cartesian_trajectory_ft_feedback_.feedback = "Adjusting Force";
			ros::Duration(timeout_before_planning).sleep();
			bool ret = false;
			while (!ret)
				ret = planAndExecuteCartesianPath_(error_waypoints, goal->eef_step, goal->jump_threshold, 
									ft_correction_speed_factor, ft_correction_speed_factor,
									goal->blend_radius, true);
			while(force_error > force_tolerance && traj_status_.status != traj_status_.ABORTED)
			{				
				ros::spinOnce();
				calculatePoseError_(move_group_.getCurrentPose().pose, next_wp, position_error, orientation_error);
				eef_force_eigen = Eigen::Vector3d(eef_force_[0], eef_force_[1], eef_force_[2]);
				eef_force_eigen_proj = eef_force_eigen.dot(force_target_eigen_unit) * force_target_eigen_unit;
				force_error = abs(eef_force_eigen_proj.norm()  - force_target_eigen.norm());

				execute_cartesian_trajectory_ft_feedback_.force_error = force_error;
				execute_cartesian_trajectory_ft_feedback_.position_error = position_error;
				execute_cartesian_trajectory_ft_feedback_.orientation_error = orientation_error;				
				execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);				
			}
			execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);
			move_group_.stop();

			geometry_msgs::Pose end_pose = move_group_.getCurrentPose().pose;
			Eigen::Vector3d end_pose_eigen(end_pose.position.x, end_pose.position.y, end_pose.position.z);
			Eigen::Vector3d start_pose_eigen(start_pose.position.x, start_pose.position.y, start_pose.position.z);
			Eigen::Vector3d K_total_eigen = end_pose_eigen - start_pose_eigen;
			double K_total = K_total_eigen.norm();
						
			new_waypoints.clear();
			//MODIFY REMAINING WAYPOINTS
			for(int i = next_wp_id; 
				i<goal_waypoints.size(); 
				i++)
			{
				geometry_msgs::Pose new_wp = goal_waypoints[i];
				new_wp.position.x += K_total * force_diff_base_unit[0];
				new_wp.position.y += K_total * force_diff_base_unit[1];
				new_wp.position.z += K_total * force_diff_base_unit[2];
				new_waypoints.push_back(new_wp);
				goal_waypoints[i] = new_wp;	
			}

			ros::Duration(timeout_before_planning).sleep();
			ret = false;
			while (!ret)
				ret = planAndExecuteCartesianPath_(new_waypoints, goal->eef_step, goal->jump_threshold, 
										goal->max_velocity_scaling_factor, goal->max_acceleration_scaling_factor,
										goal->blend_radius, true);
			// ros::Duration(timeout).sleep();
			execute_cartesian_trajectory_ft_feedback_.feedback = "New Trajectory";
			execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);

		}
	
		execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);
	}

	// async_spinner.stop();

	// set the action state to succeeded
	execute_cartesian_trajectory_ft_feedback_.feedback = "Trajectory execution completed";
	std::cout << execute_cartesian_trajectory_ft_feedback_.feedback << std::endl;
	execute_cartesian_trajectory_ft_as_.publishFeedback(execute_cartesian_trajectory_ft_feedback_);
	
	execute_cartesian_trajectory_ft_result_.pose = move_group_.getCurrentPose().pose;
	execute_cartesian_trajectory_ft_as_.setSucceeded(execute_cartesian_trajectory_ft_result_);
}

void YK_Interface::checkTrajStatus(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
	int len = msg->status_list.size();
	if (len != 0)
		traj_status_ = msg->status_list[len - 1];
}