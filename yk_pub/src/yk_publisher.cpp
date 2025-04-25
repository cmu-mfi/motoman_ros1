#include <yk_publisher.h>

YKPublisher::YKPublisher(ros::NodeHandle &nh_)
{
	//READ CONFIG FILES
	std::string current_dir = ros::package::getPath("yk_pub");
	std::string robot_config_file = current_dir + "/config/gp4.json";
	robot_DH_ = loadFromFile_(robot_config_file, "DH");
	robot_base_frame_ = loadFromFile_(robot_config_file, "base");

	// TOPIC PUBLISHERS
	tool0_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tool0_pose", 1);
	

	//TOPIC SUBSCRIBERS
	joint_states_sub_ = nh_.subscribe("joint_states", 1, &YKPublisher::jointStatesCallback, this);

}

void YKPublisher::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	// READ JOINT STATES
	VectorJd q(msg->position.size());
	for(int i=0; i<msg->position.size(); i++)
	{
		q(i) = msg->position[i];
	}

	// COMPUTE FORWARD KINEMATICS
	Eigen::Matrix4d tool0_pose = _FK(q, robot_DH_, robot_base_frame_, true);

	// PUBLISH TOOL0 POSE
	geometry_msgs::PoseStamped tool0_pose_msg;
	tool0_pose_msg.header.stamp = ros::Time::now();
	tool0_pose_msg.header.frame_id = "base";
	tool0_pose_msg.pose.position.x = tool0_pose(0, 3);
	tool0_pose_msg.pose.position.y = tool0_pose(1, 3);
	tool0_pose_msg.pose.position.z = tool0_pose(2, 3);
	Eigen::Quaterniond q_(tool0_pose.block<3, 3>(0, 0));
	tool0_pose_msg.pose.orientation.x = q_.x();
	tool0_pose_msg.pose.orientation.y = q_.y();
	tool0_pose_msg.pose.orientation.z = q_.z();
	tool0_pose_msg.pose.orientation.w = q_.w();
	
	tool0_pose_pub_.publish(tool0_pose_msg);
}


