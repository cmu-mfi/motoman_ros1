#ifndef _YK_INTERFACE_
#define _YK_INTERFACE_

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <string.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include <ros/package.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorJd;

#define PI 3.141592653589793

class YKPublisher
{
private:
	// TOPIC SUBSCIRBERS
	ros::Subscriber joint_states_sub_;
	ros::Publisher tool0_pose_pub_;
	Eigen::MatrixXd robot_DH_;
	Eigen::MatrixXd robot_base_frame_;

	// UTILS
	Eigen::Matrix4d _FK(const VectorJd& q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad);
	Eigen::MatrixXd loadFromFile_(std::string file_path, std::string key);

public:
	YKPublisher(ros::NodeHandle &nh_);
	void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

};

#endif
