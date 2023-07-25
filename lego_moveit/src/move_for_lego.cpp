#include <ros/ros.h>
#include <lego_moveit/MakeMove.h>
#include <lego_moveit/GetPose.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string.h>

class MoveForLego
{
private:
  ros::ServiceServer move_server_;
  ros::ServiceServer state_server_;
  tf::TransformListener listener_;
  geometry_msgs::PoseStamped last_pose_;
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterface move_group_;

public:
  MoveForLego(std::string group_name, ros::NodeHandle& nh_)
	  : move_group_(group_name)
  {
      ROS_INFO_STREAM("planning frame.."<<move_group_.getPlanningFrame());
      ROS_INFO_STREAM("End Effector.."<<move_group_.getEndEffector());
      ROS_INFO_STREAM("End Effector Link.."<<move_group_.getEndEffectorLink());

      //SERVICE SERVERS
      move_server_ = nh_.advertiseService("make_a_move", &MoveForLego::letsMove, this);
      state_server_= nh_.advertiseService("get_pose", &MoveForLego::getPoseInfo, this);

      //move_group.>setPlannerId("manipulator");
  }

  bool letsMove(lego_moveit::MakeMove::Request& req,
		lego_moveit::MakeMove::Response& res)
  {
	  ros::AsyncSpinner async_spinner(1);
	  async_spinner.start();
	  
	  ROS_INFO_STREAM("Let's Move!!");
	  //moveit::planning_interface::MoveGroupInterface move_group."manipulator"); //move_group name to be changes
	  move_group_.setPoseReferenceFrame(req.base_frame);
	  move_group_.setPoseTarget(req.pose); 
	  move_group_.move();
	  last_pose_ = move_group_.getCurrentPose();

	  ROS_INFO_STREAM(last_pose_);
	  
	  res.pose = last_pose_.pose;
	  async_spinner.stop();
	  return true; //usual practice to return false if invalid/erroneous response
  }

  bool getPoseInfo(lego_moveit::GetPose::Request& req,
		   lego_moveit::GetPose::Response& res)
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


};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "move_for_lego");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_INFO("MoveForLego node starting");
	ros::NodeHandle nh;

	std::string group_name;
	ros::param::param<std::string>("/group_name", group_name, "manipulator");

	MoveForLego gp4(group_name, nh);

	ROS_INFO("MoveForLego node started");

	//ros::spin();
	ros::waitForShutdown();
}
