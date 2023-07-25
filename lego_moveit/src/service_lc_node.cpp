#include <ros/ros.h>
#include <lego_moveit/MakeMove.h>
#include <lego_moveit/GetPose.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>


void get_pose_client(ros::NodeHandle nh);

void make_a_move_client(ros::NodeHandle nh);

void make_a_move_client2(ros::NodeHandle nh);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_lc_node");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ros::NodeHandle nh;
 	ros::NodeHandle private_node_handle("~");

  	std::string base_frame;
  	private_node_handle.param<std::string>("base_frame", base_frame, "base_link"); // parameter name, string object reference, default value

    

	ROS_INFO("sample node has been initialized");

	get_pose_client(nh);

	while(ros::ok())
	{
		make_a_move_client(nh);
		make_a_move_client2(nh);
	}

	ros::waitForShutdown();
  	//ros::spin();
}

void get_pose_client(ros::NodeHandle nh)
{
	ros::ServiceClient client = nh.serviceClient<lego_moveit::GetPose>("get_pose");
	lego_moveit::GetPose srv;

  	ROS_INFO("sample node has been initialized");

  	srv.request.frame = "base_link";
	
	if(client.call(srv))
  	{
		ROS_INFO_STREAM("Pose: "<<srv.response.pose);
  	}
}

void make_a_move_client(ros::NodeHandle nh)
{
 	geometry_msgs::Pose requestPose;
	requestPose.position.x = .50;		//.362019;
	requestPose.position.y = -0.002;	//0;
	requestPose.position.z = .20;		//0.60;
	requestPose.orientation.x = 1.0;	//1.0;	//0.71;
	requestPose.orientation.y = -1.7;	//-0.023;	//0;
	requestPose.orientation.z = -0.03;	//-0.028;	//0.71;
	requestPose.orientation.w = 6.47;	//0.02;	//0;


	ros::ServiceClient client = nh.serviceClient<lego_moveit::MakeMove>("make_a_move");
	lego_moveit::MakeMove srv;

	srv.request.pose = requestPose;
	srv.request.base_frame = "base_link";
	if(client.call(srv))
	{
		ROS_INFO_STREAM("Pose: "<<srv.response.pose);
  	}
}


void make_a_move_client2(ros::NodeHandle nh)
{
 	geometry_msgs::Pose requestPose;
	requestPose.position.x = 0.362019;
	requestPose.position.y = 0;
	requestPose.position.z = 0.60;
	requestPose.orientation.x = 1.0;	//0.71;
	requestPose.orientation.y = -0.023;	//0;
	requestPose.orientation.z = -0.028;	//0.71;
	requestPose.orientation.w = 0.02;	//0;

	ros::ServiceClient client = nh.serviceClient<lego_moveit::MakeMove>("make_a_move");
	lego_moveit::MakeMove srv;

	srv.request.pose = requestPose;
	srv.request.base_frame = "base_link";
	if(client.call(srv))
	{
		ROS_INFO_STREAM("Pose: "<<srv.response.pose);
  	}
}
