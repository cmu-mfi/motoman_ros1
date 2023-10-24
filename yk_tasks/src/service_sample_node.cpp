#include <ros/ros.h>
#include <yk_tasks/SetPose.h>
#include <yk_tasks/GetPose.h>


void get_pose_client(ros::NodeHandle nh);

void make_a_move_client(ros::NodeHandle nh);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_sample_node");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ros::NodeHandle nh;
 	ros::NodeHandle private_node_handle("~");

  	std::string base_frame, task;
  	private_node_handle.param<std::string>("base_frame", base_frame, "base_link"); // parameter name, string object reference, default value
	private_node_handle.param<std::string>("task", task, "get_pose"); // parameter name, string object reference, default value

	ROS_INFO("sample node has been initialized");

	if(task == "get_pose")
    	get_pose_client(nh);
	else if(task == "make_a_move")
		make_a_move_client(nh);
	else
		ROS_ERROR_STREAM("Invalid task: "<<task);

	ros::waitForShutdown();
  	//ros::spin();
}

void get_pose_client(ros::NodeHandle nh)
{
	ros::ServiceClient client = nh.serviceClient<yk_tasks::GetPose>("yk_get_pose");
	yk_tasks::GetPose srv;

  	ROS_INFO("Calling GetPose service");

  	srv.request.frame = "base_link";
	
	if(client.call(srv))
  	{
		ROS_INFO_STREAM("Pose: "<<srv.response.pose);
  	}
}

void make_a_move_client(ros::NodeHandle nh)
{
	ROS_INFO_STREAM("Calling SetPose service");
 	geometry_msgs::Pose requestPose;
	requestPose.position.x = .210;		//.362019;
	requestPose.position.y = -0.133;	//0;
	requestPose.position.z = .30;		//0.60;
	requestPose.orientation.x = 0;	//1.0;	//0.71;
	requestPose.orientation.y = -1;	//-0.023;	//0;
	requestPose.orientation.z = 0;	//-0.028;	//0.71;
	requestPose.orientation.w = 0;	//0.02;	//0;

	ros::ServiceClient client = nh.serviceClient<yk_tasks::SetPose>("yk_set_pose");
	yk_tasks::SetPose srv;

	srv.request.pose = requestPose;
	srv.request.base_frame = "base_link";
	if(client.call(srv))
	{
		ROS_INFO_STREAM("Pose: "<<srv.response.pose);
  	}
}
