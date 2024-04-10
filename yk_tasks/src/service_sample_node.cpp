#include <ros/ros.h>
#include <yk_tasks/SetPose.h>
#include <yk_tasks/GetPose.h>
#include <yk_tasks/SetJoints.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>

void get_pose_client(ros::NodeHandle nh);

void make_a_move_client(ros::NodeHandle nh);

void move_joints_client(ros::NodeHandle nh);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_sample_node");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ros::NodeHandle nh;
 	ros::NodeHandle private_node_handle("~");

  	std::string base_frame, task;
  	private_node_handle.param<std::string>("base_frame", base_frame, "base_link"); // parameter name, string object reference, default value
	private_node_handle.param<std::string>("task", task, "set_joints"); // parameter name, string object reference, default value

	ROS_INFO("sample node has been initialized ok");

	if(task == "get_pose")
    	get_pose_client(nh);
	else if(task == "make_a_move")
		make_a_move_client(nh);
	else if(task == "set_joints")
		move_joints_client(nh);
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

void move_joints_client(ros::NodeHandle nh)
{
	ROS_INFO_STREAM("Calling SetJoints service");

	ros::Publisher event_pub = nh.advertise<std_msgs::Header>("event_rec",10);

	float positions[2][6] = 
		{1.8, 5.681410402758047e-05, -2.1914011085755192e-05, 4.135732524446212e-05, -1.5026921033859253, -3.834952076431364e-05,
		-1.8, 5.681410402758047e-05, -2.1914011085755192e-05, 4.135732524446212e-05, -1.5026921033859253, -3.834952076431364e-05};
		// {0.8287244439125061, 0.14966729283332825, 0.49648383259773254, 0.031059350818395615, -1.8177671432495117, -0.28482189774513245,
		// 0.8555324673652649, 0.18888795375823975, 0.5419992208480835, 0.03393368422985077, -1.825234055519104, -0.3105544149875641};

	int index = 0;

	for (int i = 0; i<100; ++i)
	{
		sensor_msgs::JointState joint_goal_pose; 
		joint_goal_pose.name.push_back("joint_1_s");
		joint_goal_pose.name.push_back("joint_2_l");
		joint_goal_pose.name.push_back("joint_3_u");
		joint_goal_pose.name.push_back("joint_4_r");
		joint_goal_pose.name.push_back("joint_5_b");
		joint_goal_pose.name.push_back("joint_6_t");

		ros::ServiceClient client = nh.serviceClient<yk_tasks::SetJoints>("yk_set_joints");
		yk_tasks::SetJoints srv;

		for(int j=0; j< 6; j++)
		{
			joint_goal_pose.position.push_back(positions[index][j]);
		}

		srv.request.state = joint_goal_pose;

		if(client.call(srv))
		{
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.frame_id = index?"Robot Position 2":"Robot Position 1";
			event_pub.publish(header);

			ROS_INFO_STREAM("joints reached-"<<index<<"-"<<i);
			ROS_INFO_STREAM(joint_goal_pose);
		}

		index = (i+1)%2;
		ros::Duration(0.5).sleep();
	}
}