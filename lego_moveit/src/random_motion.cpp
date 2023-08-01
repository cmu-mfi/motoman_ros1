#include <ros/ros.h>
#include <lego_moveit/MakeMove.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fstream>
#include <stdlib.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

void random_motion(ros::NodeHandle& nh);
void move_here(ros::NodeHandle& nh, geometry_msgs::Pose requestPose);
void follow_points(ros::NodeHandle& nh, int pt_cnt, std::string config_fname);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "random_motion");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle("~");

	ROS_INFO("random_motion node has been initialized");

    std::string config_fname;
    int point_count;
    //config_fname = "/home/mfi/repos/ros1_ws/src/motoman_ros1/lego_moveit/config/random_motion.json";
    private_node_handle.getParam("config_fname", config_fname);
    private_node_handle.param("point_count", point_count, 5);

    ROS_INFO_STREAM("points: "<<point_count);

	//random_motion(nh);
    while(ros::ok())
        follow_points(nh, point_count,config_fname);

	ros::waitForShutdown();
}

void random_motion(ros::NodeHandle& nh)
{
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    geometry_msgs::PoseStamped random_pose;

    while(ros::ok())
    {
        random_pose = move_group.getRandomPose();

	    ROS_INFO_STREAM("generated pose: "<<random_pose);

	       //checking if valid pose
	       if(
                   random_pose.pose.position.z > 0.45 &&
                   random_pose.pose.position.x < 0.29
             )
           {
		       move_group.setStartStateToCurrentState();
		       move_group.setPoseTarget(random_pose);
		       moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		       bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		       if(success)
			       move_here(nh, random_pose.pose);
	       }
	       ros::Duration(1).sleep();
    }
}

void move_here(ros::NodeHandle& nh, geometry_msgs::Pose requestPose)
{
	ros::ServiceClient client = nh.serviceClient<lego_moveit::MakeMove>("make_a_move");
	lego_moveit::MakeMove srv;

	srv.request.pose = requestPose;
	srv.request.base_frame = "base_link";

        ROS_INFO_STREAM("Requested pose: "<<srv.request.pose);


	if(client.call(srv))
	{
		ROS_INFO_STREAM("Final pose: "<<srv.response.pose);

  	}
}

void follow_points(ros::NodeHandle& nh, int pt_cnt, std::string config_fname)
{
    std::ifstream config_file(config_fname, std::ifstream::binary);
    ROS_INFO_STREAM("FILE READ:"<<config_fname);
    Json::Value config;

    config_file>>config;

    geometry_msgs::Pose pose;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    
    for(int i=1; i<=pt_cnt; i++)
    {
        ROS_INFO_STREAM("Tring position "<<i);
        pose.position.x = config[std::to_string(i)]["position"]["x"].asFloat();
        pose.position.y = config[std::to_string(i)]["position"]["y"].asFloat();
        pose.position.z = config[std::to_string(i)]["position"]["z"].asFloat();
        pose.orientation.x = config[std::to_string(i)]["orientation"]["x"].asFloat();
        pose.orientation.y = config[std::to_string(i)]["orientation"]["y"].asFloat();
        pose.orientation.z = config[std::to_string(i)]["orientation"]["z"].asFloat();
        pose.orientation.w = config[std::to_string(i)]["orientation"]["w"].asFloat();
        

		move_group.setStartStateToCurrentState();
		move_group.setPoseTarget(pose);
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		if(success)
			move_here(nh, pose);

        //:ros::Duration(1).sleep();
    }
}

    
   

