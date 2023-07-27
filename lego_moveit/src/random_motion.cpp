#include <ros/ros.h>
#include <lego_moveit/MakeMove.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>

void random_motion(ros::NodeHandle& nh);
void move_here(ros::NodeHandle& nh, geometry_msgs::Pose requestPose);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "random_motion");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ros::NodeHandle nh;

	ROS_INFO("random_motion node has been initialized");

	random_motion(nh);

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
