#include <ros/ros.h>
#include <lego_moveit/MakeMove.h>
#include <lego_moveit/GetPose.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_sample_node");

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle("~");

  std::string base_frame;
  private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

  ros::ServiceClient client = nh.serviceClient<lego_moveit::GetPose>("get_pose");
  lego_moveit::GetPose srv;

  ROS_INFO("sample node has been initialized");

  srv.request.frame = "world";
  if(client.call(srv))
  {
	  ROS_INFO_STREAM("Pose: "<<srv.response.pose);
  }

  ros::waitForShutdown();
  //ros::spin();
}

