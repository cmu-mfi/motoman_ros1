#include <ros/ros.h>
#include <string.h>
#include <yk_interface.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "yk_interface");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_INFO("yk_interface node starting");
	ros::NodeHandle nh;

	std::string group_name;
	ros::param::param<std::string>("/group_name", group_name, "manipulator");

	YK_Interface gp4(group_name, nh);

	ROS_INFO("yk_interface node started");

	// ros::spin();
	ros::waitForShutdown();
}
