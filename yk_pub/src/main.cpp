#include <ros/ros.h>
#include <string.h>
#include <yk_publisher.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "yk_pub");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_INFO("yk_pub node starting");
	ros::NodeHandle nh;

	YKPublisher gp4(nh);

	ROS_INFO("yk_pub node started");

	// ros::spin();
	ros::waitForShutdown();
}
