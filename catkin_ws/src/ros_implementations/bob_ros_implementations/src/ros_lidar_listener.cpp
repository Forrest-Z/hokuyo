#include <bob_ros_implementations/ros_lidar_listener.h>

namespace bob
{

	ROSLidarListener::ROSLidarListener() :
	receivedScan(false)
	{
		ros::NodeHandle nh;
		laserSubscriber = nh.subscribe("scan", 1, &ROSLidarListener::messageCallback, this);
	}

	const sensor_msgs::LaserScan ROSLidarListener::mostRecentScan() const
	{
		if (receivedScan)
		{
			return cachedScan;
		}
		else
		{
			waitForScan();
			return mostRecentScan();
		}
	}

	void ROSLidarListener::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		cachedScan = *msg;		
		receivedScan = true;
	}

	void ROSLidarListener::waitForScan() const
	{
		while (!receivedScan)
		{
			ros::Duration(0.01).sleep();
		}
	}
}

