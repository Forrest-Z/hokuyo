#ifndef _BOB_LIDAR_LIDAR_LISTENER_H_
#define _BOB_LIDAR_LIDAR_LISTENER_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

namespace bob
{

	//! This is a class which contains a subscriber to a lidar scan
	//! Allows you to easily get the most recent scan in the buffer
	class ROSLidarListener
	{

		public:

			ROSLidarListener();

			void waitForScan() const;
	
			//! Get the most recent scan in the buffer
			const sensor_msgs::LaserScan mostRecentScan() const;

		private:

			void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

			sensor_msgs::LaserScan cachedScan;

			ros::Subscriber laserSubscriber;

			bool receivedScan;
	};

}

#endif
