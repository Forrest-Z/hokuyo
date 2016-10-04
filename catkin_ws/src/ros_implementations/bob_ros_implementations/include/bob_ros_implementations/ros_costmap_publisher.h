#ifndef _BOB_GRID_MAP_ROS_COSTMAP_PUBLISHER_H_
#define _BOB_GRID_MAP_ROS_COSTMAP_PUBLISHER_H_

#include <ros/ros.h>

#include <bob_ros_implementations/to_ros.h>

#include <nav_msgs/OccupancyGrid.h>

namespace bob
{

	//! A class used to publish costmaps to ROS
	//! In order to publish maps there must be a function conertToMessageData(Datatype)
	//! defined for the map datatype, which converts the map entries into an int
	class ROSCostmapPublisher
	{

		public:

			ROSCostmapPublisher(std::string topic_name)
			{
				ros::NodeHandle nh;
				publisher = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, 1);
			}

			template <typename MapType>
				void publishCostmap(const MapType& map)
				{
					nav_msgs::OccupancyGrid msg = convertMapToROS(map);
					publisher.publish(msg);
				}

		private:

			ros::Publisher publisher;

	};
}  
#endif 
