#ifndef _BOB_GRID_MAP_ROS_MAP_LISTENER_H_
#define _BOB_GRID_MAP_ROS_MAP_LISTENER_H_

#include <bob_ros_implementations/ros_costmap_publisher.h>

#include <nav_msgs/OccupancyGrid.h>

#include <boost/thread.hpp>

namespace bob
{

	class LockableMap;

	//! ROS wrapper for costmap2D.
	//! Subscribes to ROS "map" topic.
	//! Receives map data and passes it onto a costmap.
	//! Multi-threaded due to use of LockableMap
	class ROSMapListener
	{

		public:

			ROSMapListener(LockableMap& lockableMap);

		private:

			void newMapData(const nav_msgs::OccupancyGridConstPtr& new_map);

			void mapUpdateLoop(float frequency);

			LockableMap& lockableMap;

			ros::Subscriber mapSubscriber;		

			ros::Duration publishTime;

			ROSCostmapPublisher staticMapPublisher;
			ROSCostmapPublisher distanceMapPublisher;

			boost::thread mapUpdateThread; 
		
	};
}

#endif
