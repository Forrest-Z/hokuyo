#ifndef _BOB_GRID_MAP_ROS_MAP_SENDER_H_
#define _BOB_GRID_MAP_ROS_MAP_SENDER_H_

#include <bob_grid_map/map_sender.h>
#include <bob_ros_implementations/ros_costmap_publisher.h>

namespace bob
{

	//! The ROS version of MapSender
	//! This version publishes its map to ROS when update() is called.
	//! Only use this version if you will be receiving the map with a ROSMapListener
	//! This CANNOT replace MapSender because it does not inflate the obstacles
	//! when update() is called. That would be unnecessary extra work, because
	//! it would have to be duplicated in ROSMapListener
	class ROSMapSender : public MapSender
	{

		public:

			ROSMapSender(LockableMap& lockableMap) :
			MapSender(lockableMap),
			obstacleMapPublisher("obstacles"),
			obstacleDistanceMapPublisher("obstacle_distances"),
			probabilityMapPublisher("probabilities")
			{}
			
			virtual void update();

		private:

			ROSCostmapPublisher obstacleMapPublisher;
			ROSCostmapPublisher obstacleDistanceMapPublisher;
			ROSCostmapPublisher probabilityMapPublisher;

	};

}

#endif
