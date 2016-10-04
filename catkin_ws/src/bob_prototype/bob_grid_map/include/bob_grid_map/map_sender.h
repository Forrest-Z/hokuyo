#ifndef _BOB_GRID_MAP_MAP_SENDER_H_
#define _BOB_GRID_MAP_MAP_SENDER_H_

#include <bob_grid_map/lockable_map.h>
#include <bob_toolbox/strict_lock.h>
#include <bob_grid_map/costmap.h>

namespace bob
{

	//! This class is meant to make it easy to switch between ROS and
	//! non-ROS systems. An analogous ROS version exists called ROSMapSender.

	//! Since it takes references to this class as an argument, the SLAM
	//! algorithm doesn't know if it has a MapSender or ROSMapSender object.
	//! Therefore, the SLAM algorithm can be configured to send its data over
	//! ROS (via ROSCostmapPublisher) or, copy and modify the data directly.
	class MapSender
	{

		public:

			MapSender(LockableMap& map) : 
			map(map)
			{}

			LockableMap& getMap()
			{
				return map;
			}

			virtual void update()
			{
				StrictLock lock(map.getLock());
				Costmap& costmap = map.getLockedResource(lock);
				costmap.inflateObstacles();
			}

		private:

			LockableMap& map;
		
	};

}

#endif
