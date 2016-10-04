#include <bob_ros_implementations/ros_map_sender.h>

#include <bob_toolbox/strict_lock.h>
#include <bob_grid_map/costmap.h>

namespace bob
{

	void ROSMapSender::update()
	{
		MapSender::update();	

		// Note use of base class interface
		LockableMap& lockableMap = getMap();
	
		// Locking map
		StrictLock lock(lockableMap.getLock());
		Costmap& map = lockableMap.getLockedResource(lock);

		// Publishing
		obstacleMapPublisher.publishCostmap(map.getObstacleMap());
		obstacleDistanceMapPublisher.publishCostmap(map.getObstacleDistanceMap());
		probabilityMapPublisher.publishCostmap(map.getProbabilityMap());
	}

}

