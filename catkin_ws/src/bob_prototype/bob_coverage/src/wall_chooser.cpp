#include <bob_coverage/wall_chooser.h>

#include <bob_toolbox/strict_lock.h>
#include <bob_toolbox/set_algorithms.h>

#include <bob_grid_map/costmap.h>
#include <bob_grid_map/lockable_map.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_toolbox/pose2d.h>
#include <bob_visualization/visualization.h>
#include <bob_sensor/itransform_handle.h>
#include <bob_map_algorithms/obstacle_locator.h>

#include <limits>

namespace bob
{

	WallChooser::WallChooser(const LockableMap& lockableMap, const ITransformHandle& transformHandle):
		transformHandle(transformHandle)
	{
		// Lock costmap and locate all the obstacles		
		StrictLock lock(lockableMap.getLock());
		const Costmap& costmap = lockableMap.getLockedResource(lock);
		ObstacleLocator obstacleLocator(costmap);
		obstacles = obstacleLocator.getGroupedObstacles(transformHandle.getLocalizedPose());

		resolution = costmap.getResolution();
	}
		
	bool WallChooser::done() const
	{
		return obstacles.empty();
	}

	WorldPoint WallChooser::nextWallPoint()
	{
		// robot map location
		LocationMapper mapper(resolution);
		MapLocation robotLocation = mapper.worldToMap(transformHandle.getLocalizedPose());		
		
		// Get the closest wall point
		MapLocation closestWallPoint;
		float minDistance = std::numeric_limits<float>::max();
		std::vector<MapLocationSet>::iterator chosenObstacle;

		for (std::vector<MapLocationSet>::iterator itr = obstacles.begin(); itr != obstacles.end(); ++itr)
		{
			// Get the closest point to robot among current obstacle
			MapLocation wallPoint = closestPointTo(*itr, robotLocation);
			// Distance between robot and selected wall point for this obstacle
			float distanceToWallPoint = diagonalDistance<MapLocation>(robotLocation, wallPoint);
			if (distanceToWallPoint < minDistance)
			{
				minDistance = distanceToWallPoint;
				closestWallPoint = wallPoint;
				chosenObstacle = itr;
			}
		}

		obstacles.erase(chosenObstacle);
		return mapper.mapToWorld(closestWallPoint);
	}

}

