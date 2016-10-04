#ifndef _BOB_MAP_ALGORITHMS_OBSTACLE_LOCATOR_H_
#define _BOB_MAP_ALGORITHMS_OBSTACLE_LOCATOR_H_

#include <bob_grid_map/raw_map.h>
#include <bob_grid_map/map_location_set.h>

namespace bob
{

	class Costmap;	

	class ObstacleLocator
	{

		public:

			ObstacleLocator(const Costmap& map):
				map(map)
			{}
			
			std::vector<MapLocationSet> getGroupedObstacles(const WorldPoint& seed) const;

		private:

			MapLocationSet getObstacleSet(const MapLocation& seed) const;

			const Costmap& map;
	};

}

#endif
