#ifndef _BOB_FRONTIER_EXPLORATION_FRONTIER_CLOUD_H_
#define _BOB_FRONTIER_EXPLORATION_FRONTIER_CLOUD_H_

#include <bob_grid_map/map_location_set.h>

namespace bob
{

	typedef MapLocationSet FrontierCloud;

	MapLocation closestPoint(const FrontierCloud& cloud, MapLocation point);

}

#endif
