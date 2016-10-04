#ifndef _BOB_GRID_MAP_IHIDDEN_OBSTACLE_MAP_H_
#define _BOB_GRID_MAP_IHIDDEN_OBSTACLE_MAP_H_

#include <bob_toolbox/world_point.h>

namespace bob
{

	//! \brief Interface that defines how hidden obstacles are recorded in the system
	class IHiddenObstacleMap
	{

		public:

			virtual void addNewHiddenObstacle(const WorldPoint& obstacle) = 0;
	
	};

}

#endif
