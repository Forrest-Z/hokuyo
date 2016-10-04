#ifndef _BOB_MAP_ALGORITHMS_CLOSEST_CLEAR_POINT_H_
#define _BOB_MAP_ALGORITHMS_CLOSEST_CLEAR_POINT_H_

#include <bob_grid_map/costmap.h>

namespace bob
{
	//! This class provides a way to get the close clearest point near the given point
	class ClosestClearPoint
	{
		public:
			ClosestClearPoint(const Costmap& costmap):
				costmap(costmap) {}

			WorldPoint getClosestClearPoint(const WorldPoint& basePoint);
			MapLocation getClosestClearPoint(const MapLocation& basePoint);

		private:
			const Costmap& costmap;
	};

}

#endif
