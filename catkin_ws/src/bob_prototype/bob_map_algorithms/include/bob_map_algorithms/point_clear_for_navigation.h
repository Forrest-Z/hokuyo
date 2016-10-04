#ifndef _BOB_MAP_ALGORITHMS_POINT_CLEAR_FOR_NAVIGATION_H_
#define _BOB_MAP_ALGORITHMS_POINT_CLEAR_FOR_NAVIGATION_H_

#include <bob_config/config.h>

#include <bob_grid_map/costmap.h>

namespace bob
{

	class PointClearForNavigation
	{
		public:

			PointClearForNavigation() : inscribedRadius(Config::ROBOT_RADIUS + 0.04) {}


			bool pointClearForNavigation(const Costmap& costmap, const MapLocation& point) const;
			

			MapLocation expandGetClearPoint(const Costmap& costmap, const MapLocation& seed) const;
	
		private:
			
			float inscribedRadius;

	};

}

#endif
