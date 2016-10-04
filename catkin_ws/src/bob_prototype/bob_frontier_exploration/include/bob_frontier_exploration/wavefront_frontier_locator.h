#ifndef _BOB_FRONTIER_EXPLORATION_WAVEFRONT_FRONTIER_LOCATOR_H_
#define _BOB_FRONTIER_EXPLORATION_WAVEFRONT_FRONTIER_LOCATOR_H_

#include <bob_frontier_exploration/frontier_cloud.h>

namespace bob
{

	class Costmap;

	class WavefrontFrontierLocator 
	{

		public:

			WavefrontFrontierLocator(const Costmap& map) : 
				map(map)
		{};

			FrontierCloud getFrontiers(MapLocationSet& closedSet, MapLocation seed) const;

			FrontierCloud getFrontiers(MapLocationSet& closedSet, MapLocationSet seed) const;

		private:

			const Costmap& map;
			
			

	};

}

#endif
