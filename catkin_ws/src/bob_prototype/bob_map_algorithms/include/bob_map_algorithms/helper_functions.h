#ifndef _BOB_MAP_ALGORITHMS_HELPER_FUNCTIONS_H_
#define _BOB_MAP_ALGORITHMS_HELPER_FUNCTIONS_H_

#include <bob_map_algorithms/map_functor.h>

namespace bob
{

	//! Checks if all the points in a vector satisfy a functor
	bool allPointsSatisfy(std::vector<MapLocation> points, MapFunctor& functor);

}

#endif
