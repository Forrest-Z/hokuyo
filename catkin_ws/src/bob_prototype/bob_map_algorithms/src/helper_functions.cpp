#include <bob_map_algorithms/helper_functions.h>

namespace bob
{

	bool allPointsSatisfy(std::vector<MapLocation> points, MapFunctor& functor)
	{
		for(std::vector<MapLocation>::iterator itr = points.begin(); itr != points.end(); ++itr)
		{
			if(!functor(*itr))
			{
				return false;
			}
		}
		return true;
	}
}

