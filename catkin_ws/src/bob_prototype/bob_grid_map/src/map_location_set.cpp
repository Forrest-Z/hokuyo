#include <bob_grid_map/map_location_set.h>

#include <boost/functional/hash/hash.hpp>

namespace bob
{

	std::size_t MapLocationHash::operator()(const MapLocation& point) const
	{
		std::size_t seed = 0;
		boost::hash_combine(seed, point.x);
		boost::hash_combine(seed, point.y);
		return seed;
	}

}
