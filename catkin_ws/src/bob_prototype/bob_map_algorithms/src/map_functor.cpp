#include <bob_map_algorithms/map_functor.h>

namespace bob
{

	void CompositeMapFunctor::add(const MapFunctor::shared_ptr functor)
	{
		functors.push_back(functor);
	}
	
	bool CellStateIs::operator()(const MapLocation& point) const
	{
		return costmap.getObstacleMap().pointFree(point) == state; 
	}

	bool CellAwayFromObstacles::operator()(const MapLocation& point) const
	{
		return (costmap.getObstacleDistanceMap().obstacleDistance(point) >= minDistance);
	}

	bool OredMapFunctor::operator()(const MapLocation& point) const
	{
		bool toReturn = false;
		for (std::vector<MapFunctor::shared_ptr>::const_iterator itr = functors.begin(); itr != functors.end(); ++itr)
		{
			toReturn = toReturn || (**itr)(point) ;
		}
		return toReturn;
	}

	bool AndedMapFunctor::operator()(const MapLocation& point) const
	{
		bool toReturn = true;
		for (std::vector<MapFunctor::shared_ptr>::const_iterator itr = functors.begin(); itr != functors.end(); ++itr)
		{
			toReturn = toReturn && (**itr)(point) ;
		}
		return toReturn;
	}

	bool NotMapFunctor::operator()(const MapLocation& point) const
	{
		return !((*toBeNotFunctor)(point));
	}


}

