#include <bob_stc/grid_map.h>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_set>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <bob_stc/grid_point.h>
#include <bob_toolbox/world_point.h>
#include <bob_stc/grid_set.h>
#include <bob_coverage/area_tools.h>
#include <bob_toolbox/grid_algorithms.h>

namespace bob
{

	void GridMap::addPoint(const GridPoint& point, GridPointType type)
	{
		if (type == CompletelyFree)
			freePoints.insert(point);
		else if (type == PartiallyFree)
			partiallyFreePoints.insert(point);
	}

	void GridMap::removePoint(const GridPoint& point)
	{
		if (containsPoint(point, CompletelyFree))
			freePoints.erase(point);
		else if (containsPoint(point, PartiallyFree))
			partiallyFreePoints.erase(point);
	}

	bool GridMap::containsPoint(const GridPoint& point) const
	{
		return (freePoints.count(point) == 1 || partiallyFreePoints.count(point));
	}

	bool GridMap::containsPoint(const GridPoint& point, GridPointType type) const
	{
		if (type == CompletelyFree)
			return (freePoints.count(point) == 1);
		else if (type == PartiallyFree)
			return (partiallyFreePoints.count(point) == 1);
	}

	GridSet GridMap::freeSet()
	{
		GridSet newSet(getOrigin(), getRotation(), getWidth());
		for (std::unordered_set<GridPoint, GridPointHash>::iterator itr = freePoints.begin(); itr != freePoints.end(); ++itr)
			newSet.addPoint(*itr);
		return newSet;
	}

	int GridMap::numFree()
	{
		return freePoints.size();
	}

}

