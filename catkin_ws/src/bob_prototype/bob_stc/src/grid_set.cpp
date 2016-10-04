#include <bob_stc/grid_set.h>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_set>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <bob_toolbox/line_geometry.h>

#include <bob_toolbox/geometry.h>
#include <bob_stc/grid_point.h>
#include <bob_toolbox/world_point.h>
#include <bob_toolbox/grid_algorithms.h>

namespace bob
{

	void GridSet::addPoint(const GridPoint& point)
	{
		points.insert(point);	
	}

	template <typename Container>
	void GridSet::addPoints(const Container& points)
	{
		for (typename Container::const_iterator itr = points.begin(); itr != points.end(); ++itr)
			addPoint(*itr);
	}	

	void GridSet::removePoint(const GridPoint& point)
	{
		points.erase(point);
	}

	bool GridSet::containsPoint(const GridPoint& point) const
	{
		return (points.count(point) == 1);
	}

	int GridSet::size() const
	{
		return points.size();
	}

	bool GridSet::empty() const
	{
		return (size() == 0);
	}

	GridPoint GridSet::getArbitraryPoint() const
	{
		return *(points.begin());
	}

	GridPoint GridSet::closestPointTo(WorldPoint point)
	{
		GridPoint toReturn;
		float lowestDistance = std::numeric_limits<float>::max();
		
		for (std::unordered_set<GridPoint, GridPointHash>::iterator itr = points.begin(); itr != points.end(); itr++){
			WorldPoint gridWorldPoint = gridToWorld(*itr);
			float newDistance = diagonalDistance(gridWorldPoint, point);
			if (newDistance < lowestDistance)
			{
				lowestDistance = newDistance;
				toReturn = *itr;
			}	
		}
		return toReturn;
	
	}

	std::unordered_set<GridPoint, GridPointHash>::const_iterator GridSet::cbegin() const
	{
		return points.begin();
	}

	std::unordered_set<GridPoint, GridPointHash>::const_iterator GridSet::cend() const
	{
		return points.end();
	}

	std::vector<GridSet> GridSet::fourConnectedSets() const
	{
		std::vector<GridSet> result;
		typedef std::vector< std::unordered_set<GridPoint, GridPointHash> > RawType;
		RawType connectedList = fourConnected(points);
		for (RawType::iterator listItr = connectedList.begin(); listItr != connectedList.end(); ++listItr)
		{
			GridSet newSet(getOrigin(), getRotation(), getWidth());
			newSet.points = *listItr;
			/*
			for (std::vector<GridPoint>::iterator pointItr = listItr->begin(); pointItr != listItr->end(); ++pointItr)
			{
				newSet.addPoint(*pointItr);
			}*/
			result.push_back(newSet);
		}
		return result;
	}
}

