#ifndef _BOB_COVERAGE_AREA_TOOLS_H_
#define _BOB_COVERAGE_AREA_TOOLS_H_

#include <vector>

#include <bob_toolbox/world_point.h>
#include <bob_coverage/discrete_area.h>
#include <bob_grid_map/costmap.h>

namespace bob 
{

	//! \brief Get the area inside a polygon that is free and a certain distance from obstacles
	//! \param map The map used to check if points satisfy the conditions
	//! \param convexPolygon A polygon inside which to examine points. The polygon MUST be convex or the algorithm will have problems
	//! \param minDistance The minimum distance from obstacles
	//! \param resolution The resolution of the resulting area. Does not need to be the same as the map
	DiscreteArea extractThresholdedArea(const Costmap& map, const std::vector<WorldPoint>& convexPolygon, float minDistance, float resolution);
	
	//! \brief Find the shortest distance between a point and an area. Checks all the points
	//! in the area to see how far they are from the point, and returns the smallest value found.
	//! \param area The are to test
	//! \param point The point to use for distance calculations
	//! \return The smallest distance found
	float smallestDistanceTo(const DiscreteArea& area, WorldPoint point);

	//! \brief Find the closest area to a point in a container of areas.
	//! \param areas A container of areas.
	//! \param point The point
	//! \return Iterator to the area in the containre with the small minimum distance to the point
	template <typename Container>
	typename Container::iterator closestArea(Container& areas, WorldPoint point)
	{
		float smallestSoFar = std::numeric_limits<float>::max();
		typename Container::iterator toReturn;
		for (auto areaItr = areas.begin(); areaItr != areas.end(); ++areaItr)
		{
			float distanceToArea = smallestDistanceTo(*areaItr, point);
			if (distanceToArea < smallestSoFar)
			{
				smallestSoFar = distanceToArea;
				toReturn = areaItr;
			}
		}
		return toReturn;
	}

	//! \brief Searches to find the smallest bounding box around a given area.
	//! This function is trying to find the angle that generates the smallest 
	//! bounding box.
	//! \param area The area that which will be contained in the box
	//! \return A good approximation for the smallest bounding box around the area
	WorldRectangle minAreaBoundingBox(const DiscreteArea& area);

	//! \brief Finds a bounding box around an area. 
	//! \param The area that will fit in the box
	//! \param The angle of the bounding box, from parallel to x axis
	//! \return A rectangle which bounds the area, and is rotated from the x axis by the given angle
	WorldRectangle boundingBox(const DiscreteArea& area, float angle);

};

#endif
