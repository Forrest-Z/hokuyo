#ifndef _BOB_STC_GRID_MAP_H_
#define _BOB_STC_GRID_MAP_H_

#include <vector>
#include <unordered_set>

#include <bob_stc/grid.h>
#include <bob_stc/grid_point.h>
#include <bob_stc/grid_set.h>
#include <bob_toolbox/world_point.h>

namespace bob 
{

	//
	//! This class represents a grid which contains different types of cells.
	//! This grid is built from a map and then used to create spanning trees.
	//
	class GridMap : public Grid
	{

		//! Stores the points in the grid
		std::unordered_set<GridPoint, GridPointHash> freePoints;
		std::unordered_set<GridPoint, GridPointHash> partiallyFreePoints;

	public:
		
		//! Simple constructor that locates the map
		GridMap(WorldPoint origin, float gridRotation, float gridWidth) : Grid(origin, gridRotation, gridWidth){};

		GridMap() : Grid(WorldPoint(0, 0), 0, 0.7){};

		//! These functions add vertices to the grid
		void addPoint(const GridPoint& point, GridPointType type);

		//! These functions remove vertices from the grid
		void removePoint(const GridPoint& point);

		//! These functions test for membership of a vertex
		bool containsPoint(const GridPoint& point) const;

		bool containsPoint(const GridPoint& point, GridPointType) const;

		int numFree();

		GridSet freeSet();
	};

}

#endif
