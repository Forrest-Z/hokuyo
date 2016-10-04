#ifndef _BOB_STC_GRID_SET_H_
#define _BOB_STC_GRID_SET_H_

#include <vector>
#include <unordered_set>

#include <bob_stc/grid.h>
#include <bob_stc/grid_point.h>
#include <bob_toolbox/world_point.h>


namespace bob 
{

	//
	//! This class represents a graph where each vertex is aligned with a grid.
	//! Additionally, all adjacent vertices in the grid are considered connected.
	//! Non-adjacent vertices are not considered connected.
	//
	class GridSet : public Grid	
	{

		public:

			typedef std::unordered_set<GridPoint, GridPointHash>::const_iterator const_iterator;

			//! Simple constructor that locates the map
			GridSet(WorldPoint origin=WorldPoint(0, 0), float gridRotation=0, float gridWidth=0.7) : 
				Grid(origin, gridRotation, gridWidth)
		{}

			//! These functions add vertices to the grid
			void addPoint(const GridPoint& point);
			
			template <typename Container>
			void addPoints(const Container& points);

			//! These functions remove vertices from the grid
			void removePoint(const GridPoint& point);

			bool empty() const;

			GridPoint closestPointTo(WorldPoint point);

			//! These functions test for membership of a vertex
			bool containsPoint(const GridPoint& point) const;

			GridPoint getArbitraryPoint() const;

			int size() const;

			std::unordered_set<GridPoint, GridPointHash>::const_iterator cbegin() const;
			std::unordered_set<GridPoint, GridPointHash>::const_iterator cend() const;

			std::vector<GridSet> fourConnectedSets() const;

		private:

			//! Stores the points in the grid
			std::unordered_set<GridPoint, GridPointHash> points;
	};

}

#endif
