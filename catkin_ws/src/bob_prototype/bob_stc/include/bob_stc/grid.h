#ifndef _BOB_STC_GRID_H_
#define _BOB_STC_GRID_H_

#include <bob_stc/grid_point.h>
#include <bob_toolbox/world_point.h>
#include <utility>
#include <cmath>

namespace bob 
{

	class Grid 
	{

		//! The co-ordinates of the origin of the grid in world frame.
		WorldPoint origin;
		
		//! The rotation of the grid along the z axis relative to world frame.
		float rotation;
		
		//! The width of a cell in the grid.
		float width;

	public:
		
		//! Simple constructor that locates the grid
		Grid(WorldPoint o, float r, float w) : origin(o), width(w) 
		{
			//! Limit rotation to first quadrant to simplify calculations
			rotation = std::fmod(r, M_PI/2);
			if (rotation < 0)
				rotation += M_PI/2;
		};

		//! Converts grid co-ordinates into world co-ordinates. The grid cell does not have to exist in the graph.
		//! Doubles are accepted as input to allow co-ordinates in-between grid cells to be located.
		//
		//! Input:
		//! gridIndices - The indices of the grid cell
		//
		//! Output:
		//! The co-ordinates of the point in the world frame
		WorldPoint gridToWorld(std::pair<float, float> gridIndices) const;

		//! Version of gridToWorld that accepts int pairs
		WorldPoint gridToWorld(GridPoint gridIndices) const;
		
		WorldPoint gridToWorld(float gridX, float gridY) const;

		//! Converts world co-ordinates into grid indices. The grid cell does not have to exist in the graph.
		//
		//! Input:
		//! worldCoordinates - The co-ordinates of the point in the world frame
		//
		//! Output:
		//! The indices of the cell that contains the point
		GridPoint worldToGrid(WorldPoint worldCoordinates) const;
	
		float getRotation() const;
		float getWidth() const;
		WorldPoint getOrigin() const;	
	};
}

#endif
