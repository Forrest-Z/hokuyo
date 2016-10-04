#ifndef _BOB_STC_STC_HAMILTONIAN_H_
#define _BOB_STC_STC_HAMILTONIAN_H_

#include <vector>
#include <utility>

#include <bob_stc/spanning_tree_grid.h>
#include <bob_stc/grid_point.h>
#include <bob_toolbox/world_point.h>
#include <bob_stc/grid_set.h>
#include <bob_stc/spanning_tree_grid_depth_return_iterator.h>
#include <bob_toolbox/quadrant.h>
#include <bob_stc/stc_path.h>

namespace bob
{

	//
	//! This class uses the spanning tree iterator to build a hamiltonian path around a spanning tree.
	//! Currently the algorithm only supports counter-clockwise traversal, but support for clockwise
	//! traversal may be added in the future.
	//! 
	class STCHamiltonian
	{

		public:

			//! Gets the hamiltonian path from the object
			STCPath generatePath(const SpanningTreeGrid& freeTree);

		private:

			//! An enum that defines the type of cell traversal
			enum BendType
			{
				LeftTurn,
				UTurn,
				RightTurn,
				Straight
			};

			//! Generates a WorldPoint located at a given quadrant of a grid cell, with 
			//! additional option to add a special offset
			//
			//! source - The spanning tree
			//! gridPoint - The grid cell in which the quadrant lies
			//! quadrant - The desired quadrant
			WorldPoint quadrantPoint(const SpanningTreeGrid& source, GridPoint gridPoint, Quadrant quadrant);

			//! Adds path points within a gridcell that is assumed to be completely free of obstacles
			//
			//! source - The spanning tree
			//! startingQuadrant - The quadrant in the map that starts the path
			//! gridPivot - The gridCell around which to add points for the path
			//! turnDirection - The direction to the next gridcell in the spanning tree (so we know where to go and where to exit)
			std::vector<WorldPoint> pointsForTurn(const SpanningTreeGrid& source, Quadrant startingQuadrant, GridPoint gridPivot, Direction turnDirection);

			//! Adds points to the path based on a given on a list of quadrants around a gridPoint
			//
			//! source - The spanning tree
			//! gridPoint - The grid cell around which to add the path points
			//! quadrantsToAdd - The quadrants around the point to add to the path  
			std::vector<WorldPoint> quadrantPoints(const SpanningTreeGrid& source, GridPoint gridPoint, std::vector<Quadrant>& quadrants);

			//! Parameterizes the type of bend
			//! 
			//! startingQuadrant - The quadrant to start the bend
			//! turnDirection - The the direction to the next grid cell
			//
			//! Returns:
			//! The type of bend resulting from the input
			BendType calculateBendType(Quadrant startingQuadrant, Direction turnDirection);

			std::vector<Quadrant> quadrantsWithCorner(BendType bendType, Quadrant startingQuadrant);

			Quadrant quadrantAfterGoal(Direction movedDirection);


	};

}

#endif
