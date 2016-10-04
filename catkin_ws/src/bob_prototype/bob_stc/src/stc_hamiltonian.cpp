#include <bob_stc/stc_hamiltonian.h>

#include <vector>
#include <utility>
#include <cmath>
#include <limits>

#include <bob_stc/spanning_tree_grid.h>
#include <bob_stc/spanning_tree_grid_depth_return_iterator.h>
#include <bob_toolbox/direction.h>
#include <bob_toolbox/line_geometry.h>
#include <bob_visualization/visualization.h>
#include <bob_stc/grid_set.h>



namespace bob
{
	Quadrant STCHamiltonian::quadrantAfterGoal(Direction movedDirection)
	{
		Quadrant resultingQuadrant;
		switch (movedDirection)
		{
			case up:
				resultingQuadrant = 4;
				break;
			case left:
				resultingQuadrant = 1;
				break;
			case down:
				resultingQuadrant = 2;
				break;
			case right:
				resultingQuadrant = 3;
				break;
		}
		return resultingQuadrant;
	}

	std::vector<WorldPoint> STCHamiltonian::pointsForTurn(const SpanningTreeGrid& source, Quadrant startingQuadrant, GridPoint gridPivot, Direction turnDirection)
	{
		BendType bend = calculateBendType(startingQuadrant, turnDirection);
		std::vector<Quadrant> quadrantsToAdd = quadrantsWithCorner(bend, startingQuadrant);	
		return quadrantPoints(source, gridPivot, quadrantsToAdd);
	}

	STCHamiltonian::BendType STCHamiltonian::calculateBendType(Quadrant startingQuadrant, Direction turnDirection)
	{
		Quadrant exitQuadrant;

		// Converts the enum to an int representing the quadrant just to 
		// the counter-clockwise of the direction
		switch (turnDirection)
		{
			case up:
				exitQuadrant = 1;
				break;
			case left:
				exitQuadrant = 2;	
				break;
			case down:
				exitQuadrant = 3;
				break;
			case right:
				exitQuadrant = 4;
				break;
		}

		// The equation below is a bit strange because modulus can return a negative
		// value when given a negative input. Also, the problem where 4 % 4 = 8 % 4 = 0 (zero values).
		// This modification is used to make 4 % 4 = 0 % 4 = 4
		int numQuadrantsPassedThrough = 1 + (exitQuadrant - startingQuadrant + 8) % 4;

		BendType toReturn;
		switch (numQuadrantsPassedThrough)
		{
			case 1:
				toReturn = RightTurn;
				break;
			case 2:
				toReturn = Straight;
				break;
			case 3:
				toReturn = LeftTurn;
				break;
			case 4:
				toReturn = UTurn;
				break;
		}
		return toReturn;
	}

	std::vector<Quadrant> STCHamiltonian::quadrantsWithCorner(BendType bendType, Quadrant startingQuadrant)
	{
		std::vector<Quadrant> quadrants;

		switch (bendType)
		{
			case LeftTurn:
				quadrants.push_back(startingQuadrant.rotate(1));
				break;
			case UTurn:
				quadrants.push_back(startingQuadrant.rotate(1));
				quadrants.push_back(startingQuadrant.rotate(1));
				break;
			case RightTurn:
				quadrants.push_back(startingQuadrant);
				break;
				// Note: there is another case: Straight, but it has no corners
		}
		return quadrants;

	}

	std::vector<WorldPoint> STCHamiltonian::quadrantPoints(const SpanningTreeGrid& source, GridPoint gridPoint, std::vector<Quadrant>& quadrants)
	{
		std::vector<WorldPoint> points;
		for (std::vector<Quadrant>::iterator itr = quadrants.begin(); itr != quadrants.end(); ++itr)
		{
			WorldPoint newPoint = quadrantPoint(source, gridPoint, *itr);
			points.push_back(newPoint);
		}
		return points;
	}

	WorldPoint STCHamiltonian::quadrantPoint(const SpanningTreeGrid& source, GridPoint gridPoint, Quadrant quadrant)
	{
		GridPoint offsetPoint;
		switch (quadrant){
			case 1:
				offsetPoint.x = gridPoint.x + 0.25;
				offsetPoint.y = gridPoint.y + 0.25;
				break;
			case 2:
				offsetPoint.x = gridPoint.x - 0.25;
				offsetPoint.y = gridPoint.y + 0.25;
				break;
			case 3:
				offsetPoint.x = gridPoint.x - 0.25;
				offsetPoint.y = gridPoint.y - 0.25;
				break;
			case 4:
				offsetPoint.x = gridPoint.x + 0.25;
				offsetPoint.y = gridPoint.y - 0.25;
				break;
			default:
				offsetPoint = gridPoint;
		}
		return source.gridToWorld(offsetPoint);
	}

	STCPath STCHamiltonian::generatePath(const SpanningTreeGrid& freeTree)
	{
		STCPath path;

		if (freeTree.hasChildren())		
		{
			// The quadrant from which we enter into the current cell
			Quadrant entryIntoCell;

			SpanningTreeGridDepthReturnIterator iterator = freeTree.depthReturnIterator();

			// Start in the first node AFTER the parent, not the parent
			entryIntoCell = quadrantAfterGoal(iterator.directionToNext());
			iterator.next();

			while (!iterator.done())
			{
				// Determine points for cell
				std::vector<WorldPoint> turnPoints = pointsForTurn(freeTree, entryIntoCell, iterator.currentPoint(), iterator.directionToNext());

				// Add the points into the path
				path.insert(path.end(), turnPoints.begin(), turnPoints.end());

				// Determine entry into next cell and then enter the cell
				entryIntoCell = quadrantAfterGoal(iterator.directionToNext());
				iterator.next();
			}		

			// Considering the quadrants of the parent point
			// Re-initialize iterator so it is at parent point again
			// entryIntoCell now points to the entry into parent point (which can't be
			// determined until the entire tree was iterated over and returned
			// to start)
			iterator = freeTree.depthReturnIterator();
			std::vector<WorldPoint> turnPoints = pointsForTurn(freeTree, entryIntoCell, iterator.currentPoint(), iterator.directionToNext());
			path.insert(path.end(), turnPoints.begin(), turnPoints.end());
			
		}
		else
		{
			// Special case when the tree is a single point.
			// Just make a square around the point
			GridPoint onlyPoint = freeTree.parentPoint();	
			for (int quadrant = 1; quadrant <= 4; quadrant++)
			{
				path.push_back(quadrantPoint(freeTree, onlyPoint, quadrant));
			}
		}
		path.push_back(*(path.begin()));
		return path;
	}
}
