#include <bob_stc/low_bend_tree_builder.h>

#include <string>
#include <utility>
#include <iterator>
#include <bob_stc/grid_tile_type.h>
#include <bob_stc/grid_point.h>
#include <bob_stc/spanning_tree_grid.h>
#include <bob_stc/spanning_tree_grid_depth_iterator.h>
#include <bob_toolbox/direction.h>
#include <boost/range/iterator_range_core.hpp>
#include <limits>
#include <set>

namespace bob
{

	SpanningTreeGrid LowBendTreeBuilder::buildTree(GridSet freeCells)
	{
		tileTypes.clear();
		remainingTiles = freeCells;
		
		// Does a "first pass" to initialize the tile types based on walls
		initializeTilesWithWalls();

		// Propogates the effects of leafs onto neighbor gridPoints
		propagateLeafs();

		// Chooses either horizontal or vertical and adjusts tile types
		chooseOrientation();

		// Print tree for debugging
		//printTree();
	
		return buildTreeFromTypes();
	}

	bool LowBendTreeBuilder::growTreeOnce(SpanningTreeGrid& tree, bool allowLocalChange, bool allowOutsideChange)
	{
		SpanningTreeGridDepthIterator depthIterator = tree.depthIterator();
		while (!depthIterator.done())
		{
			SpanningTreeGridNode* currentNode = depthIterator.currentNode();
			GridPoint currentPoint = currentNode->point();
			const GridTileType* tileType = tileTypes.left.find(currentPoint)->second;

			std::set<Direction> directionsToTry;
			if (allowLocalChange)
				directionsToTry = allDirectionsSet();
			else
				directionsToTry = tileType->possibleConnections();

			for (std::set<Direction>::iterator directionItr = directionsToTry.begin(); 
				directionItr != directionsToTry.end(); ++directionItr)
			{
				GridPoint neighborPoint = pointInDirection(currentPoint, *directionItr);

				if (remainingTiles.containsPoint(neighborPoint))
				{	
					Direction reverseDirection = directionBetween(neighborPoint, currentPoint);
					const GridTileType* neighborType = tileTypes.left.find(neighborPoint)->second;
					if (allowOutsideChange || neighborType->possibleConnections().count(reverseDirection) == 1)
					{
						currentNode->addChild(neighborPoint);
						remainingTiles.removePoint(neighborPoint);
						return true;
					}
				}
			}
			depthIterator.next(); 
		}
		return false;
	}	

	void LowBendTreeBuilder::growTreeWithoutChanging(SpanningTreeGrid& tree)
	{
		SpanningTreeGridDepthIterator depthIterator = tree.depthIterator();
		while (!depthIterator.done())
		{
			SpanningTreeGridNode* currentNode = depthIterator.currentNode();
			GridPoint currentPoint = currentNode->point();
			const GridTileType* tileType = tileTypes.left.find(currentPoint)->second;

			std::set<Direction> directionsToTry = tileType->possibleConnections();
			for (std::set<Direction>::iterator directionItr = directionsToTry.begin(); 
				directionItr != directionsToTry.end(); ++directionItr)
			{
				GridPoint neighborPoint = pointInDirection(currentPoint, *directionItr);

				if (remainingTiles.containsPoint(neighborPoint))
				{	
					Direction reverseDirection = directionBetween(neighborPoint, currentPoint);
					const GridTileType* neighborType = tileTypes.left.find(neighborPoint)->second;
					if (neighborType->possibleConnections().count(reverseDirection) == 1)
					{
						currentNode->addChild(neighborPoint);
						remainingTiles.removePoint(neighborPoint);
					}
				}
			}
			depthIterator.next(); 
		}
	}
	
	GridPoint LowBendTreeBuilder::chooseParentPoint()
	{
		std::vector<const GridTileType*> typesToTry;

		// The types will be tried in order listed here
		// We prefer to start with gridPoints for which
		// the neighbors are more limited/obvious
		typesToTry.push_back(&horizontal);
		typesToTry.push_back(&vertical);
		typesToTry.push_back(&horizontalBridge);
		typesToTry.push_back(&verticalBridge);
		typesToTry.push_back(&leaf);
		typesToTry.push_back(&two);
		typesToTry.push_back(&cross);
		for (std::vector<const GridTileType*>::const_iterator typeItr = typesToTry.begin(); typeItr != typesToTry.end(); ++typeItr)
		{
			if (tileTypes.right.find(*typeItr) != tileTypes.right.end())
			{
				// A point with the type is found. Return that point
				return tileTypes.right.find(*typeItr)->second;
			}
		}	
	}

	SpanningTreeGrid LowBendTreeBuilder::buildTreeFromTypes()
	{
		GridPoint parentPoint = chooseParentPoint();

		SpanningTreeGrid toReturn(remainingTiles, parentPoint);
		remainingTiles.removePoint(parentPoint);
		
		do
		{
			// First try to grow the tree without changing the point types
			growTreeWithoutChanging(toReturn);			

			// If points remain, then try to grow the tree just once in a few different
			// ways which will change point types, resulting in more corners
			// This is just done once to get is back in a position where we can grow
			// the tree normally again.
			if (!remainingTiles.empty())
				if(!growTreeOnce(toReturn, false, true))
					if(!growTreeOnce(toReturn, true, false))
						if(!growTreeOnce(toReturn, true, true))
							break; // We have failed to completely build the tree	

		} while (!remainingTiles.empty());

		return toReturn;
	}

	void LowBendTreeBuilder::chooseOrientation()
	{
		const GridTileType* bestOrientation;
		const GridTileType* discardedOrientation;
		std::set<Direction> directionsForDiscarded;
		
		// Choosing the dominant orientation based on quantity
		if (tileTypes.right.count(&horizontal) > tileTypes.right.count(&vertical))
		{
			bestOrientation = &horizontal;
			discardedOrientation = &vertical;
			directionsForDiscarded.insert(up);
			directionsForDiscarded.insert(down);
		}
		else
		{
			bestOrientation = &vertical;
			discardedOrientation = &horizontal;
			directionsForDiscarded.insert(left);
			directionsForDiscarded.insert(right);
		}

		// Replace all the wild tiles with the chosen orientation
		while (tileTypes.right.find(&wild) != tileTypes.right.end())
		{
			tileTypes.right.replace_key(tileTypes.right.find(&wild), bestOrientation);
		}

		// TODO: Combine this with forcing function
		std::vector<Direction> directions = allDirections();
		for (tileTypeBimap::left_iterator typeIterator = tileTypes.left.begin(); typeIterator != tileTypes.left.end(); ++typeIterator)
		{
			GridPoint currentPoint = typeIterator->first;
			std::set<Direction> directionOfLeafs;
			for (std::vector<Direction>::iterator directionIterator = directions.begin(); 
				directionIterator != directions.end(); 
				++directionIterator)	
			{
				GridPoint neighbor = pointInDirection(currentPoint, *directionIterator);
				if (tileTypes.left.find(neighbor) != tileTypes.left.end())
				{
					const GridTileType* neighborType = tileTypes.left.find(neighbor)->second;
					if (neighborType == &leaf || 
						neighborType == &horizontalBridge ||
						neighborType == &verticalBridge ||
						((bestOrientation == neighborType) && (neighborType  == &horizontal) && (*directionIterator == left || *directionIterator == right)) ||
						((bestOrientation == neighborType) && (neighborType  == &vertical) && (*directionIterator == up || *directionIterator == down)))
					{
						directionOfLeafs.insert(*directionIterator);
					}
				}
			}
			// Forcing the connections
			tileTypes.left.replace_data(typeIterator, typeAfterConnections(typeIterator->second, directionOfLeafs));	
		}
	}

	// Propagate the effect of leafs and bridges to lower quality of tiles
	void LowBendTreeBuilder::propagateLeafs()
	{
		for (tileTypeBimap::left_iterator typeIterator = tileTypes.left.begin(); typeIterator != tileTypes.left.end(); ++typeIterator)
		{
			GridPoint currentPoint = typeIterator->first;
			std::set<Direction> directionOfLeafs = forcingDirections(currentPoint);
			tileTypes.left.replace_data(typeIterator, typeAfterConnections(typeIterator->second, directionOfLeafs));	
		}
	}

	std::set<Direction> LowBendTreeBuilder::forcingDirections(GridPoint point)
	{
		std::set<Direction> result;
		std::vector<Direction> directions = allDirections();
		for (std::vector<Direction>::iterator directionIterator = directions.begin(); 
			directionIterator != directions.end(); 
			++directionIterator)	
		{
			// Loop through the neighbors in all directions
			GridPoint neighbor = pointInDirection(point, *directionIterator);

			// Check to make sure there is a tile at that point
			if (tileTypes.left.find(neighbor) != tileTypes.left.end())
			{
				const GridTileType* neighborType = tileTypes.left.find(neighbor)->second;
				if (neighborType == &leaf || 
					neighborType == &horizontalBridge ||
					neighborType == &verticalBridge)
				{
					// If the neighbor forces a connection, then mark the direction
					result.insert(*directionIterator);
				}
			}
		}
		return result;
	}

	void LowBendTreeBuilder::initializeTilesWithWalls()
	{
		// Loop through each grid point
		for (GridSet::const_iterator gridItr = remainingTiles.cbegin(); gridItr != remainingTiles.cend(); ++gridItr)
		{
			// Find out the walls surrounding the point	
			std::set<Direction> wallsForTile = directionOfWalls(*gridItr);

			// Figure out the type based on the walls
			const GridTileType* typeToInsert = tileFromWalls(wallsForTile);
			tileTypes.insert(tileTypeBimap::value_type(*gridItr, typeToInsert));
		}				
	}

	std::set<Direction> LowBendTreeBuilder::directionOfWalls(GridPoint point)
	{
		std::vector<Direction> directions = allDirections();
		std::set<Direction> wallsForTile;
		for (std::vector<Direction>::iterator directionItr = directions.begin(); directionItr != directions.end(); ++directionItr)
		{
			// If there is no adjacent tile, then there is a wall in that direction
			if (!remainingTiles.containsPoint(pointInDirection(point, *directionItr)))
				wallsForTile.insert(*directionItr);	
		}
		return wallsForTile;
	}

	const GridTileType* LowBendTreeBuilder::tileFromWalls(std::set<Direction> walls)
	{
		if (walls.size() == 0)
		{
			// Boardered by no walls, so wildcard
			return &wild;
		} 
		else if (walls.size() == 1)
		{
			// Boardered by a single wall, so a horizontal or vertical
			Direction wallDirection = *(walls.begin());

			if (wallDirection == left || wallDirection == right)
				return &vertical;
			else if (wallDirection == up || wallDirection == down)
				return &horizontal; 
		}
		else if (walls.size() == 2)
		{
			Direction firstDirection = *(walls.begin());
			Direction secondDirection = *(++walls.begin());

			if (secondDirection == rotateDirection(firstDirection, 1) ||
				secondDirection == rotateDirection(firstDirection, 3))
				return &two;
			else
			{
				if (firstDirection == left)
					return &verticalBridge;
				else
					return &horizontalBridge;
			}

		}
		else if (walls.size() == 3 || walls.size() == 4)
		{
			// The cell is a terminus
			return &leaf;
		}
	}

	const GridTileType* LowBendTreeBuilder::typeAfterConnections(const GridTileType* oldType, std::set<Direction> connections)
	{
		const GridTileType* newType = oldType;
		if (oldType == &wild)
		{
			if (connections.size() == 1)
			{
				if (connections.count(left) == 1 || connections.count(right) == 1)
					newType = &horizontal;
				else
					newType = &vertical;
			}
			else if (connections.size() == 2)
			{
				newType = &two;
			}
			else if (connections.size() == 3 || connections.size() == 4)
			{
				newType = &cross;
			}
		} 
		else if (oldType == &horizontal)
		{
			if (connections.count(up) == 1 || connections.count(down) == 1)
				newType = &two;
		}
		else if (oldType == &vertical)
		{
			if (connections.count(left) == 1 || connections.count(right) == 1)
				newType = &two; 
		}
		else
		{
			if (connections.size() == 3 || connections.size() == 4)
			{
				newType = &cross;
			}
		}
		return newType;
	}

	/*
	void LowBendTreeBuilder::printTree()
	{
		std::string output;
		int minX, minY, maxX, maxY;
		minX = minY = std::numeric_limits<int>::max();
		maxX = maxY = std::numeric_limits<int>::min();
		for (tileTypeBimap::left_const_iterator gridItr = tileTypes.left.begin(); gridItr != tileTypes.left.end(); ++gridItr)
		{
			if (gridItr->first.x > maxX)
				maxX = gridItr->first.x;	
			if (gridItr->first.x < minX)
				minX = gridItr->first.x;
			if (gridItr->first.y > maxY)
				maxY = gridItr->first.y;
			if (gridItr->first.y < minY)
				minY = gridItr->first.y;
		}

		output += "\n";
		for (int gridY = maxY; gridY >= minY; gridY--)
		{
			for (int gridX = minX; gridX <= maxX; gridX++)
			{
				if (tileTypes.left.find(GridPoint(gridX, gridY)) != tileTypes.left.end())
				{
					output += tileTypes.left.find(GridPoint(gridX, gridY))->second->representation();
				}
				else
				{
					output += " ";
				}	
			}
			output += "\n";
		}
		LOG(output);
	}
	*/
}
