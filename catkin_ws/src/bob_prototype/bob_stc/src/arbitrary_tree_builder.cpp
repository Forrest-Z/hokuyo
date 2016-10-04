#include <bob_stc/arbitrary_tree_builder.h>

#include <bob_stc/spanning_tree_grid.h>
#include <bob_stc/grid_set.h>
#include <bob_stc/grid_point.h>
#include <bob_stc/spanning_tree_grid_depth_iterator.h>

namespace bob
{

typedef SpanningTreeGridDepthIterator DepthIterator;

SpanningTreeGrid ArbitraryTreeBuilder::buildTree(GridSet freeCells)
{

	if (freeCells.size() == 0)
	{
		// Throw exception
	}

	GridPoint parentNodePoint = chooseParent(freeCells);
	freeCells.removePoint(parentNodePoint);
	SpanningTreeGrid result(freeCells, parentNodePoint);
	
	bool found;
	do
	{
		found = expandTree(freeCells, result, up) || 
				expandTree(freeCells, result, down) || 
				expandTree(freeCells, result, left) ||
				expandTree(freeCells, result, right);
	} while (found);
	return result;
}

bool ArbitraryTreeBuilder::expandTree(GridSet& source, SpanningTreeGrid& toExpand, Direction direction) const
{
	bool added = false;
	
	DepthIterator iterator = toExpand.depthIterator();

	while (!iterator.done())
	{
		GridPoint currentPoint = iterator.currentPoint();
		GridPoint candidatePoint = pointInDirection(currentPoint, direction);
		if (source.containsPoint(candidatePoint))
		{
			iterator.currentNode()->addChild(candidatePoint);
			source.removePoint(candidatePoint);
			added = true;	
		}
		iterator.next();
	} 
	return added;
}

GridPoint ArbitraryTreeBuilder::chooseParent(const GridSet& source) const
{
	return source.getArbitraryPoint();
}

}
