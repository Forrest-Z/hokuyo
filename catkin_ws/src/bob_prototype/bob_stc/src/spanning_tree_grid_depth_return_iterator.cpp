#include <bob_stc/spanning_tree_grid_depth_return_iterator.h>

#include <bob_stc/spanning_tree_grid.h>
#include <bob_toolbox/direction.h>
#include <bob_stc/spanning_tree_grid_node.h>
#include <boost/shared_ptr.hpp>

namespace bob 
{

	SpanningTreeGridDepthReturnIterator::SpanningTreeGridDepthReturnIterator(boost::shared_ptr<SpanningTreeGridNode> parentNode, Direction initialDirection) :
		parent(parentNode),
		current(parent.get()),
		started(false)
	{
		nextDirection = firstDirection = findValidNextDirection(initialDirection);	
	}

	Direction SpanningTreeGridDepthReturnIterator::directionToNext() const
	{
		return nextDirection;
	}

	bool SpanningTreeGridDepthReturnIterator::done() const
	{
		return (current == 0);
	}

	GridPoint SpanningTreeGridDepthReturnIterator::currentPoint() const
	{
		return current->point();
	}

	void SpanningTreeGridDepthReturnIterator::next()
	{
		Direction searchDirection;
		if (started && parentNodeStack.empty() && (nextDirection == firstDirection)) 
		{
			current = 0;
		}
		else if (current->getChild(nextDirection))
		{	
			parentNodeStack.push(current);
			current = current->getChild(nextDirection);

			searchDirection = rotateDirection(nextDirection, 3);
			nextDirection = findValidNextDirection(searchDirection);			
		}
		else if (nextDirection == directionBetween(currentPoint(), parentNodeStack.top()->point()))
		{
			// We are going back up to a parent			
			current = parentNodeStack.top();
			parentNodeStack.pop();

			searchDirection = rotateDirection(nextDirection, 3);
			nextDirection = findValidNextDirection(searchDirection);			
		}
		started = true;
	}

	Direction SpanningTreeGridDepthReturnIterator::findValidNextDirection(Direction startingDirection) const
	{
		bool foundNext = false;
		Direction searchDirection = startingDirection;
		Direction directionToParent = invalid;

		if (!parentNodeStack.empty())
			directionToParent = directionBetween(current->point(), parentNodeStack.top()->point());
		while (!(current->getChild(searchDirection)) && !(searchDirection == directionToParent))
		{ 
			searchDirection = rotateDirection(searchDirection, 1);
		}
		return searchDirection;
	}
}
