#include <bob_stc/spanning_tree_grid_node.h>

#include <boost/shared_ptr.hpp>
#include <bob_stc/grid_point.h>


namespace bob 
{

	// Locates the point and sets all children to null
	SpanningTreeGridNode::SpanningTreeGridNode(GridPoint point) : 
		p(point) {}

	SpanningTreeGridNode* SpanningTreeGridNode::addChild(GridPoint point)
	{
		int indexToAdd = directionBetween(p, point);
		SpanningTreeGridNode* node = 0;
		if (indexToAdd != invalid){
			children[indexToAdd] = boost::shared_ptr<SpanningTreeGridNode>(new SpanningTreeGridNode(point));
			node = children[indexToAdd].get();
		}
		return node;
	}

	SpanningTreeGridNode* SpanningTreeGridNode::getChild(Direction index)
	{
		if (index == invalid)
			return 0;
		else
			return children[index].get();
	}

	const SpanningTreeGridNode* SpanningTreeGridNode::getChild(Direction index) const
	{
		if (index == invalid)
			return 0;
		else
			return children[index].get();
	}

	GridPoint SpanningTreeGridNode::point() const
	{
		return p;
	}

}
