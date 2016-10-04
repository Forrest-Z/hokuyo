#include <bob_stc/spanning_tree_grid_depth_iterator.h>

#include <bob_stc/spanning_tree_grid_node.h>
#include <boost/shared_ptr.hpp>

namespace bob
{

	SpanningTreeGridDepthIterator::SpanningTreeGridDepthIterator(boost::shared_ptr<SpanningTreeGridNode> parentNode) :
	parent(parentNode)
	{
		unopenedNodes.push(parent.get());
	}

	GridPoint SpanningTreeGridDepthIterator::currentPoint()
	{
		return unopenedNodes.top()->point();
	} 

	void SpanningTreeGridDepthIterator::next()
	{
		SpanningTreeGridNode * currentNode = unopenedNodes.top();
		unopenedNodes.pop();

		SpanningTreeGridNode * leftChild = currentNode->getChild(left);
		SpanningTreeGridNode * upChild = currentNode->getChild(up);
		SpanningTreeGridNode * rightChild = currentNode->getChild(right);
		SpanningTreeGridNode * downChild = currentNode->getChild(down);
		
		if (leftChild != 0)
			unopenedNodes.push(leftChild);
		
		if (rightChild != 0)
			unopenedNodes.push(rightChild);
		
		if (upChild != 0)
			unopenedNodes.push(upChild);
		
		if (downChild != 0)
			unopenedNodes.push(downChild);

	}

	bool SpanningTreeGridDepthIterator::done()
	{
		return unopenedNodes.empty();
	}	
	
	SpanningTreeGridNode* SpanningTreeGridDepthIterator::currentNode()
	{
		return unopenedNodes.top();
	}

}
