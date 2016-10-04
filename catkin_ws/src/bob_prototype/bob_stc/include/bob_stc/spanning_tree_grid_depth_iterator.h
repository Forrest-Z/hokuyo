#ifndef _BOB_STC_SPANNING_TREE_GRID_DEPTH_ITERATOR_H_
#define _BOB_STC_SPANNING_TREE_GRID_DEPTH_ITERATOR_H_

#include <boost/shared_ptr.hpp>

#include <stack>

#include <bob_stc/grid_point.h>

namespace bob 
{

	class SpanningTreeGridNode;

	//
	//! This class iterates over a Grid::SpanningTree object
	//! in a depth first manner, with each node visited twice:
	//! once when expanding, and once when leaving.
	//
	class SpanningTreeGridDepthIterator
	{
		
		boost::shared_ptr<SpanningTreeGridNode> parent;

		//! Stack of parent nodes
		std::stack<SpanningTreeGridNode*> unopenedNodes;		
		
	public:


		//! Constructs the iterator from the parent node of the spanning tree.
		//! firstDirection is the first direction that will be expanded when iterating the spanning tree.
		SpanningTreeGridDepthIterator(boost::shared_ptr<SpanningTreeGridNode> parentNode);
	
		//! If true then the tree has been iterated
		bool done();

		//! Gets the current goal direction
		GridPoint currentPoint();

		void next();

		SpanningTreeGridNode* currentNode();
	};

}

#endif
