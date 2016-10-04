#ifndef _BOB_STC_SPANNING_TREE_GRID_DEPTH_RETURN_ITERATOR_H_
#define _BOB_STC_SPANNING_TREE_GRID_DEPTH_RETURN_ITERATOR_H_

#include <bob_toolbox/direction.h>
#include <bob_stc/spanning_tree_grid_node.h>
#include <boost/shared_ptr.hpp>

#include <stack>

namespace bob 
{

	class SpanningTreeGridNode;

	//
	//! This class iterates over a Grid::SpanningTree object
	//! in a depth first manner, with each node visited twice:
	//! once when expanding, and once when leaving.
	//
	class SpanningTreeGridDepthReturnIterator
	{

		public:


			//! Constructs the iterator from the parent node of the spanning tree.
			//! firstDirection is the first direction that will be expanded when iterating the spanning tree.
			SpanningTreeGridDepthReturnIterator(boost::shared_ptr<SpanningTreeGridNode> parentNode, Direction firstDirection=up);

			//! If true then the tree has been iterated
			bool done() const;

			//! Gets the current goal direction
			GridPoint currentPoint() const;

			//! The direction in which the iterator is trying to get us to go
			Direction directionToNext() const;

			void next();

		private:

			//! The first direction that the iterator travels
			//! when traversing the spanning tree
			//! This is needed because the parent node has no parent,
			//! so we can't use how we got there to determine end condition
			Direction firstDirection;

			boost::shared_ptr<SpanningTreeGridNode> parent;

			//! Stack of parent nodes
			std::stack<SpanningTreeGridNode*> parentNodeStack;		

			//! Current node. Will be pushed into the parent stack when it becomes a parent 
			SpanningTreeGridNode * current;

			Direction nextDirection;

			bool started;

			Direction findValidNextDirection(Direction startingDirection) const;

	};

}

#endif
