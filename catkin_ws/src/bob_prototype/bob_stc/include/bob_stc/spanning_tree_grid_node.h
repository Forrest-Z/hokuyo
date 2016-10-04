#ifndef _BOB_STC_SPANNING_TREE_GRID_NODE_H_
#define _BOB_STC_SPANNING_TREE_GRID_NODE_H_

#include <boost/shared_ptr.hpp>

#include <bob_stc/grid_point.h>

namespace bob 
{

	//
	//! Node defines a node in the spanning tree.
	//
	class SpanningTreeGridNode
	{

			//! The location of this node, in grid co-ordinates.
			GridPoint p;

			//! Pointers to the children of this node.
			boost::shared_ptr<SpanningTreeGridNode> children[4];

		public:

			//! Constructs a new Node object with no children.
			SpanningTreeGridNode(GridPoint point);

			//! Creates a new Node at the specified location and
			//! adds it as a child.
			//
			//! Input:
			//! point - Location of new child
			SpanningTreeGridNode* addChild(GridPoint point);
			
			//! Gets the pointer to a child at a certain index in the spanning tree.
			//
			//! Input:
			//! index - The index of the child to get
			//
			//! Output:
			//! Pointer to the child at that index
			SpanningTreeGridNode* getChild(Direction index);
			
			const SpanningTreeGridNode* getChild(Direction index) const;

			//! Returns the location of this node
			GridPoint point() const;
			
	};
}

#endif
