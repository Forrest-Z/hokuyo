#ifndef _BOB_STC_SPANNING_TREE_GRID_H_
#define _BOB_STC_SPANNING_TREE_GRID_H_

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <bob_stc/grid.h>
#include <bob_stc/grid_set.h>
#include <bob_toolbox/direction.h>
#include <bob_stc/grid_point.h>
#include <bob_toolbox/world_point.h>
#include <bob_stc/spanning_tree_grid_node.h>
#include <bob_stc/spanning_tree_grid_depth_return_iterator.h>
#include <bob_stc/spanning_tree_grid_depth_iterator.h>

#include <bob_stc/grid_map.h>

namespace bob 
{

	//! //! SpanningTree defines a spanning tree where all the points are aligned with a grid.
	//! Instances of this class are produced by passing in other Graph objects, notably ConnectedGraph
	//
	class SpanningTreeGrid : public Grid
	{

		public:

			typedef SpanningTreeGridDepthReturnIterator DepthReturnIterator;
			DepthReturnIterator depthReturnIterator() const;

			typedef SpanningTreeGridDepthIterator DepthIterator;
			DepthIterator depthIterator() const;

			//! Constructor which creates a spanning tree from a ConnectedGraph.
			//
			//! Input:
			//! source - ConnectedGraph from which to build the spanning tree
			//! parentNode - The point from which to build the spanning tree
			//SpanningTreeGrid(const GridSet& source, GridPoint parentNode);
			SpanningTreeGrid(const Grid& source, GridPoint parent);

			bool hasChildren() const;

			GridPoint parentPoint() const { return parent->point(); }

		private:

			//! The parent node of the tree.
			//! This node also "owns" the other nodes
			boost::shared_ptr<SpanningTreeGridNode> parent;

	};
}

#endif
