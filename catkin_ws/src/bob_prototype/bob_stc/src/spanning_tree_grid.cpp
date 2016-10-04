#include <bob_stc/spanning_tree_grid.h>

#include <utility>
#include <stack>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <bob_coverage/area_tools.h>
#include <bob_stc/grid_set.h>
#include <bob_stc/spanning_tree_grid_depth_return_iterator.h>
#include <bob_stc/spanning_tree_grid_depth_iterator.h>
#include <bob_toolbox/direction.h>
#include <bob_stc/grid_point.h>
#include <bob_toolbox/world_point.h>
#include <bob_stc/spanning_tree_grid_node.h>
#include <bob_stc/grid_map.h>

namespace bob
{

	SpanningTreeGrid::DepthReturnIterator SpanningTreeGrid::depthReturnIterator() const
	{
		return DepthReturnIterator(parent);
	}

	SpanningTreeGrid::DepthIterator SpanningTreeGrid::depthIterator() const
	{
		return DepthIterator(parent);
	}


	SpanningTreeGrid::SpanningTreeGrid(const Grid& source, GridPoint parent) : 
	parent(new SpanningTreeGridNode(parent)), 
	Grid(source) 
	{}

	bool SpanningTreeGrid::hasChildren() const
	{
		for(int i = 0; i < 4; i++)
		{
			if (parent->getChild((Direction)i) != 0)
				return true;
		}
		return false;
	}
}


