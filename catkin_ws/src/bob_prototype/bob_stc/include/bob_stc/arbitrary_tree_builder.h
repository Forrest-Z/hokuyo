#ifndef _BOB_STC_ARBITRARY_TREE_BUILDER_H_
#define _BOB_STC_ARBITRARY_TREE_BUILDER_H_

#include <bob_stc/tree_builder.h>
#include <bob_stc/grid_set.h>

namespace bob
{

class SpanningTreeGrid;
class GridPoint;
enum Direction;

class ArbitraryTreeBuilder : public TreeBuilder
{

public:

	virtual SpanningTreeGrid buildTree(GridSet freeCells);

private:

	GridPoint chooseParent(const GridSet& source) const;

	bool expandTree(GridSet& source, SpanningTreeGrid& toExpand, Direction direction) const;

};

}

#endif
