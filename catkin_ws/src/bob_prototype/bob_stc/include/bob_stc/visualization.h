#ifndef _BOB_STC_VISUALIZATION_H_
#define _BOB_STC_VISUALIZATION_H_

#include <vector>

#include <bob_visualization/marker_types.h>

#include <bob_stc/stc_plan.h>
#include <bob_stc/spanning_tree_grid.h>

namespace bob
{

	MarkerLine visualization(const STCPlan& plan);

	MarkerLineList visualization(const SpanningTreeGrid& spanningTreeGrid);
	
}

#endif
