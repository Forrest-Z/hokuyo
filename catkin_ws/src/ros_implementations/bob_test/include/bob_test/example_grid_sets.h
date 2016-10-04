#ifndef _BOB_TEST_EXAMPLE_GRID_SETS_H_
#define _BOB_TEST_EXAMPLE_GRID_SETS_H_

#include <vector>
#include <bob_stc/grid_set.h>

namespace bob
{

class ExampleGridSets
{

	static std::vector<GridSet> setVector;

	static void init();

public:

	static const std::vector<GridSet>& sets();

};

}

#endif
