#include <bob_test/example_grid_sets.h>

#include <vector>
#include <bob_stc/grid_set.h>
#include <bob_stc/grid_point.h>

namespace bob
{

	std::vector<GridSet> ExampleGridSets::setVector;

	void ExampleGridSets::init()
	{
		GridSet tempSet;
		tempSet.addPoint(GridPoint(0, 0));
		tempSet.addPoint(GridPoint(1, 0));
		tempSet.addPoint(GridPoint(2, 0));

		tempSet.addPoint(GridPoint(0, 1));
		tempSet.addPoint(GridPoint(1, 1));
		tempSet.addPoint(GridPoint(2, 1));

		tempSet.addPoint(GridPoint(2, 2));
		setVector.push_back(tempSet);

		tempSet.addPoint(GridPoint(3, 2));
		setVector.push_back(tempSet);

		tempSet.addPoint(GridPoint(0, 2));
		tempSet.addPoint(GridPoint(1, 2));
		setVector.push_back(tempSet);

		tempSet.addPoint(GridPoint(2, 3));
		setVector.push_back(tempSet);

		tempSet.addPoint(GridPoint(1, 3));

		tempSet.addPoint(GridPoint(2, 4));
		tempSet.addPoint(GridPoint(3, 4));
		tempSet.addPoint(GridPoint(4, 4));

		tempSet.addPoint(GridPoint(2, 5));
		tempSet.addPoint(GridPoint(3, 5));
		tempSet.addPoint(GridPoint(4, 5));
		setVector.push_back(tempSet);

		tempSet = GridSet();
		tempSet.addPoint(GridPoint(0, 0));
		tempSet.addPoint(GridPoint(1, 0));
		tempSet.addPoint(GridPoint(2, 0));
		tempSet.addPoint(GridPoint(3, 0));
		tempSet.addPoint(GridPoint(4, 0));
		tempSet.addPoint(GridPoint(5, 0));

		tempSet.addPoint(GridPoint(1, 1));
		tempSet.addPoint(GridPoint(2, 1));
		tempSet.addPoint(GridPoint(3, 1));
		tempSet.addPoint(GridPoint(4, 1));

		tempSet.addPoint(GridPoint(2, 2));
		tempSet.addPoint(GridPoint(3, 2));
		setVector.push_back(tempSet);

		// "Big H"
		tempSet = GridSet();
		tempSet.addPoint(GridPoint(0, 0));

		tempSet.addPoint(GridPoint(1, -2));
		tempSet.addPoint(GridPoint(1, -1));
		tempSet.addPoint(GridPoint(1, 0));
		tempSet.addPoint(GridPoint(1, 1));
		tempSet.addPoint(GridPoint(1, 2));

		tempSet.addPoint(GridPoint(2, -2));
		tempSet.addPoint(GridPoint(2, -1));
		tempSet.addPoint(GridPoint(2, 0));
		tempSet.addPoint(GridPoint(2, 1));
		tempSet.addPoint(GridPoint(2, 2));

		tempSet.addPoint(GridPoint(-1, -2));
		tempSet.addPoint(GridPoint(-1, -1));
		tempSet.addPoint(GridPoint(-1, 0));
		tempSet.addPoint(GridPoint(-1, 1));
		tempSet.addPoint(GridPoint(-1, 2));

		tempSet.addPoint(GridPoint(-2, -2));
		tempSet.addPoint(GridPoint(-2, -1));
		tempSet.addPoint(GridPoint(-2, 0));
		tempSet.addPoint(GridPoint(-2, 1));
		tempSet.addPoint(GridPoint(-2, 2));
		setVector.push_back(tempSet);
	
		// + with loop
		tempSet = GridSet();

		// The +
		tempSet.addPoint(GridPoint(0, 0));
		tempSet.addPoint(GridPoint(1, 0));
		tempSet.addPoint(GridPoint(-1, 0));
		tempSet.addPoint(GridPoint(0, 1));
		tempSet.addPoint(GridPoint(0, -1));

		// The loop
		tempSet.addPoint(GridPoint(0, 2));
		tempSet.addPoint(GridPoint(1, 2));
		tempSet.addPoint(GridPoint(2, 2));
		tempSet.addPoint(GridPoint(3, 2));
		tempSet.addPoint(GridPoint(3, 1));

		tempSet.addPoint(GridPoint(3, 0));

		tempSet.addPoint(GridPoint(0, -2));
		tempSet.addPoint(GridPoint(1, -2));
		tempSet.addPoint(GridPoint(2, -2));
		tempSet.addPoint(GridPoint(3, -2));
		tempSet.addPoint(GridPoint(3, -1));
		setVector.push_back(tempSet);
	}

	const std::vector<GridSet>& ExampleGridSets::sets()
	{
		if (setVector.size() == 0)
			init();
		return setVector;
	}

}
