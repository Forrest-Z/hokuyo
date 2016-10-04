#include <bob_test/example_stc_plans.h>

#include <cmath>

#include <bob_stc/stc_path.h>
#include <bob_stc/stc_path_combiner.h>
#include <bob_stc/stc_hamiltonian.h>
#include <bob_stc/low_bend_tree_builder.h>

#include <bob_stc/grid_set.h>

namespace bob
{

	STCPlan simplePathWithAlternate(float alternateOffset, float alternateOrientation)
	{

		STCPathCombiner combiner;
		STCHamiltonian hamiltonian;
		LowBendTreeBuilder treeBuilder;

		float gridWidth = 0.7;

		GridSet main(WorldPoint(0, 0), 0, gridWidth);
		main.addPoint(GridPoint(0, 0));
		main.addPoint(GridPoint(1, 0));
		main.addPoint(GridPoint(2, 0));

		float simplePathOffsetFactor = (1.0 / 4.0);
		float alternatePathOffsetFactor = std::max(fabs(sin(alternateOrientation + M_PI / 4)), fabs(sin(alternateOrientation + 3 * M_PI / 4))) * (sqrt(2) / 4.0);

		// Amount required for intersection
		WorldPoint altGridOrigin(1, -gridWidth * (simplePathOffsetFactor + alternatePathOffsetFactor));
		altGridOrigin.y -= alternateOffset;

		GridSet alt(altGridOrigin, alternateOrientation, gridWidth);
		alt.addPoint(GridPoint(0, 0));

		std::list<STCPath> paths;
		paths.push_back(hamiltonian.generatePath(treeBuilder.buildTree(main)));
		paths.push_back(hamiltonian.generatePath(treeBuilder.buildTree(alt)));

		STCPlan plan = combiner.combinePaths(paths);

		return plan;
	}

	STCPlan exampleSTCPlan1()
	{
		return simplePathWithAlternate(0, M_PI / 4);
	}

	STCPlan exampleSTCPlan2()
	{
		return simplePathWithAlternate(0, 0.1);;
	}

	STCPlan exampleSTCPlan3()
	{
		return simplePathWithAlternate(0.2, M_PI / 4);
	}

	STCPlan exampleSTCPlan4()
	{
		return simplePathWithAlternate(0.2, 0.1);
	}
}

