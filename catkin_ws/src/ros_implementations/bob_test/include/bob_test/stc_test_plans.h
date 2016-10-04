#ifndef _BOB_TEST_STC_TEST_PLANS_H_
#define _BOB_TEST_STC_TEST_PLANS_H_

#include <bob_stc/stc_path.h>
#include <bob_stc/stc_path_combiner.h>
#include <bob_stc/stc_hamiltonian.h>
#include <bob_stc/low_bend_tree_builder.h>
#include <bob_stc/grid_set.h>


namespace bob
{
	//! This class generates stc plans for test.
	
	class STCTestPlans
	{

		public:

			//! Constructs a STC test plan object. This function will generate the plans to be test.
			STCTestPlans();

			//! This function will return the id-th plan in generated test plans.
			STCPlan getPlan(const unsigned id) const;

			//! This function will return the size of the generated test plans.
			size_t size() const;

		private:
		
			//! Generate a plan for the given GridSets.
			STCPlan generate(const std::vector<GridSet>& gridSets);

			//! Return a vector of simple rectangle GridPoints with the given width, height and the origin.
			std::vector<GridPoint> getSimpleGridPoints(const int width, const int height, const WorldPoint& origin);

			//! Generate a GridSet from the given GridPoints with the given origin, orientation.
			GridSet getGridSet(const WorldPoint& origin, const float orientation, const std::vector<GridPoint>& gridPoints) const;

			//! Generate simple plans and add them to test plans.
			void getSimpleAreaPlans();

			//! Generate complex plans and add them to test plans.
			void getComplexPlan();

			std::vector<STCPlan> plans;

			STCPathCombiner combiner;
			STCHamiltonian hamiltonian;
			LowBendTreeBuilder treeBuilder;
			
	};

}

#endif
