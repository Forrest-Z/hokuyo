#include <ros/ros.h>

#include <bob_config/config.h>

#include <bob_toolbox/logging.h>
#include <bob_toolbox/easy_print.h>
#include <bob_visualization/visualization.h>
#include <bob_stc/grid_set.h>
#include <bob_stc/stc_plan_tools.h>
#include <bob_stc/stc_hamiltonian.h>
#include <bob_test/example_grid_sets.h>
#include <bob_visualization/color.h>
#include <bob_toolbox/world_point_shapes.h>
#include <bob_stc/arbitrary_tree_builder.h>
#include <bob_stc/spanning_tree_grid.h>
#include <bob_stc/stc_path_combiner.h>
#include <bob_coverage/area_tools.h>
#include <bob_stc/low_bend_tree_builder.h>
#include <bob_test/example_stc_plans.h>
#include <bob_stc/visualization.h>
#include <bob_stc/easy_print.h>
#include <iostream>
#include <list>

#include <bob_stc/stc_planner.h>

#include <bob_ros_implementations/ros_map_listener.h>

using namespace bob;
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "grid_test");
	ros::NodeHandle n;
	

	

	visualizer->visualize("plan", visualization(exampleSTCPlan4()));
	
	ros::spin();

	// The following code is used to demonstrate the difference
	// between the arbitrary tree builder and the low bend tree builder
	/*

	// Used to publish markerManager to rviz
	bob::
	
	// Define marker styles
	bob::MarkerStyle gridStyle;
	gridStyle.width = 0.7;
	gridStyle.color.reset(new Green);

	bob::MarkerStyle treeStyle;
	treeStyle.width = 0.1;
	treeStyle.color.reset(new Green);

	std::vector<GridSet> gridSets = ExampleGridSets::sets();

	for (std::vector<GridSet>::iterator itr = gridSets.begin(); itr != gridSets.end(); ++itr)
	{
		// First, try with the arbitrary tree builder
		bob::ArbitraryTreeBuilder arbitraryBuilder;
		bob::SpanningTreeGrid tree = arbitraryBuilder.buildTree(*itr);
	
		// Wait for user input
		LOG_TEST("Press enter to continue");
		std::cin.ignore();

		// Try with low bend tree builder
		bob::LowBendTreeBuilder builder;
		tree = builder.buildTree(*itr);
	
		// Wait for user input
		LOG_TEST("Press enter to continue");
		std::cin.ignore();
	}	

	ros::spin();

	*/

	return 0;
}


