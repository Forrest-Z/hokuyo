#include <ros/ros.h>
#include <bob_toolbox/logging.h>

#include <bob_toolbox/grid_wavefront_full_iterator.h>
#include <bob_toolbox/wave_expander.h>
#include <bob_visualization/visualization.h>
#include <bob_toolbox/world_point.h>

#include <bob_grid_map/map_location_set.h>
#include <bob_grid_map/location_mapper.h>
#include <bob_visualization/bob_toolbox_visualization.h>

// To run this code, do the following:
//
// 1. Run 'roscore' in a terminal
//
// 2. Run 'rosrun bob_test arm_expander_test' in another terminal
//
// 3. Run 'rosrun rviz rviz' in another terminal to observe the visualization
//

using namespace bob;

// Functor used to stop points from expanding
class SquareFunctor
{

	public:

		SquareFunctor(float widthInMeters) : 
		pixelWidth(widthInMeters / (2 * 0.05))	
		{
		}

		inline bool operator()(MapLocation point)
		{
			// Some arbitrary bounds
			return (point.x > pixelWidth) ||
				(point.x < -pixelWidth) ||
				(point.y > pixelWidth) ||
				(point.y < -pixelWidth);
		}

		int pixelWidth;

};

int main(int argc, char** argv)
{
	// ROS stuff to setup the node
	ros::init(argc, argv, "frontier_test");
	ros::NodeHandle nh;

	// Do the wave expansion algorithm
	// This is the block that needs to be profiled
	MapLocationSet closedSet;
	MapLocation seedPoint(0, 0);
	SquareFunctor functor(5);
	GridWavefrontFullIterator<MapLocationSet, SquareFunctor> iterator(closedSet, seedPoint, functor);

	ros::Time start = ros::Time::now();
	waveExpand(iterator);
	LOG_TEST("Done. Took " << (ros::Time::now() - start).toSec());

	// Converting map points to world co-ordinates
	LocationMapper mapper(0.05);
	std::vector<WorldPoint> edgePoints = mapper.mapToWorld<std::vector<WorldPoint> >(iterator.getEdge());
	std::vector<WorldPoint> filledPoints = mapper.mapToWorld<std::vector<WorldPoint> >(closedSet);

	// ROS Visualization 
	// No need for profiling
	MarkerStyle edgeStyle(0.8, 0.04, new Red);
	MarkerStyle filledStyle(0.8, 0.04, new Blue);
	visualizer->visualize("edgePoints", MarkerSquares(edgePoints, 0.05), edgeStyle);
	visualizer->visualize("filledPoints", MarkerSquares(filledPoints, 0.05), filledStyle);

	LOG_TEST("Spin");

	LOG_TEST("Spin");

	// ROS stuff to keep visualization up
	ros::spin();
}
