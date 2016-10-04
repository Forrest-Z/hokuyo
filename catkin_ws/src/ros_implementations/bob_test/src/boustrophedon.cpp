#include <bob_toolbox/logging.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_visualization/marker_types.h>
#include <bob_grid_map/toolbox.h>

#include <bob_boustrophedon/boustrophedon.h>

#include <bob_visualization/visualization.h>

#include <bob_ros_implementations/ros_system_handle.h>

using namespace bob;

int main(int argc, char** argv)
{
	// This test file is used to test the sub-actions of boustrophedon on the robot

	ROSSystemHandle rosSystemHandle(argc, argv, "demo");
	
	Boustrophedon boustrophedon(rosSystemHandle.getControlHandle(), rosSystemHandle.getSensorHandle());

	WorldRectangle rectangle;
	rectangle.bounds = Dimension2D<float>(1, 1);
	std::vector<WorldPoint> worldRect = rectangle.getPoints();
	worldRect.push_back(worldRect[0]);
	visualizer->visualize("rectangle", MarkerLine(worldRect), greenMarkerStyle());

	DiscreteArea area(0.05); 
	std::vector<MapLocation> areaPoints = convexFillCells(area.worldToMap<std::vector<MapLocation> >(rectangle.getPoints()));
	area.insert(areaPoints);

	LOG_TEST("Covering area");
	boustrophedon.coverArea(area);
	
	LOG_TEST("Done");	

	return 0;
}


