#include <bob_visualization/visualization.h>

#include <vector>
#include <cmath>

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/logging.h>

#include <bob_system/system_utilities.h>
#include <bob_ros_implementations/ros_system_utilities.h>

#include <ros/ros.h>

#include <bob_ros_implementations/ros_visualizer.h>

using namespace bob;

std::vector<WorldPoint> pointSpiral(int numberOfPoints)
{	
	std::vector<WorldPoint> toReturn;
	float totalAngle = 4 * M_PI;
	float angleIncrement = totalAngle / (float)numberOfPoints;
	for (int i = 0; i < numberOfPoints; i++)
	{
		float angle = angleIncrement * i;
		float radius = exp(0.3 * angle);
		WorldPoint newPoint = radius * unitVector(angle);
		toReturn.push_back(newPoint);
	}
	return toReturn;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "marker_test");
	ros::NodeHandle n;

	configureROSVisualization();
	configureROSSystemUtilities();

	std::vector<WorldPoint> linePoints = pointSpiral(500);

	visualizer->visualize("Test", MarkerEllipse(Pose2D(WorldPoint(2, 1), M_PI / 4), Dimension2D<float>(1, 2.0)));
	LOG_TEST("Press Enter");
	std::cin.ignore();

	visualizer->visualize("Test", MarkerArrow(Pose2D(WorldPoint(2, 1), M_PI / 4)));
	LOG_TEST("Press Enter");
	std::cin.ignore();

	visualizer->visualize("Test", MarkerCircle(WorldPoint(0, 0), 1));
	LOG_TEST("Press Enter");
	std::cin.ignore();

	visualizer->visualize("Test", MarkerSquares(linePoints, 0.05));
	LOG_TEST("Press Enter");
	std::cin.ignore();

	visualizer->visualize("Test", MarkerLine(linePoints));
	LOG_TEST("Press Enter");
	std::cin.ignore();

	visualizer->visualize("Test", MarkerLineList(linePoints));
	LOG_TEST("Press Enter");
	std::cin.ignore();

	LOG_TEST("Done");

	ros::spin();

}
