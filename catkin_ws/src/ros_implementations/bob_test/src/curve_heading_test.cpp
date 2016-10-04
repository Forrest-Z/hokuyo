#include <ros/ros.h>

#include <bob_toolbox/logging.h>
#include <bob_toolbox/easy_print.h>

#include <bob_control/curve_tools.h>
#include <bob_control/curve_heading.h>

#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_visualization/visualization.h>

using namespace bob;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle nh;

	Line toCurveTo(WorldPoint(1, 0), WorldVector(1, 1));
	Pose2D robotPose(WorldPoint(1, 1), 3* M_PI / 4);
	
	// visualization
	std::vector<WorldPoint> line;
	line.push_back(toCurveTo.origin);
	line.push_back(toCurveTo.origin + toCurveTo.vector);
	
	visualizer->visualize("test_line", MarkerLine(line));
	visualizer->visualize("robot_pose", robotPose);

	CurvePlanner curvePlanner(toCurveTo, robotPose);
	CurvePlanner::CurveSpecifics curve = curvePlanner.getCurve();
	LOG_TEST("radius: " << curve.radius);

	CurveHeading curveHeading(toCurveTo, curve.radius);
	
	WorldPoint test1(1, 1);
	WorldPoint test2(0, 2);
	WorldPoint test3(-1,3);
	
	LOG_TEST("point" << test1 << ":" << curveHeading.desiredHeading(test1) * 180 / M_PI);
	LOG_TEST("point" << test2 << ":" << curveHeading.desiredHeading(test2) * 180 / M_PI);
	LOG_TEST("point" << test3 << ":" << curveHeading.desiredHeading(test3) * 180 / M_PI);
	
	ros::spin();
}



