#include <ros/ros.h>
#include <bob_sensor/itransform_handle.h>
#include <bob_sensor/isensor_handle.h>
#include <bob_sensor/iodometry_handle.h>
#include <bob_toolbox/logging.h>

#include <bob_control/simple_commander.h>
#include <bob_control/curve_planner.h>
#include <bob_control/conditions/pointing_to_goal_condition.h>
#include <bob_control/curved_path_executor.h>
#include <bob_control/curve_heading.h>

#include <bob_toolbox/geometry.h>
#include <bob_toolbox/line_geometry.h>
#include <bob_toolbox/line.h>
#include <bob_toolbox/circular_direction.h>
#include <bob_visualization/bob_toolbox_visualization.h>

#include <bob_grid_map/map_sender.h>
#include <bob_ros_implementations/ros_map_sender.h>
#include <bob_grid_map/toolbox.h>

#include <bob_ros_implementations/ros_sensor_handle.h>
#include <bob_ros_implementations/ros_velocity_publisher.h>
#include <bob_visualization/visualization.h>
#include <bob_slam/slam_ros.h>
#include <bob_config/config.h>
#include <bob_ros_implementations/ros_system_handle.h>

using namespace bob;

std::vector<WorldPoint> getPath(int numOfPath)
{
	// curve path test
	std::vector<WorldPoint> path;

	switch(numOfPath)
	{
		case 1:
			// test 1: actualProximity, distanceToGoal < headingDist, distanceToGoal <= proximity: curve on to point
			// distanceToGoal > proximity: heading rotation, drive and curve to point
			path.push_back(WorldPoint(0.01, 0.01));
			path.push_back(WorldPoint(0.0, 1.0));
			path.push_back(WorldPoint(-0.1, 1.0));
			path.push_back(WorldPoint(-0.1, 0.0));
			path.push_back(WorldPoint(-0.2, 0.0));
			path.push_back(WorldPoint(-0.2, 1.0));
			break;
		
		case 2:
			// test 2: distanceToGoal > proximity: drive and rotate to point
			// distanceToGoal <= proximity: heading rotatin, rotating to point
			path.push_back(WorldPoint(1.0, 0.15));
			path.push_back(WorldPoint(0.0, 0.15));
			path.push_back(WorldPoint(0.0, 0.25));
			path.push_back(WorldPoint(-0.1, -0.5));
			break;
		case 3:{
				// test 3: rectangle test
				float width = 1.0;
				float height = 0.3;
				path.push_back(WorldPoint(width, 0));
				path.push_back(WorldPoint(width, height));
				path.push_back(WorldPoint(0, height));
				path.push_back(WorldPoint(0, 0));
				break;
			}
		case 4:
			// test 4: adjust heading test
			path.push_back(WorldPoint(-1, 0));
			path.push_back(WorldPoint(-1, 0.3)); 
			break;
	}

	return path;
}

void executePath(const std::vector<WorldPoint>& path, IPathExecutor& pathExecutor)
{
	// Execute normal path
	float proximity = 0.2;
	pathExecutor.run(path);
}


void executePathRecursively(const std::vector<WorldPoint>& path, IPathExecutor& pathExecutor)
{
	float proximity = 0.2;
	
	while(true)
	{
		pathExecutor.run(path);
	}
}


int main(int argc, char** argv)
{
	ROSSystemHandle systemHandle(argc, argv, "curve_test");

	CurvedPathExecutor pathExecutor(systemHandle.getControlHandle().simpleCommander, systemHandle.getSensorHandle().getTransformHandle());

	ros::Duration(10.0).sleep();
	
	std::vector<WorldPoint> path = getPath(3);
	visualizer->visualize("path", MarkerLine(path));

	executePathRecursively(path, pathExecutor);
	LOG_TEST("finish");
	
	return 0;
}

/*
// Curve heading test

	float radius = 0.3;
	
	Line line(WorldPoint(radius, 0.0), unitVector(M_PI / 2));
	std::vector<WorldPoint> lineVect;
	lineVect.push_back(line.origin);
	lineVect.push_back(line.origin + line.vector);

	CurveHeading curveHeading(line, radius);

	float y = 0.0;
	float x = 0.0;
	while(x <= 2 * radius)
	{
		WorldPoint center(x, radius + y);
		float angle = -M_PI;
		while(angle < 0)
		{
			WorldPoint point = center + radius * unitVector(angle);
			float heading = curveHeading.desiredHeading(point);
			
			std::vector<WorldPoint> robotHeading;
			robotHeading.push_back(point);
			robotHeading.push_back(point + 0.2 * unitVector(heading));


			ros::Duration(2.0).sleep();
			angle += M_PI / 6;
		}
		x += 2 * radius;
	}	
*/

