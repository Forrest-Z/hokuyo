#include <ros/ros.h>

#include <bob_control/commands/track_line_command.h>
#include <bob_control/commands/curve_command.h>
#include <bob_control/commands/heading_rotation_command.h>
#include <bob_control/curved_path_executor.h>
#include <bob_control/robot_remote.h>
#include <bob_toolbox/logging.h>

#include <bob_toolbox/line.h>
#include <bob_toolbox/angles.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_navigation/navigation_manager.h>
#include <bob_visualization/visualization.h>
#include <bob_ros_implementations/ros_system_handle.h>

using namespace bob;

void headingRotationTest(SimpleCommander& simpleCommander)
{

	LOG_TEST("Heading Rotation test");
	float angle = 0;	

	std::vector<float> angles;
	angles.push_back(angle += 0.01);
	angles.push_back(angle += 1.0);
	angles.push_back(angle += 0.02);
	angles.push_back(angle += 1.0);

	for(std::vector<float>::const_iterator itr = angles.begin(); itr != angles.end(); ++itr)
	{
		LOG_TEST(toDegree(*itr) << " degree rotation start");
		HeadingRotationCommand command(*itr);
		simpleCommander.execute(command);
		LOG_TEST(toDegree(*itr) << " degree rotation done");
		ros::Duration(1.0).sleep();
	}
}


void trackLineTest(SimpleCommander& simpleCommander)
{
	LOG_TEST("Track line test");
	TrackLineCommand trackLineCommand(Line(WorldPoint(0, 0), WorldVector(0.5, 0)), WorldPoint(2, 0), 0.1, 0.02);
	simpleCommander.execute(trackLineCommand);
}

void curveCommandTest(SimpleCommander& simpleCommander)
{
	LOG_TEST("Curve command test");
	Line toCurveTo(WorldPoint(0.5 ,0), WorldVector(0, 0.5));
	CurvePlanner planner(toCurveTo, Pose2D(WorldPoint(0,0), 0));
	CurveCommand curveCommand(planner.getCurve());
	simpleCommander.execute(curveCommand);
}

void curveRemoteTest(RobotRemote& robotRemote)
{
        Line toCurveTo(WorldPoint(-0.5, 0), WorldVector(0, 1));
	std::cin.ignore();

        // visualization
        std::vector<WorldPoint> line;
        line.push_back(toCurveTo.origin);
        line.push_back(toCurveTo.origin + toCurveTo.vector);

        visualizer->visualize("test_line", MarkerLine(line));
	robotRemote.curveOnToLine(toCurveTo);

}

void navigationTest(LockableMap& lockableMap, SimpleCommander& simpleCommander, const ISensorHandle& sensorHandle)
{
	CurvedPathExecutor pathExecutor(simpleCommander, sensorHandle.getTransformHandle());
	NavigationManager navigationManager(pathExecutor, lockableMap, simpleCommander, sensorHandle);

	NavigationManager::Result result = navigationManager.navigationGoal(WorldPoint(-0.5, -0.3));
	
	LOG_TEST("result: " << result);

}


int main(int argc, char** argv)
{
	// This test file is used to test the sub-actions of boustrophedon on the robot
	ROSSystemHandle systemHandle(argc, argv, "commands_test");
	
	RobotRemote robotRemote(systemHandle.getControlHandle().simpleCommander, systemHandle.getSensorHandle());
	//headingRotationTest(simpleCommander);
	trackLineTest(systemHandle.getControlHandle().simpleCommander);
	//curveCommandTest(simpleCommander);
	//curveRemoteTest(robotRemote);
	//navigationTest(lockableMap, simpleCommander, sensorHandle);
	robotRemote.goForward(0.2);

	LOG_TEST("Test finished");

	return 0;
}
