#include <ros/ros.h>

#include <bob_control/robot_remote.h>
#include <bob_ros_implementations/ros_system_handle.h>

using namespace bob;
	
void testBehavior(RobotRemote& remote)
{
	remote.basicCurve(0.5, 0.1, Clockwise);
}

int main(int argc, char** argv)
{
	// This test file is used to test the sub-actions of boustrophedon on the robot
	ROSSystemHandle systemHandle(argc, argv, "robot_remote_test");

	RobotRemote robotRemote(systemHandle.getControlHandle().simpleCommander, systemHandle.getSensorHandle());

	testBehavior(robotRemote);

	return 0;
}
