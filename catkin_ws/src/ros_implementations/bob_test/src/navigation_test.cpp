#include <ros/ros.h>

#include <bob_ros_implementations/ros_system_handle.h>
#include <bob_toolbox/logging.h>
#include <bob_toolbox/world_point.h>

using namespace bob;

int main(int argc, char** argv)
{
	// This test file is used to test the sub-actions of boustrophedon on the robot

	ROSSystemHandle systemHandle(argc, argv, "demo");
	
	NavigationManager::Result result = systemHandle.getControlHandle().navigationManager.navigationGoal(WorldPoint(-1, 0));

	if (result == NavigationManager::PlanningFailed)
	{
		LOG_TEST("Planning failed");
	}
	else if (result == NavigationManager::StopCondition)
	{
		LOG_TEST("Stop Condition");
	}
	else if (result == NavigationManager::ReachedGoal)
	{
		LOG_TEST("Reached Goal");
	}
	else if (result == NavigationManager::Aborted)
	{
		LOG_TEST("Aborted");
	}
	
	LOG_TEST("Done");	

	return 0;
}


