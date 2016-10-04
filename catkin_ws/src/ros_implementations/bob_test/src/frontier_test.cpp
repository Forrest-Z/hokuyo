#include <ros/ros.h>
#include <bob_sensor/itransform_handle.h>
#include <bob_ros_implementations/ros_map_listener.h>
#include <bob_visualization/visualization.h>
#include <bob_control/straight_driver.h>
#include <bob_map_algorithms/closest_clear_point.h>


#include <bob_control/conditions/pointing_to_goal_condition.h>
#include <bob_sensor/iodometry_handle.h>
#include <bob_control/simple_commander.h>
#include <bob_control/controller_runner.h>
#include <bob_config/config.h>

using namespace bob;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "frontier_test");
	ros::NodeHandle nh;
        

	ROSMapListener inflatedMap;

	ITransformHandle transformHandle;

	OdometryHelper odometryHelper;

	SimpleCommander simpleCommander(transformHandle, odometryHelper);	
//	StraightDriver straightDriver(transformHandle, simpleCommander);
	CurvedPathExecutor pathExecutor(simpleCommander, transformHandle);

	ros::Duration(10.0).sleep();
	
//	ClosestClearPoint closestClearPoint(inflatedMap.getCostmap());
//	WorldPoint test = closestClearPoint.getClosestClearPoint(transformHandle.getLocalizedPose());
//	

//	ros::spin();
/*
	while(true)
	{
		straightDriver.driveGoal(WorldPoint(1,0));
		straightDriver.driveGoal(WorldPoint(0,0));
	}
*/		
}
