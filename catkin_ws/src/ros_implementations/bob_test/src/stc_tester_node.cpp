#include <bob_test/stc_tester.h>

#include <ros/ros.h> 

#include <bob_config/config.h> 

#include <bob_ros_implementations/ros_map_listener.h>

#include <bob_control/curved_path_executor.h>
#include <bob_control_handle/control_handle.h>

#include <bob_sensor/isensor_handle.h>


using namespace bob;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stc_tester");
	ros::NodeHandle nh;

	

	LockableMap map;

	// Importing map over ROS
	ROSMapListener costmapROS(map);

	ISensorHandle sensorHandle;
	ControlHandle controlHandle(sensorHandle, map);

	CurvedPathExecutor curvedPathExecutor(controlHandle.simpleCommander, sensorHandle.getTransformHandle());
	STCPlanExecutor stcPlanExecutor(curvedPathExecutor, controlHandle.navigationManager, map);

	STCTester stcTester(map, stcPlanExecutor);

	ros::spin();
	return 0;

}
