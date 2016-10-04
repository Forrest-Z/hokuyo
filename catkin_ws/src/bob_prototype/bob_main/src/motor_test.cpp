#include <bob_grid_map/lockable_map.h>

#include <bob_freertos_implementations/freertos_velocity_publisher.h>
#include <bob_freertos_implementations/freertos_sensor_handle.h>

#include <bob_control/simple_commander.h>
#include <bob_control/robot_remote.h>

using namespace bob;

int main(int argc, char** argv)
{
	LockableMap lockableMap;
	FreeRTOSVelocityPublisher velocityPublisher;
	FreeRTOSSensorHandle sensorHandle;

	SimpleCommander simpleCommander(sensorHandle, velocityPublisher, lockableMap);

	RobotRemote remote(simpleCommander, sensorHandle);
	
	for (int i = 0; i < 8; i++)
	{
		remote.goForward(0.5);
		remote.headingRotation(M_PI / 4);
	}
	
	return 0;
}
