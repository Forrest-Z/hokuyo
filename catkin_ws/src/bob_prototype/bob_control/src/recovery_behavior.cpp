#include <bob_control/recovery_behavior.h>

#include <bob_control/simple_commander.h>
#include <bob_sensor/isensor_handle.h>
#include <bob_control/robot_remote.h>


namespace bob
{

	void recoverFromBumperEvent(SimpleCommander& simpleCommander, const ISensorHandle& sensorHandle)
	{
		if (canBackUp(sensorHandle))
		{
			RobotRemote remote(simpleCommander, sensorHandle);
			remote.backUp(0.05);
		}
	}

	bool canBackUp(const ISensorHandle& sensorHandle)
	{
		return true;
	}

}

