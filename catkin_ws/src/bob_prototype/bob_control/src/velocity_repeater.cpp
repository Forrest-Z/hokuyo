#include <bob_control/velocity_repeater.h>


namespace bob
{
	Velocity2D VelocityRepeater::nextCommand(const ISensorHandle& sensorHandle)
	{
		return velCommand;
	}

}

