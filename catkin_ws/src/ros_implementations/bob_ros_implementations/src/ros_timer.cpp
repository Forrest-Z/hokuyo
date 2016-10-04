#include <bob_ros_implementations/ros_timer.h>

namespace bob
{

	void ROSTimer::startTimer()
	{
		startTime = ros::Time::now();
	}

	float ROSTimer::timeElapsed()
	{
		return (ros::Time::now() - startTime).toSec();
	}

}

