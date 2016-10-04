#include <bob_ros_implementations/ros_rate_timer.h>

namespace bob
{

	ROSRateTimer::ROSRateTimer(float rate) :
	concreteRate(rate)
	{}

	void ROSRateTimer::startTimer()
	{
		concreteRate.reset();
	}

	void ROSRateTimer::sleepRemaining()
	{
		concreteRate.sleep();
	}	

}

