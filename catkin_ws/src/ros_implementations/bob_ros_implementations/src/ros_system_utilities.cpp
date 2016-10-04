#include <bob_ros_implementations/ros_system_utilities.h>

#include <bob_ros_implementations/ros_rate_timer.h>
#include <bob_ros_implementations/ros_timer.h>

#include <bob_system/system_utilities.h>

#include <ros/ros.h>

namespace bob
{

	void configureROSSystemUtilities()
	{
		systemUtilities.reset(new ROSSystemUtilities());
	}

	bool ROSSystemUtilities::ok() const
	{
		return ros::ok();
	}

	void ROSSystemUtilities::sleep(float seconds)
	{
		ros::Duration(seconds).sleep();
	}

	std::unique_ptr<IRateTimer> ROSSystemUtilities::rateTimer(float frequency)
	{
		return std::unique_ptr<IRateTimer>(new ROSRateTimer(frequency));
	}

	Timer ROSSystemUtilities::timer()
	{
		return Timer(new ROSTimer());
	}

}

