#include <bob_ros_implementations/ros_spinner.h>

#include <ros/ros.h>
#include <boost/bind.hpp>

namespace bob
{

	ROSSpinner::ROSSpinner()
	{
		start();
	}

	void ROSSpinner::start()
	{
		thread = boost::thread(boost::bind(ros::spin));
	}

}

