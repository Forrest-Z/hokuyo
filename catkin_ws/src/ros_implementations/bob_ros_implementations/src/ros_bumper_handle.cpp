#include <bob_ros_implementations/ros_bumper_handle.h>

namespace bob
{

	ROSBumperHandle::ROSBumperHandle() :
		lastBumperState{}
	{
		ros::NodeHandle nh;
		bumperSubscriber = nh.subscribe("/mobile_base/events/bumper", 20, &ROSBumperHandle::messageCallback, this);
	}

	void ROSBumperHandle::messageCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
	{
		// Only enable bumpers, don't disable
		lastBumperState[msg->bumper] |= msg->state;
	}


	BumperState ROSBumperHandle::getState() const
	{
		if (lastBumperState[Left])
		{
			if (lastBumperState[Right])
			{
				return AllEvent;
			}
			else if(lastBumperState[Center])
			{
				return LeftCenterEvent;
			}
			else
			{
				return LeftEvent;
			}
		}
		else if (lastBumperState[Center])
		{
			if (lastBumperState[Right])
			{
				return RightCenterEvent;
			}	
			else
			{
				return CenterEvent;
			}
		}
		else if (lastBumperState[Right])
		{
			return RightEvent;
		}
		return NoEvent;
	}

	bool ROSBumperHandle::hitSomething() const
	{
		return (lastBumperState[0] || 
				lastBumperState[1] || 
				lastBumperState[2]);
	}

	void ROSBumperHandle::clearFlags()
	{
		lastBumperState[0] = false;	
		lastBumperState[1] = false;	
		lastBumperState[2] = false;	
	}
}

