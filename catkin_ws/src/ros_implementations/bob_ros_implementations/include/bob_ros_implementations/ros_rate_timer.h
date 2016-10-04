#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_RATE_TIMER_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_RATE_TIMER_H_

#include <ros/ros.h>

#include <bob_system/irate_timer.h>


namespace bob
{

	class ROSRateTimer : public IRateTimer
	{

		public:

			ROSRateTimer(float rate);

			virtual void startTimer();

			virtual void sleepRemaining();

		private:

			ros::Rate concreteRate;		

	};

}

#endif
