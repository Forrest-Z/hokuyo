#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_TIMER_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_TIMER_H_

#include <bob_system/itimer.h>

#include <ros/ros.h>

namespace bob
{
	
	//! \brief ROS implementation of ITimer
	class ROSTimer : public ITimer
	{

		public:

			//! \brief Construct an instance of ROSTimer. 
			//! Starts the timer, so calling startTimer() isn't really necessary
			ROSTimer()
			{
				this->startTimer();
			}

			virtual void startTimer();

			virtual float timeElapsed();

		private:

			//! The time at which the timer was last started
			ros::Time startTime;

	};

}

#endif
