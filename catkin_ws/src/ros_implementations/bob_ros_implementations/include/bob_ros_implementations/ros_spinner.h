#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_SPINNER_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_SPINNER_H_

#include <boost/thread.hpp>

namespace bob
{

	//! \brief Calls ros::spin() on a separate thread which will service
	//! ros::subscriber objects with their callbacks
	class ROSSpinner
	{

		public:

			//! \brief Construct and start a ROSSpinner object
			ROSSpinner();

		private:

			//! \brief Start the ROS Spinner
			void start();

			//! The thread on which to do the spinning
			boost::thread thread;

	};

}

#endif
