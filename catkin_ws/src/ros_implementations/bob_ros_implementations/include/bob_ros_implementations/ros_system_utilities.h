#ifndef _BOB_ROS_IMPLEMENTATIONS_ROS_SYSTEM_H_
#define _BOB_ROS_IMPLEMENTATIONS_ROS_SYSTEM_H_

#include <bob_system/system_utilities.h>
#include <memory>

namespace bob
{

	void configureROSSystemUtilities();

	class ROSSystemUtilities : public SystemUtilities
	{

		public:

			virtual bool ok() const;

			virtual void sleep(float seconds);

			virtual std::unique_ptr<IRateTimer> rateTimer(float frequency);

			virtual Timer timer();

	};

}

#endif
