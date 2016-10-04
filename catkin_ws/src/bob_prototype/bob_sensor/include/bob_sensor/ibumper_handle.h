#ifndef _BOB_SENSOR_IBUMPER_HANDLE_H_
#define _BOB_SENSOR_IBUMPER_HANDLE_H_

namespace bob
{

	enum BumperState
	{
		LeftEvent,
		LeftCenterEvent,
		CenterEvent,
		RightCenterEvent,
		RightEvent,
		AllEvent,
		NoEvent
	};

	//! \brief Provides information about the bumper state. This is a work in progress.
	class IBumperHandle
	{

		public:

			virtual BumperState getState() const = 0;

			virtual bool hitSomething() const = 0;

			virtual void clearFlags() = 0;

			virtual ~IBumperHandle() 
			{}

	};

}

#endif
