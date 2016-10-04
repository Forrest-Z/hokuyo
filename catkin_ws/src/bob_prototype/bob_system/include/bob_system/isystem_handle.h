#ifndef _BOB_SYSTEM_ISYSTEM_HANDLE_H_
#define _BOB_SYSTEM_ISYSTEM_HANDLE_H_

namespace bob
{

	class ISensorHandle;
	class LockableMap;
	class ControlHandle;
	class IVelocityPublisher;

	class ISystemHandle
	{

		public:

			//! \brief Obtain a reference to the ISensorHandle member
			//! \return A reference to the ISensorHandle member
			virtual ISensorHandle& getSensorHandle() = 0;

			//! \brief Obtain a reference to the LockableMap member
			//! \return A reference to the LockableMap member
			virtual LockableMap& getLockableMap() = 0;
	
			//! \brief Obtain a reference to the ControlHandle member
			//! \return A reference to the ControlHandle member
			virtual ControlHandle& getControlHandle() = 0;

			virtual IVelocityPublisher& getVelocityPublisher() = 0;

			virtual ~ISystemHandle()
			{}

	};

}

#endif
