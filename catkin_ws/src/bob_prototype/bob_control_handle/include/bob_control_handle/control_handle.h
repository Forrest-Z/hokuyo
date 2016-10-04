#ifndef _BOB_CONTROL_HANDLE_CONTROL_HANDLE_H_
#define _BOB_CONTROL_HANDLE_CONTROL_HANDLE_H_

#include <bob_control/curved_path_executor.h>
#include <bob_navigation/navigation_manager.h>
#include <bob_control/simple_commander.h>

namespace bob
{

	class IVelocityPublisher;
	class LockableMap;
	class ISensorHandle;

	//! Collects multiple control objects into one object for convienience
	class ControlHandle
	{

		public:

			ControlHandle(ISensorHandle& sensorHandle, LockableMap& lockableMap, IVelocityPublisher& velocityPublisher) : 
				velocityPublisher(velocityPublisher),
				simpleCommander(sensorHandle, velocityPublisher, lockableMap),
				pathExecutor(simpleCommander, sensorHandle.getTransformHandle()),
				navigationManager(pathExecutor, lockableMap, simpleCommander, sensorHandle)
		{}

			IVelocityPublisher& velocityPublisher;

			SimpleCommander simpleCommander;

			CurvedPathExecutor pathExecutor;

			NavigationManager navigationManager;	


	};

}

#endif
