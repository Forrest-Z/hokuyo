#ifndef _BOB_CONTROL_SIMPLE_COMMANDER_H_
#define _BOB_CONTROL_SIMPLE_COMMANDER_H_

#include <bob_control/control_result.h>
#include <bob_control/ivelocity_publisher.h>
#include <bob_control/controller_runner.h>
#include <bob_grid_map/lockable_map.h>
#include <bob_sensor/ibumper_handle.h>

namespace bob
{

	// Forward declarations
	class ISensorHandle;
 	class ISimpleCommand;

	//! \brief Provides the ability to run SimpleCommands. These commands contain all the basic details 
	//! required to carry out a control action on the robot. The system is designed to be flexible so that
	//! new control actions can be quickly designed and tested. SimpleCommander is also responsible for 
	//! recovering from bumper events, but that may change in the future.
	class SimpleCommander
	{

		public:

			//! \brief Create a SimpleCommander object
			//! \param sensorHandle Reference providing access to sensor information from the robot
			//! \param velocityPublisher Reference providing ability to modify current robot velocity
			SimpleCommander(ISensorHandle& sensorHandle, IVelocityPublisher& velocityPublisher, LockableMap& lockableMap) : 
				sensorHandle(sensorHandle),
				controllerRunner(sensorHandle, velocityPublisher),
				velocityPublisher(velocityPublisher),
				lockableMap(lockableMap)
		{}

			//! \brief Execute a simple command and recover if a bumper is hit
			//! \param simpleCommand The command to run
			//! \return The result of running the command
			ControlResult execute(ISimpleCommand& simpleCommand);

		private:

			//! \brief Execute a simple command and retrieve result
			//! \param simpleCommand The command to run
			//! \return The result of running the command
			ControlResult runCommand(ISimpleCommand& simpleCommand);

			void insertHiddenObstacle();

			WorldVector offsetFromBumper(const BumperState& state);

			//! Provides access to sensor information from the robot
			ISensorHandle& sensorHandle;

			//! Does the actual work of running controllers
			ControllerRunner controllerRunner;

			IVelocityPublisher& velocityPublisher;

			LockableMap& lockableMap;			


	};

}

#endif
