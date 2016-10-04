#ifndef _BOB_CONTROL_ISIMPLE_COMMAND_H_
#define _BOB_CONTROL_ISIMPLE_COMMAND_H_

#include <bob_control/control_result.h>

#include <bob_control/controller_runner.h>
#include <bob_sensor/isensor_handle.h>

#include <bob_control/runnable.h>
#include <bob_control/icontroller.h>
#include <bob_control/conditions/ored_condition.h>


namespace bob
{

	class IStopCondition;

	class ISimpleCommand
	{

		public:	

			ISimpleCommand() :
			bumperEnabled(true)
			{}

			Runnable generateRunnable(const ISensorHandle& sensorHandle);

			virtual IController::shared_ptr generateController(const ISensorHandle& sensorHandle) const = 0;

			virtual OredCondition::shared_ptr generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const = 0;

			//! Virtual destructor, allowing derived class to be deleated via pointer to base
			virtual ~ISimpleCommand()
			{}

			//! If true, bumper condition will be added to the runnable when it is generated
			bool bumperEnabled;
	};

}

#endif
