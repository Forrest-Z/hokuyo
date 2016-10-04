#ifndef _BOB_CONTROL_CURVED_SUBTASK_EXECUTOR_H_
#define _BOB_CONTROL_CURVED_SUBTASK_EXECUTOR_H_

#include <bob_sensor/itransform_handle.h>

#include <bob_control/conditions/istop_condition.h>
#include <bob_control/simple_commander.h>
#include <bob_control/conditions/null_condition.h>
#include <bob_control/straight_driver.h>
#include <bob_control/ipath_executor.h>

#include <bob_toolbox/subtask.h>

#include <bob_sensor/isensor_handle.h>

#include <bob_config/config.h>

namespace bob
{

	class CurvedSubtaskExecutor
	{
		public:

			CurvedSubtaskExecutor(SimpleCommander& simpleCommander, const ITransformHandle& transformHandle, float safetyDistance = Config::FRONT_SAFETY_DISTANCE) :
				simpleCommander(simpleCommander),
				transformHandle(transformHandle),
				straightDriver(transformHandle, simpleCommander),
				safetyDistance(safetyDistance)
			{}


			ControlResult execute(Subtask task, float toCurveToLength = 0.2,  IStopCondition::shared_ptr additionalCondition = NullCon);
			
		private:

			//! Rotates the robot in place using a heading rotation
			ControlResult rotateAlignWithNextLine(Subtask task, IStopCondition::shared_ptr additionalCondition);

			ControlResult driveAndCurveToPoint(Subtask task, float proximity, IStopCondition::shared_ptr additionalCondition);

			ControlResult curveOntoLine(Subtask task, IStopCondition::shared_ptr additionalCondition);

			SimpleCommander& simpleCommander;

			const ITransformHandle& transformHandle;

			StraightDriver straightDriver;
			
			float safetyDistance;
	
	};

}

#endif
