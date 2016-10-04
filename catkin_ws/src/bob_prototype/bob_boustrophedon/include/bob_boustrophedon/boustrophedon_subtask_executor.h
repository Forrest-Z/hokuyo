#ifndef _BOB_BOUSTROPHEDON_BOUSTROPHEDON_SUBTASK_EXECUTOR_H_
#define _BOB_BOUSTROPHEDON_BOUSTROPHEDON_SUBTASK_EXECUTOR_H_

#include <bob_toolbox/line.h>
#include <bob_toolbox/subtask.h>

#include <bob_sensor/isensor_handle.h>

#include <bob_control_handle/control_handle.h>

#include <bob_wall_following/wall_follower.h>

namespace bob
{

	//! Carries out Boustrophedon subtasks, and reports the result
	class BoustrophedonSubtaskExecutor
	{

		public:

			//! Create a BoustrophedonSubtaskExecutor
			BoustrophedonSubtaskExecutor(ControlHandle& controlHandle, const ISensorHandle& sensorHandle) :
			controlHandle(controlHandle),
			sensorHandle(sensorHandle)
			{}

			//! The result of a subtask execution
			enum DriveCurveResult
			{
				//! Wall followed to the next boustrophedon line. This occurs
				//! when an obstacle was encountered on the path, and the robot	
				//! wall follows trying to get around it, but the object is too large
				//! so the robot hits the next line.
				WallFollowedToNext,

				//! Reached the end of the current line. The subtask was completed fully.
				ReachedEnd,

				//! Wall following failed to reach the next line. This probably means that 
				//! the Boustrophedon task should be finished.
				WallFollowFailed,
			};

			struct TaskProperties
			{
				CircularDirection finalCurveDirection;

				WallFollowSide wallFollowSide;
			};

			//! Executes a subtask
			//! @param action The subtask to execute
			//! @param failLength The length perpendicular to the line at which to 	
			//! wall follow before failure
			DriveCurveResult driveAndCurve(Subtask action, float failLength);

		private:

			ControlHandle& controlHandle;
			const ISensorHandle& sensorHandle;

			float travellingAngle;
			float curveToAngle;
			Subtask action;

			void cacheAction(Subtask nextAction);

			TaskProperties determineTaskProperties();

	};
}

#endif
