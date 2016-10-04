#ifndef _BOB_CONTROL_CURVED_PATH_EXECUTOR_H_
#define _BOB_CONTROL_CURVED_PATH_EXECUTOR_H_

#include <bob_control/straight_driver.h>
#include <bob_control/curved_subtask_executor.h>

#include <bob_control/conditions/istop_condition.h>

namespace bob
{

	//! An implementation of IPathExecutor which uses curved paths to complete its task
	class CurvedPathExecutor : public IPathExecutor
	{

		public:

			CurvedPathExecutor(SimpleCommander& simpleCommander, const ITransformHandle& transformHandle, float safetyDistance = 0.05) :
				transformHandle(transformHandle),
				straightDriver(transformHandle, simpleCommander),
				curvedSubtaskExecutor(simpleCommander, transformHandle, safetyDistance)
			{}
		
			virtual Result run(const std::vector<WorldPoint>& path, IStopCondition::shared_ptr additionalCondition = NullCon);
		
		private:

			const ITransformHandle& transformHandle;			
			StraightDriver straightDriver;
			CurvedSubtaskExecutor curvedSubtaskExecutor;
	};

}

#endif
