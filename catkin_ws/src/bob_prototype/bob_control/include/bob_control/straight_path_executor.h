#ifndef _BOB_CONTROL_STRAIGHT_PATH_EXECUTOR_H_
#define _BOB_CONTROL_STRAIGHT_PATH_EXECUTOR_H_

#include <vector>

#include <bob_sensor/itransform_handle.h>

#include <bob_control/simple_commander.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_control/conditions/null_condition.h>
#include <bob_control/ipath_executor.h>

namespace bob
{

	class StraightPathExecutor : public IPathExecutor
	{

		public:

			StraightPathExecutor(SimpleCommander& simpleCommander, const ITransformHandle& transformHandle) : 
			simpleCommander(simpleCommander),
			transformHandle(transformHandle)
			{}

			virtual Result run(const std::vector<WorldPoint>& path, IStopCondition::shared_ptr additionalCondition = NullCon);

		private:

			SimpleCommander& simpleCommander;

			const ITransformHandle& transformHandle;
		
	};

}

#endif
