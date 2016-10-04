#ifndef _BOB_CONTROL_IPATH_EXECUTOR_H_
#define _BOB_CONTROL_IPATH_EXECUTOR_H_

#include <bob_control/conditions/null_condition.h>

namespace bob
{

	//! An interface for a class which takes in a path of world points and then drives the robot over the path.
	class IPathExecutor
	{

		public:

			typedef std::vector<WorldPoint> Path; 

			struct Result
			{
				Path::const_iterator lastPointReached;
				ControlResult lowLevelResult;
			};

			virtual Result run(const Path& path, IStopCondition::shared_ptr additionalCondition = NullCon) = 0;


	};

}

#endif
