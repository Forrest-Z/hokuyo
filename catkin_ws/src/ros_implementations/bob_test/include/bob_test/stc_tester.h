#ifndef _BOB_TEST_STC_TESTER_H_
#define _BOB_TEST_STC_TESTER_H_

#include <bob_stc/stc_plan_executor.h>

namespace bob
{
	//! This class will test the STCPlanExecutor.
	
	class STCTester
	{
		public:
			STCTester(const LockableMap& lockableMap, STCPlanExecutor& planExecutor);

		private:
			
			STCPlanExecutor& planExecutor;

	};

}

#endif
