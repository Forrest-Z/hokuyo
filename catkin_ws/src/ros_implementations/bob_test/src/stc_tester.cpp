#include <bob_test/stc_tester.h>

#include <bob_toolbox/logging.h>
#include <bob_test/stc_test_plans.h>

#include <ros/ros.h>

#include <bob_stc/visualization.h>

namespace bob
{

	STCTester::STCTester(const LockableMap& lockableMap, STCPlanExecutor& planExecutor):
		planExecutor(planExecutor)
	{
		STCTestPlans testPlans;

		for(int i = 0; i < testPlans.size(); ++i)
		{
			LOG_TEST("plan" << i);
			STCPlan plan = testPlans.getPlan(i);
			planExecutor.execute(plan);

			std::cin.ignore();
		}

	}

}

