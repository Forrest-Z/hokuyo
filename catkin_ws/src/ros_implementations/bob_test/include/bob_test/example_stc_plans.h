#ifndef _BOB_TEST_EXAMPLE_STC_PLANS_H_
#define _BOB_TEST_EXAMPLE_STC_PLANS_H_

#include <bob_stc/stc_plan.h>

namespace bob
{




	//! File contains several example STC plans used for testing the stc_plan_executor
	//! These plans cons

	STCPlan simplePathWithAlternate(float alternateOffset, float alternateOrientation);

	//! A square which intersects the path at one corner and is rotated 45 degrees
	STCPlan exampleSTCPlan1();

	//! A square which intersects the path and is slightly rotated relative to path 
	STCPlan exampleSTCPlan2();

	//! A square which is a short distance offset from the first path and is rotated 45 degrees
	STCPlan exampleSTCPlan3();

	//! A square which is a short distance offset from the first path and is slightly rotated relative to path
	STCPlan exampleSTCPlan4();

}

#endif
