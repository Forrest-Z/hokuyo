#ifndef _BOB_STC_STC_PLAN_TOOLS_H_
#define _BOB_STC_STC_PLAN_TOOLS_H_

#include <bob_stc/stc_plan.h>
#include <bob_toolbox/world_point.h>

namespace bob
{

	STCPlan reorganizeAboutPoint(const STCPlan& plan, WorldPoint point);

}

#endif
