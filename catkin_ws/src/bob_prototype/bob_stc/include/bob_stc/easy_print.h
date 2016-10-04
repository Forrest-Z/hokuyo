#ifndef _BOB_STC_EASY_PRINT_H_
#define _BOB_STC_EASY_PRINT_H_

#include <bob_stc/stc_plan.h>
#include <ostream>

namespace bob
{

	std::ostream& operator<<(std::ostream& os, const STCPlan& plan);

}

#endif
