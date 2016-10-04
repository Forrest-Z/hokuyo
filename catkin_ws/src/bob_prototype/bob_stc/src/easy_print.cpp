#include <bob_stc/easy_print.h>

#include <bob_toolbox/easy_print.h>

namespace bob
{

	std::ostream& operator<<(std::ostream& os, const STCPlan& plan)
	{
		for (STCPlan::const_iterator pathItr = plan.begin(); pathItr != plan.end(); ++pathItr)
		{
			os << "Next path: " << std::endl;
			for (STCPath::const_iterator pointItr = pathItr->begin(); pointItr != pathItr->end(); ++pointItr)
			{
				os << "Point: " << *pointItr << std::endl;
			}
		}
		return os;
	}

}

