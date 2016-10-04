#include <bob_toolbox/simple_patterns.h>

#include <cmath>

namespace bob
{
	bool approximatelyEqual(float value1, float value2, float threshold)
	{
		return (fabs(value1 - value2) < threshold);
	}
}

