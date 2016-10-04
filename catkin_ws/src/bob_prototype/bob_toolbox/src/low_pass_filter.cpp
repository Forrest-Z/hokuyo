#include <bob_toolbox/low_pass_filter.h>

namespace bob
{

	float LowPassFilter::filterDataPoint(float unfiltered)
	{
		if (!initialized)
		{
			previousOutput = unfiltered;
			initialized = true;
		}
		else
		{
			previousOutput = previousOutput + alpha * (unfiltered - previousOutput);
		}
		return previousOutput;
	}

}

