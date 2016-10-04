#ifndef _BOB_TOOLBOX_LOW_PASS_FILTER_H_
#define _BOB_TOOLBOX_LOW_PASS_FILTER_H_

namespace bob
{

	class LowPassFilter
	{

		public:

			LowPassFilter(float timeConstant, float dt) : 
				alpha(dt / (timeConstant + dt)),
				previousOutput(0),
				initialized(false)
		{}

			float filterDataPoint(float unfiltered);

		private:

			float alpha;

			float previousOutput;

			bool initialized;

	};

}

#endif
