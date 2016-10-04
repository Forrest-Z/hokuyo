#ifndef _BOB_CONTROL_UNSAFE_RANGE_CALCULATOR_H_
#define _BOB_CONTROL_UNSAFE_RANGE_CALCULATOR_H_

#include <bob_toolbox/angular_range.h>

#include <bob_sensor/lidar_scan.h>

namespace bob
{

	class UnsafeRangeCalculator
	{

		public:

			UnsafeRangeCalculator();

			AngularRange determineUnsafeRange(const LidarScan& beams) const;

		private: 

			float headingForClearance(float beamLength) const;
			float headingForPassing(float beamLength) const;
			SimpleAngularRange unsafeRangeForBeam(LidarBeam beam) const;

			float forwardClearance;
			float unclearableDistance;
			float sideDistance;

	};

}

#endif
