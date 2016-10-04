#include <bob_control/unsafe_range_calculator.h>

#include <cmath>

#include <bob_config/config.h>

namespace bob
{

	UnsafeRangeCalculator::UnsafeRangeCalculator()
	{
		forwardClearance = 0.15;

		// If the beam is within this distance then we must rotate it outside the path of the robot
		unclearableDistance = sqrt(pow(Config::ROBOT_RADIUS, 2) + pow(forwardClearance, 2));
		sideDistance = 0.01;
	}

	AngularRange UnsafeRangeCalculator::determineUnsafeRange(const LidarScan& beams) const
	{
		AngularRange unsafeRange;	

		for (LidarScan::iterator beamItr = beams.begin();
				beamItr != beams.end();
				++beamItr)
		{
			// If the beam was at this angle relative to the robot we'd be able to drive
			// forward at least the length of the clearingdistance
			if (beamItr->range < (Config::ROBOT_RADIUS + forwardClearance) && beamItr->range > Config::ROBOT_RADIUS)
			{
				// We only need to avoid points that are close enough to limit
				// our ability to move.
				SimpleAngularRange newUnsafe = unsafeRangeForBeam(*beamItr);
				unsafeRange.add(newUnsafe);		
			}
		}
		return unsafeRange;
	}

	SimpleAngularRange UnsafeRangeCalculator::unsafeRangeForBeam(LidarBeam beam) const
	{
		float unsafeOffset;
		if (beam.range < unclearableDistance)
		{
			// The beam is too short, so we can never move it to forwardClearance distance
			if (beam.range > (sideDistance + Config::ROBOT_RADIUS))
			{
				// Move it outside the robot's path so it can drive past it
				unsafeOffset = headingForPassing(beam.range);	
			}
			else
			{
				// The object is too close to pass by sideDistance away
				unsafeOffset = M_PI / 2;
			}
		}
		else
		{ 
			// The beam is at a distance where we must rotate a bit to move it in a better position
			unsafeOffset = headingForClearance(beam.range);
		}	
		return SimpleAngularRange(beam.angle - unsafeOffset, beam.angle + unsafeOffset);
	}

	float UnsafeRangeCalculator::headingForPassing(float beamLength) const
	{
		return asin((Config::ROBOT_RADIUS + sideDistance) / beamLength);	
	}

	float UnsafeRangeCalculator::headingForClearance(float beamLength) const
	{
		// These equations were obtained using the equation in distanceFromRobot:
		// forwardClearance = beamLength * cos(angle) - sqrt(r^2 - (beamLength * sin(angle))^2)

		// That equation was inverted to solve for 'angle' which resulted in the
		// equations below. You can do this inversion in wolframAlpha, for example.

		float numerator = pow(forwardClearance, 2) + pow(beamLength, 2) - pow(Config::ROBOT_RADIUS, 2);
		float denominator = 2 * forwardClearance * beamLength;
		float acosParam = numerator/denominator;

		return acos(acosParam);
	}

}

