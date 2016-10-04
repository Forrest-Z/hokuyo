#ifndef _BOB_CONTROL_CURVE_HEADING_H_
#define _BOB_CONTROL_CURVE_HEADING_H_

#include <bob_toolbox/line.h>

namespace bob
{
	
	//! Used to calculate desired robot heading according to the line and robot current heading.
	class CurveHeading
	{

		public:

			CurveHeading(Line line, float radius):
				line(line),
				radius(radius)
			{}
		
			float desiredHeading(const WorldPoint& robotPoint);			

		private:

			Line line;
			float radius;


	};

}

#endif
