#ifndef _BOB_TOOLBOX_SIMPLE_PATTERNS_H_
#define _BOB_TOOLBOX_SIMPLE_PATTERNS_H_

namespace bob
{

	//TODO: Rename file to something more appropriate

	//! Returns true if two float values are approximately equal, to within an optionally specified threshold.
	bool approximatelyEqual(float value1, float value2, float threshold = 0.001);

}

#endif
