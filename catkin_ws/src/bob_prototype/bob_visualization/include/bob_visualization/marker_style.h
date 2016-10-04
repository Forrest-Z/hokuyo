#ifndef _BOB_VISUALIZATION_MARKER_STYLE_H_
#define _BOB_VISUALIZATION_MARKER_STYLE_H_

#include <bob_visualization/color.h>

#include <memory>

namespace bob
{

	struct MarkerStyle
	{
		float alpha;
		float width;
		float rotation;
		std::auto_ptr<Color> color;

		MarkerStyle() :
			alpha(0.8),
			width(0.02),
			color(new Red) {}

		MarkerStyle(float alpha, float width, Color* color) :
			alpha(alpha),
			width(width),
			color(color) {}



	};

	inline MarkerStyle& redMarkerStyle(float width = 0.05)
	{
		static MarkerStyle style(0.8, width, new Red);
		return style;	
	}

	inline MarkerStyle& greenMarkerStyle(float width = 0.05)
	{
		static MarkerStyle style(0.8, width, new Green);
		return style;	
	}
	
	inline MarkerStyle& blueMarkerStyle(float width = 0.05)
	{
		static MarkerStyle style(0.8, width, new Blue);
		return style;	
	}

	inline MarkerStyle& purpleMarkerStyle()
	{
		static MarkerStyle style(0.8, 0.05, new Purple);
		return style;	
	}
}

#endif
