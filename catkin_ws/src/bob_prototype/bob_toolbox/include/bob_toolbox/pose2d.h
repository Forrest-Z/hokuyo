#ifndef _BOB_TOOLBOX_POSE2D_H_
#define _BOB_TOOLBOX_POSE2D_H_

#include <bob_toolbox/world_point.h>
#include <cmath>

namespace bob
{

	struct Pose2D : public WorldPoint
	{
		Pose2D() : WorldPoint(), theta(0) {}
		Pose2D(WorldPoint position, float theta) : WorldPoint(position), theta(theta) {}
		Pose2D(float x, float y, float theta) : WorldPoint(x, y), theta(theta) {}

		float theta;

		Pose2D generateInversion() const
		{
			float newX = -x * cos(theta) - y * sin(theta);
			float newY = x * sin(theta) - y * cos(theta);
			return Pose2D(newX, newY, -theta);
		}

		Pose2D operator*(const Pose2D& other) const
		{
			float newX = x + cos(theta) * other.x - sin(theta) * other.y;
			float newY = y + sin(theta) * other.x + cos(theta) * other.y;
			float newTheta = theta + other.theta;
			return Pose2D(newX, newY, newTheta);
		}

		Pose2D operator+(const Pose2D& other) const
		{
			return Pose2D(x + other.x, y + other.y, theta + other.theta);
		}
		
		Pose2D operator-(const Pose2D& other) const
		{
			return Pose2D(x - other.x, y - other.y, theta - other.theta);
		}

		Pose2D operator+=(const Pose2D& other)
		{
			*this = *this + other;
		}
	};

}

#endif
