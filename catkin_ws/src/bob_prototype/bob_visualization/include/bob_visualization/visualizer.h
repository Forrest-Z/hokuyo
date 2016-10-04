#ifndef _BOB_VISUALIZATION_VISUALIZER_H_
#define _BOB_VISUALIZATION_VISUALIZER_H_

#include <memory>

#include <bob_toolbox/world_point.h>

#include <bob_visualization/marker_style.h>
#include <bob_visualization/marker_types.h>

namespace bob
{

	class Pose2D;

	//! \brief Base class for visualization system.
	//! This class can be re-implemented for different visualization systems. To enable a particular visualization method,
	//! you must initialize the global visualizer variable with an instance to that visualization method.
	class Visualizer
	{

		public:

			virtual void visualize(std::string title, const MarkerLine& subject, const MarkerStyle& style = MarkerStyle())
			{}

			virtual void visualize(std::string title, const MarkerLineList& subject, const MarkerStyle& style = MarkerStyle())
			{}

			virtual void visualize(std::string title, const MarkerCircle& subject, const MarkerStyle& style = MarkerStyle())
			{
				int numberOfSteps = 180;
				float angleIncrement = 2 * M_PI / (float)(numberOfSteps);

				float angle = 0;
				std::vector<WorldPoint> points;
				while(angle < 2 * M_PI)
				{	
					WorldPoint nextPoint = subject.center + subject.radius * unitVector(angle);
					points.push_back(nextPoint);
					angle += angleIncrement;
				}
	
				// Call polymorphically
				this->visualize(title, MarkerLine(points), style);
			}


			virtual void visualize(std::string title, const MarkerSquares& subject, const MarkerStyle& style = MarkerStyle())
			{}
			
			virtual void visualize(std::string title, const MarkerArrow& subject, const MarkerStyle& style = MarkerStyle())
			{}

			virtual void visualize(std::string title, const WorldPoint& subject, const MarkerStyle& style = MarkerStyle())
			{}

			virtual void visualize(std::string title, const Pose2D& subject, const MarkerStyle& style = MarkerStyle())
			{}

			virtual void visualize(std::string title, const MarkerPoints& subject, const MarkerStyle& style = MarkerStyle())
			{}

			virtual void visualize(std::string title, const MarkerEllipse& subject, const MarkerStyle& style = MarkerStyle())
			{
				int numberOfSteps = 180;
				float angleIncrement = 2 * M_PI / (float)(numberOfSteps);

				WorldVector xUnit(1, 0);
				WorldVector yUnit(0, 1);

				float angle = 0;
				float finalAngle = 2 * M_PI;
				std::vector<WorldPoint> points;
				while(angle < finalAngle)
				{	
					float xOffset = subject.dimensions.width * cos(angle);
					float yOffset = subject.dimensions.height * sin(angle);
					WorldVector unrotatedVector(xOffset, yOffset);

					WorldPoint nextPoint = (WorldPoint)subject.pose + rotate(unrotatedVector, subject.pose.theta);
					points.push_back(nextPoint);
					angle += angleIncrement;
				}
	
				// Call polymorphically
				this->visualize(title, MarkerLine(points), style);
			}

	};

}

#endif
