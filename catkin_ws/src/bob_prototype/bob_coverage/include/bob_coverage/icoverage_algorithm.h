#ifndef _BOB_COVERAGE_ICOVERAGE_ALGORITHM_H_
#define _BOB_COVERAGE_ICOVERAGE_ALGORITHM_H_

namespace bob
{

	class DiscreteArea;

	//! \brief An interface for coverage algorithms. Derived classes implement an algorithm
	//! which calculates a path to cover an area, and then executes that path with the robot.
	//! The path used is designed to cover the area requested.
	class ICoverageAlgorithm
	{

		public:

			//! \brief Cover a given area with the robot. Method to be overriden in derived classes.
			//! \param area The area to cover
			virtual void coverArea(const DiscreteArea& area) = 0;

	};

}

#endif
