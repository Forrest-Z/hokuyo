#ifndef _BOB_BOUSTROPHEDON_BOUSTROPHEDON_H_
#define _BOB_BOUSTROPHEDON_BOUSTROPHEDON_H_

#include <bob_coverage/icoverage_algorithm.h>

#include <bob_boustrophedon/boustrophedon_task_parameters.h>
#include <bob_boustrophedon/boustrophedon_subtask_executor.h>

#include <bob_toolbox/subtask.h>


namespace bob
{

	class ControlHandle;
	class ISensorHandle;
	class DiscreteArea;

	//! \brief Implements the zig-zag Boustrophedon coverage algorithm. Implements the ICoverageAlgorithm interface.
	class Boustrophedon : public ICoverageAlgorithm
	{

		public:

			//! \brief Construct a Boustrophedon object
			//! \param controlHandle Provides connection to control robot behavior
			//! \param sensorHandle Provides connection to sensor information
			Boustrophedon(ControlHandle& controlHandle, const ISensorHandle& sensorHandle) : 
				controlHandle(controlHandle),
				sensorHandle(sensorHandle),
				curveExecutor(controlHandle, sensorHandle),
				lineSpacing(0.3)
			{}
			
			//! \brief Cover an area with the Boustrophedon algorithm. 
			//! See documentation in ICoverageAlgorthm.
			virtual void coverArea(const DiscreteArea& area);
			
		private:

			//! Connection to robot sensor information
			const ISensorHandle& sensorHandle;

			//! Connection to control robot behavior
			ControlHandle& controlHandle;

			//! Defines the current task in simple terms
			BoustrophedonTaskParameters taskParameters;

			//! Carries out the boustrophedon actions
			BoustrophedonSubtaskExecutor curveExecutor;

			//! Spacing between successive boustrophedon lines, should be slightly less than robot diameter.
			float lineSpacing;	

			//! \brief Executes the boustrophedon action based on the configured task parameters.
			void execute();

			//! Updates the Subtask one position along the Boustrophedon task.
			//! \param spacing The length of the new task
			//! \param newCurveToAngle The new angle to curve to at the end of the Subtask
			void updateSubtask(Subtask& toUpdate, float spacing, float newCurveToAngle); 
	};

}

#endif
