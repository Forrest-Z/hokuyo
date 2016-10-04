#ifndef _BOB_BOUSTROPHEDON_BOUSTROPHEDON_CONDITION_H_
#define _BOB_BOUSTROPHEDON_BOUSTROPHEDON_CONDITION_H_

#include <bob_boustrophedon/boustrophedon_curve_spaces.h>
#include <bob_toolbox/subtask.h>

#include <bob_sensor/isensor_handle.h>

#include <bob_control/conditions/half_space_condition.h>
#include <bob_control/conditions/anded_condition.h>
#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/distance_world_direction_condition.h>

namespace bob
{

	class BoustrophedonCondition : public OredCondition
	{

		public:	

			typedef boost::shared_ptr<BoustrophedonCondition> shared_ptr;

			BoustrophedonCondition(BoustrophedonCurveSpaces curveSpaces, const ISensorHandle& sensorHandle, float travellingAngle);

			bool continueAlongLine() const;

			bool endOfRectangle() const;

			bool followedToNext() const;

			bool backToStart() const;

		private:

			HalfSpaceCondition::shared_ptr followedToNextCondition;
			HalfSpaceCondition::shared_ptr endOfRectangleCondition;
			HalfSpaceCondition::shared_ptr backToLineCondition;
			HalfSpaceCondition::shared_ptr backToStartCondition;

			AndedCondition::shared_ptr continueAlongLineCondition;
			DistanceWorldDirectionCondition::shared_ptr frontFree;
	};

}

#endif
