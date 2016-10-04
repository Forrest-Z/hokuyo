#include <bob_boustrophedon/boustrophedon_condition.h>

namespace bob
{

	BoustrophedonCondition::BoustrophedonCondition(BoustrophedonCurveSpaces curveSpaces, const ISensorHandle& sensorHandle, float travellingAngle) : 
		followedToNextCondition(new HalfSpaceCondition(curveSpaces.skipLine)),
		endOfRectangleCondition(new HalfSpaceCondition(curveSpaces.passingGoal)),
		backToLineCondition(new HalfSpaceCondition(curveSpaces.backToLine)),
		backToStartCondition(new HalfSpaceCondition(curveSpaces.backToStart)),
		frontFree(new DistanceWorldDirectionCondition(travellingAngle, 0.2)),
		continueAlongLineCondition(new AndedCondition())
	{
		continueAlongLineCondition->add(frontFree);	
		continueAlongLineCondition->add(backToLineCondition);

		add(followedToNextCondition);
		add(endOfRectangleCondition);
		add(continueAlongLineCondition);
		add(backToStartCondition);
	}

	bool BoustrophedonCondition::continueAlongLine() const
	{
		return continueAlongLineCondition->wasSatisfied();
	}

	bool BoustrophedonCondition::endOfRectangle() const
	{
		return endOfRectangleCondition->wasSatisfied();
	}

	bool BoustrophedonCondition::followedToNext() const
	{
		return followedToNextCondition->wasSatisfied();
	}

	bool BoustrophedonCondition::backToStart() const
	{
		return backToStartCondition->wasSatisfied();
	}

}

