#ifndef _BOB_CONTROL_COMPOSITE_CONDITION_H_
#define _BOB_CONTROL_COMPOSITE_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>
#include <bob_toolbox/pose2d.h>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace bob
{

	//! This is a base class that allows multiple conditions to be collected into one unit.
	//! Derived classes can then use the contained conditions to determine their own condition.
	//
	//! NOTE: Objects added into the condition MUST not be destroyed before the condition is used.
	//! Objects are not copied, but kept as references. Undefined behavior will occur if they are
	//! destroyed.
	//
	//! References are used because it allows clients to call wasSatisfied() on the added
	//! conditions to check if they were satisfied. 
	class CompositeCondition : public IStopCondition
	{

		public:
			
			//! Shadowing IStopCondition typedef, since CompositeCondition has different API
			typedef boost::shared_ptr<CompositeCondition> shared_ptr;

			//! Constructor that is initialized with no stored conditions
			CompositeCondition(){}

			//! Store a new reference to a condition
			void add(IStopCondition::shared_ptr condition);

		protected:

			std::vector<IStopCondition::shared_ptr> conditions;

	};

}

#endif
