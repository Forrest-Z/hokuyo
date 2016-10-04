#ifndef _BOB_TOOLBOX_ITERATOR_H_
#define _BOB_TOOLBOX_ITERATOR_H_

namespace bob
{

	//! This iterator will loop back to start when it gets to the end of the collection.
	//! The backToStart() condition replaces the end() normally used with iterators.
	//! It returns true when the first element is seen for the second time after looping
	//! back around through the collection.
	template <typename containerType>
	class LoopIterator
	{

		public:

			LoopIterator(const containerType& source, typename containerType::const_iterator start) :
			incrementedOnce(false),
			source(source),
			first(start),
			current(start)
			{}

			LoopIterator& operator++()
			{
				++current;	
				if (current == source.end())
					current = source.begin();

				incrementedOnce = true;
			}				

			typename containerType::const_iterator operator->()
			{
				return current;
			}

			bool backToStart()
			{
				return incrementedOnce && (first == current);
			}

		private:

			bool incrementedOnce;

			const containerType& source;

			typename containerType::const_iterator first;
			typename containerType::const_iterator current;


	};

}

#endif
