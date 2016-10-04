#ifndef _BOB_TF_EXCEPTIONS_H_
#define _BOB_TF_EXCEPTIONS_H_

#include <stdexcept>

namespace bob
{

	//! Base class for all exceptions
	class TransformException : public std::runtime_error
	{ 
		public:
			TransformException(const std::string errorDescription) : 
				std::runtime_error(errorDescription) 
		{}
	};

	//! The frame tree is not connected between the frames requested
	class ConnectivityException : public TransformException
	{ 
		public:
			ConnectivityException(const std::string errorDescription) : 
				TransformException(errorDescription) 
		{}
	};

	//! A frame not in the graph has been requested
	class LookupException: public TransformException
	{ 
		public:
			LookupException(const std::string errorDescription) : 
				TransformException(errorDescription) 
		{}
	};

	//! An exception class to notify that the requested value would have required extrapolation beyond current limits.
	class ExtrapolationException : public TransformException 
	{ 
		public:
			ExtrapolationException(const std::string errorDescription) : 
				TransformException(errorDescription) 
		{}
	};

	//! An exception class to notify that one of the arguments is invalid
	class InvalidArgumentException: public TransformException  
	{ 
		public:
			InvalidArgumentException(const std::string errorDescription) : 
				TransformException(errorDescription) 
		{}
	};

	//! An exception class to notify that a timeout has occured
	class TimeoutException: public TransformException  
	{ 
		public:
			TimeoutException(const std::string errorDescription) : 
				TransformException(errorDescription) 
		{}
	};


}

#endif
