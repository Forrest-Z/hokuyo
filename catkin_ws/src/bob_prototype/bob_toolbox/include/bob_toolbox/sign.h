#ifndef _BOB_TOOLBOX_SIGN_H_
#define _BOB_TOOLBOX_SIGN_H_

//! Returns the sign of an input
template <typename T> 
int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

#endif
