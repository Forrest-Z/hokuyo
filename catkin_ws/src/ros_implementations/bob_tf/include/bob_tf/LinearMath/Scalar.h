#ifndef _BOB_TF2_SCALAR_H
#define _BOB_TF2_SCALAR_H

#include <math.h>
#include <stdlib.h>//size_t for MSVC 6.0
#include <cstdlib>
#include <cfloat>
#include <float.h>

typedef float tf2Scalar;

//this number could be bigger in float precision
#define TF2_LARGE_FLOAT 1e30

inline tf2Scalar tf2Acos(tf2Scalar x) 
{ 
	if (x<tf2Scalar(-1))	
		x=tf2Scalar(-1); 

	if (x>tf2Scalar(1))	
		x=tf2Scalar(1); 

	return acos(x); 
}

inline tf2Scalar tf2Asin(tf2Scalar x) 
{ 
	if (x<tf2Scalar(-1))	
		x=tf2Scalar(-1); 

	if (x>tf2Scalar(1))	
		x=tf2Scalar(1); 

	return asin(x); 
}

#define TF2SIMD_2_PI         tf2Scalar(6.283185307179586232)
#define TF2SIMD_PI           (TF2SIMD_2_PI * tf2Scalar(0.5))
#define TF2SIMD_HALF_PI      (TF2SIMD_2_PI * tf2Scalar(0.25))
#define TF2SIMD_RADS_PER_DEG (TF2SIMD_2_PI / tf2Scalar(360.0))
#define TF2SIMD_DEGS_PER_RAD  (tf2Scalar(360.0) / TF2SIMD_2_PI)
#define TF2SIMDSQRT12 tf2Scalar(0.7071067811865475244008443621048490)

#endif
