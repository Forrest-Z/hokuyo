#ifndef _BOB_TOOLBOX_BASIC_ALGEBRA_H_
#define _BOB_TOOLBOX_BASIC_ALGEBRA_H_

#include <complex>
#include <utility>

namespace bob
{

	typedef std::complex<float> FloatComplex;
	typedef std::pair<FloatComplex, FloatComplex> QuadraticSolution;

	QuadraticSolution quadraticFactor(float a, float b, float c);

}

#endif
