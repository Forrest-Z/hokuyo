#include <bob_toolbox/stat.h>

#include <time.h>
#include <stdlib.h>
#include <math.h>

namespace bob 
{

	// Uses the polar form of the Box-Muller transform
	float sampleZeroMeanGaussian(float sigma)
	{
		static bool seeded = false;
		if (!seeded)
		{
			// Seed with system time
			srand(time(NULL));
			seeded = true;
		}

		if (sigma == 0)
			return 0;

		float x1, x2, w;
		float r;

		do
		{
			do 
			{ 	
				r = drand48(); 
			} while (r == 0.0);
			x1 = 2.0 * r - 1.0;

			do 
			{ 
				r = drand48(); 
			} while (r == 0.0);
			x2 = 2.0 * drand48() - 1.0;

			w = x1 * x1 + x2 * x2;

		} while(w > 1.0 || w == 0.0);

		return (sigma * x2 * sqrt(- 2.0 * log(w)/ w));
	}

}

