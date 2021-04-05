#ifndef CORE_H
#define CORE_H

#include <armadillo>
using namespace arma;


// Struct defining the Core Frame
struct CORE
{
	vec origin;
	mat frame;
	vec targ;

	CORE()
	{
		origin.resize(3);
		origin.fill(0);

		frame.resize(3, 3);
		frame = eye(3, 3);

		targ = {2.2720, 0.8453, -0.8453};
	};
};

#endif /* CORE_H */