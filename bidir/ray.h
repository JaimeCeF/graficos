#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray 
{ 
public:
	Point o; // origen del rayo
	Vector d; // direcccion del rayo
	Ray(Point o_, Vector d_) : o(o_), d(d_) {} // constructor
};


#endif // RAY_H