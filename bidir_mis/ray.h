#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray 
{ 
public:
	Point o; // Ray origin
	Vector d; // Ray direction
	Ray(Point o_, Vector d_) : o(o_), d(d_) {} // Constructor
};


#endif // RAY_H