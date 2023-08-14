#ifndef LIGHT_PATH_H
#define LIGHT_PATH_H

#include "vector.h"

struct LightPath {
	Vector x; // Light ray hit point
	Color color; // Hit object color
	int light; // Light from which light ray was trhown
	int obj; // Index of object hit by light ray

	LightPath() {}

	LightPath(const Vector &x, const Color &color, const int light, const int obj) :
		x(x), 
		color(color), 
		light(light), 
		obj(obj){
	}
};

#endif // LIGHT_PATH_H