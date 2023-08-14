#ifndef LIGHT_PATH_H
#define LIGHT_PATH_H

#include "vector.h"

struct LightPath {
	Vector x;
	Color color;
	int light;
	int obj;

	LightPath() {}

	LightPath(const Vector &x, const Color &color, const int light, const int obj) :
		x(x), 
		color(color), 
		light(light), 
		obj(obj){
	}
};

#endif // LIGHT_PATH_H