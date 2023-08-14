#ifndef SPHERE_H
#define SPHERE_H

#include <cmath>
#include "ray.h"

enum Surface {
    diffuse, specular, dielectric
};

class Sphere 
{
public:
	double r;	// Sphere radius
	Point p;	// Position
	Color c;	// Color  
	Color e;	// Emission
	Surface mat;	// Surface type

	Sphere(double r_, Point p_, Color c_, Color e_, Surface mat_): r(r_), p(p_), c(c_), e(e_), mat(mat_){}
  
	// Determine if ray hits sphere
	double intersect(const Ray &ray) const {
		// Return distance if there's interesection
		// Return 0.0 otherwise
		Vector op = ray.o - p;
		Vector d = ray.d;
		double t;
		double tol = 0.00001;
		double b = op.dot(d);
		double ce = op.dot(op) - r * r;
		double disc = b * b - ce;
		if (disc < 0) return 0.0;
		else disc = sqrt(disc);
		t = -b - disc;
		if (t > tol) return t;
		else t = -b + disc;
		if (t > tol) return t;
		else return 0;
	}
};

#endif // SPHERE_H