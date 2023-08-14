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
	double r;	// radio de la esfera
	Point p;	// posicion
	Color c;	// color  
	Color e;	// emision
	Surface mat;	// surface type, 0 = diffuse, 1 = specular, 2 = dielectric

	Sphere(double r_, Point p_, Color c_, Color e_, Surface mat_): r(r_), p(p_), c(c_), e(e_), mat(mat_){}
  
	// determina si el rayo intersecta a esta esfera
	double intersect(const Ray &ray) const {
		// regresar distancia si hay intersección
		// regresar 0.0 si no hay interseccion
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