#ifndef HELPERS_H
#define HELPERS_H
 
#include <random>
#include "sphere.h"

using namespace std;

double invPi = 1.0 / M_PI;

std::random_device rd;  // Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> dis(0.0, 1.0);

// limita el valor de x a [0,1]
inline double clamp(const double x) { 
	if(x < 0.0)
		return 0.0;
	else if(x > 1.0)
		return 1.0;
	return x;
}

inline double Clamp(double val, int low, int high) {
    if (val < low) return low;
    else if (val > high) return high;
    else return val;
}

// convierte un valor de color en [0,1] a un entero en [0,255]
inline int toDisplayValue(const double x) {
	return int( pow( clamp(x), 1.0/2.2 ) * 255 + .5); 
}

void coordinateSystem(Vector &n, Vector &s, Vector &t){
	if (std::fabs(n.x) > std::fabs(n.y)) {
		float invLen = 1.0f / std::sqrt(n.x * n.x + n.z * n.z);
		s = Vector(n.z * invLen, 0.0f, -n.x * invLen);
	} else {
		float invLen = 1.0f / std::sqrt(n.y * n.y + n.z * n.z);
		s = Vector(0.0f, n.z * invLen, -n.y * invLen);
	}
	t = (s % n);
}

// funcion para obtener una direccion global a partir de una local
Vector globalizeCoord(Vector &locVec, Vector &n, Vector &s, Vector &t){
	Vector globalCoord;
	Vector row1 = Vector (s.x, t.x, n.x);
	Vector row2 = Vector (s.y, t.y, n.y);
	Vector row3 = Vector (s.z, t.z, n.z);

	globalCoord = Vector (row1.dot(locVec), row2.dot(locVec), row3.dot(locVec));
	return globalCoord;
}

// funcion para hacer un vector a partir de theta y phi
Vector makeVec(double &theta, double &phi){
	double x = sin(theta) * cos(phi);
    double y = sin(theta) * sin(phi);
    double z = cos(theta);

	Vector vec = Vector (x, y, z);
	return vec;
}

// funcion para obtener los parametros de muestreo de coseno hemisferico
void paramCosineHemisphere(double &theta, double &phi) {
    double rand1 = dis(gen);
    double rand2 = dis(gen);
    theta = acos(sqrt(1.0 - rand1));
    phi = 2.0 * M_PI * rand2;
}

// funcion para obtener los parametros de muestreo de angulo solido
void paramSolidAngle(Point &p, double &theta, double &phi, double &cosTmax, const Sphere &light) {
	double r = light.r;
    double rand1 = dis(gen);
    double rand2 = dis(gen);
	Vector wc = (light.p - p);
	double lengthWc = sqrt((wc.x * wc.x) + (wc.y * wc.y) + (wc.z * wc.z));
	double sinTmax = r / lengthWc;
	cosTmax = sqrt(1.0 - (sinTmax * sinTmax));
    theta = acos(1.0 - rand1 + (rand1 * (cosTmax)));
    phi = 2.0 * M_PI * rand2;
}

// funcion para obtener direccion de muestreo en area
Vector sphereUniformDir() {
    double rand1 = dis(gen);
    double rand2 = dis(gen);
    double theta = acos(1.0 - 2.0 * rand1);
    double phi = 2.0 * M_PI * rand2;

	Vector dir = makeVec(theta, phi);
	return dir;
}

// funcion para calcular probabilidad de muestreo de coseno hemisferico (diffuse material pdf)
double probCosineHemisphere(Vector &wi, Vector &n){
	return (wi.dot(n)) * invPi;
}

// funcion para calcular probabilidad de muestreo del angulo solido (light pdf)
double probSolidAngle(double &cosTmax) {
	return 1.0 / (2 * M_PI * (1 - cosTmax));
}

// funcion para calcuar pdf de direccion de refraccion o reflexion en dielectricos
double probDielectric(double &F, bool &reflect){
	double pdf;
	if (reflect){
		pdf = F;
	}
	else {
		pdf = 1 - F;
	}
	return pdf;
}

// funcion para obtener una diraccion muestreada en superficie difusa
Vector sampleDir(Vector &n, double &theta, double &phi){
	Vector s, t;
	coordinateSystem(n, s, t);
	Vector wi = makeVec(theta, phi);
	Vector wiglob = globalizeCoord(wi, n, s, t);
	return wiglob;
}

// calculo de Fresnel
double dielectricFresnel(double &ni, double &nt, double &n2, double &cosTi, double &cosTt){
	double rpar, rper, F;

	double sinTi = sqrt(max(0.0, 1.0 - cosTi * cosTi));
	double sinTt = n2 * sinTi;
	if (sinTt >= 1){
		F = 1.0;
	}
	else{
		rpar = ((nt*cosTi) - (ni*cosTt))/((nt*cosTi) + (ni*cosTt));
		rper = ((ni*cosTi) - (nt*cosTt))/((ni*cosTi) + (nt*cosTt));
		F = (rpar*rpar + rper*rper) * 0.5;
	}
	return F;
}

// ley de snell para calcular cosTt
double snell(double &cosTi, double &eta){
	double sinTi = sqrt(max(0.0, 1.0 - cosTi * cosTi));
	double sinTt = eta * sinTi;
	double cosTt = sqrt(max(0.0, 1 - sinTt * sinTt));
	return cosTt;
}

#endif // HELPERS_H