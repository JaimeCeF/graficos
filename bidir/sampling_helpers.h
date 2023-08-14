#ifndef HELPERS_H
#define HELPERS_H
 
#include <random>
#include "sphere.h"

using namespace std;

double invPi = 1.0 / M_PI;

std::random_device rd;  // Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> dis(0.0, 1.0);

// Limits range of x to [0,1]
inline double clamp(const double x) { 
	if(x < 0.0)
		return 0.0;
	else if(x > 1.0)
		return 1.0;
	return x;
}

// Limits range of x to [low,high]
inline double Clamp(double val, int low, int high) {
    if (val < low) return low;
    else if (val > high) return high;
    else return val;
}

// Converts color value from [0,1] to int in [0,255]
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

// Get global direction vector from local direction vector using coordinateSystem()
Vector globalizeCoord(Vector &locVec, Vector &n, Vector &s, Vector &t){
	Vector globalCoord;
	Vector row1 = Vector (s.x, t.x, n.x);
	Vector row2 = Vector (s.y, t.y, n.y);
	Vector row3 = Vector (s.z, t.z, n.z);

	globalCoord = Vector (row1.dot(locVec), row2.dot(locVec), row3.dot(locVec));
	return globalCoord;
}

// Make vector from theta and phi
Vector makeVec(double &theta, double &phi){
	double x = sin(theta) * cos(phi);
    double y = sin(theta) * sin(phi);
    double z = cos(theta);

	Vector vec = Vector (x, y, z);
	return vec;
}

// Obtain cosine hemisphere sampling parameters
void paramCosineHemisphere(double &theta, double &phi) {
    double rand1 = dis(gen);
    double rand2 = dis(gen);
    theta = acos(sqrt(1.0 - rand1));
    phi = 2.0 * M_PI * rand2;
}

// Obtain solid angle sampling parameters
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

//  Obtain direction by uniformly sampling a sphere
Vector sphereUniformDir() {
    double rand1 = dis(gen);
    double rand2 = dis(gen);
    double theta = acos(1.0 - 2.0 * rand1);
    double phi = 2.0 * M_PI * rand2;

	Vector dir = makeVec(theta, phi);
	return dir;
}

// Compute diffuse material pdf from cosine hemisphere sampling
double probCosineHemisphere(Vector &wi, Vector &n){
	return (wi.dot(n)) * invPi;
}

// Compute light pdf from solid angle sampling
double probSolidAngle(double &cosTmax) {
	return 1.0 / (2 * M_PI * (1 - cosTmax));
}

// Compute reflection or refraction pdf
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

// Compute direction vector sampled from diffuse surface
Vector sampleDir(Vector &n, double &theta, double &phi){
	Vector s, t;
	coordinateSystem(n, s, t);
	Vector wi = makeVec(theta, phi);
	Vector wiglob = globalizeCoord(wi, n, s, t);
	return wiglob;
}

// Compute Fresnel for dielectric surface
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

// Snell's law to compute cosTt
double snell(double &cosTi, double &eta){
	double sinTi = sqrt(max(0.0, 1.0 - cosTi * cosTi));
	double sinTt = eta * sinTi;
	double cosTt = sqrt(max(0.0, 1 - sinTt * sinTt));
	return cosTt;
}

#endif // HELPERS_H