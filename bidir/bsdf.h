#ifndef BSDF_H
#define BSDF_H

#include "vector.h"
#include "ray.h"
#include "sampling_helpers.h"

class DirectionBSDF {
public:
	static Vector DiffuseBSDF(Vector &n) {
        double theta, phi, probMat;
		paramCosineHemisphere(theta, phi);
		Vector wi = sampleDir(n, theta, phi).normalize();
        return wi;
	}

    static Vector SpecularBSDF(Ray &r, Vector &n) {
        Vector wr = (r.d - n * (2 * n.dot(r.d))).normalize();
        return wr;
	}

	static Vector DielectricBSDF(const Ray &r, const Vector &wi, const Vector &nv, double &n2, double &cosTi, double &cosTt, bool &reflect) {

		Vector wr = r.d - nv*2*((nv.dot(r.d)));  // direccion de refleccion especular ideal
		wr.normalize();
		Vector wt;  							// direccion de transmision

		wt = ((wi*-1) * n2) + nv*(n2 * cosTi - cosTt);
		wt.normalize();

		if (reflect){
			return wr;
		}
		else return wt;
	}

};


#endif // BSDF_H