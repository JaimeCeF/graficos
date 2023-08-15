// rt: un lanzador de rayos minimalista
// g++ -O3 -fopenmp rt.cpp -o rt

#include <math.h>
#include <stdlib.h>
#include <stdio.h>  
#include <omp.h>
#include <vector>
#include <random>
#include <iostream>
#include <unistd.h>
#include <map>

#include "sphere.h"
#include "sampling_helpers.h"
#include "bsdf.h"
#include "light_path.h"

using namespace std;

Sphere spheres[] = {
	//Escena: radio, posicion, color, emision, material
	Sphere(1e5,  Point(-1e5 - 49, 0, 0),   Color(.75, .25, .25), Color(),         diffuse), // pared izq
	Sphere(1e5,  Point(1e5 + 49, 0, 0),    Color(.25, .25, .75), Color(),	      diffuse), // pared der
	Sphere(1e5,  Point(0, 0, -1e5 - 81.6), Color(.25, .75, .25), Color(),	      diffuse), // pared detras
	Sphere(1e5,  Point(0, -1e5 - 40.8, 0), Color(.25, .75, .75), Color(),	      diffuse), // suelo
	Sphere(1e5,  Point(0, 1e5 + 40.8, 0),  Color(.75, .75, .25), Color(),	      diffuse), // techo
	Sphere(16.5, Point(-23, -24.3, -34.6), Color(1, 1, 1),	     Color(),	      specular), // esfera espejo
	Sphere(10.5, Point(-40, -30.3, -34.6), Color(1, 1, 1),	     Color(),	      diffuse), // esfera difusa
	Sphere(10.5, Point(-23, -30.3, -60), Color(1, 1, 1),	     Color(),	      diffuse), // esfera difusa
	Sphere(16.5, Point(23, -24.3, -3.6),   Color(1, 1, 1), 	     Color(),	      dielectric), // esfera dielectrica
	Sphere(1.5, Point(-40, -39.3, -60),        Color(0, 0, 0),       Color(600,600,600), diffuse)  // esfera de luz
};

// Sphere spheres[] = {
// 	//Escena: radio, posicion, color, emision, material
// 	Sphere(1e5,  Point(-1e5 - 49, 0, 0),   Color(.75, .25, .25), Color(),         diffuse), // pared izq
// 	Sphere(1e5,  Point(1e5 + 49, 0, 0),    Color(.25, .25, .75), Color(),	      diffuse), // pared der
// 	Sphere(1e5,  Point(0, 0, -1e5 - 81.6), Color(.25, .75, .25), Color(),	      diffuse), // pared detras
// 	Sphere(1e5,  Point(0, -1e5 - 40.8, 0), Color(.25, .75, .75), Color(),	      diffuse), // suelo
// 	Sphere(1e5,  Point(0, 1e5 + 40.8, 0),  Color(.75, .75, .25), Color(),	      diffuse), // techo
// 	Sphere(16.5, Point(-23, -24.3, -34.6), Color(1, 1, 1),	     Color(),	      specular), // esfera espejo
// 	Sphere(16.5, Point(23, -24.3, -3.6),   Color(1, 1, 1),       Color(), 		  dielectric), // esfera dielectrica
// 	Sphere(10.5, Point(0, 24.3, 0),        Color(0, 0, 0),       Color(10,10,10), diffuse)  // esfera de luz
// };

double totalShperes = sizeof(spheres)/sizeof(Sphere);

// Calculate the intersection of ray 'r' with all the spheres
// Return true if there was an intersection, false otherwise
// Store the distance where the intersection occurs in 't'
// Store the index from spheres[] of the closest intersected object in 'id'
inline bool intersect(const Ray &r, double &t, int &id) {
	double dist;
	double thresh = t = 100000000000;

	for (int i=0;i < totalShperes;i++) {
		dist = spheres[i].intersect(r);
		if (dist && dist > 0.01 && dist < t) {
			t = dist;
			id = i;
		}
	}
	if (t < thresh) return true;
	else return false;
}

// Light sources in the scene (id, light object)
std::map<int, Sphere> lights;
// Amount of lights in scene
int totalLights;

// Identify light sources in the scene
// Store their 'spheres' list index and associated Sphere object in 'lights' map
// Update value of 'totalLights'
void getLights(){
	for (int i = 0; i < totalShperes; i++) {
		Sphere obj = spheres[i];
		if (obj.e.x <= 0 && obj.e.y <= 0 && obj.e.z <= 0)
			continue;
		else {
			lights.insert(std::make_pair(i, obj));
		}
	}
	totalLights = lights.size();
}

double getCosTmax(Ray &brdfRay, Point &x) {
	double t;
	int id;
	intersect(brdfRay, t, id);
	const Sphere &obj = spheres[id];

	double r = obj.r;
    double rand1 = dis(gen);
    double rand2 = dis(gen);
	Vector wc = (obj.p - x);
	double lengthWc = sqrt((wc.x * wc.x) + (wc.y * wc.y) + (wc.z * wc.z));
	double aux = r / lengthWc;
	return sqrt(1.0 - (aux * aux));
}

// Create array 'lightPath' of LightPath objects by shooting a ray from each light source in the scene
// and having it bounce 'bounces' amount of times
// Only adds LightPath imformation to array if hit object is diffuse
std::vector<LightPath> makeLightPath(const int bounces) {
	std::vector<LightPath> lightPath;
	lightPath.reserve(bounces*totalLights);
	std::map<int,Sphere>::const_iterator it;

	for(it = lights.begin(); it != lights.end(); it++) {
		
		// Current light, light position
		Sphere light = it->second;
		Point lightPosition = light.p;

		// Create random direction from light by uniformly sampling light sphere, normalize direction
		Vector dir = sphereUniformDir();
		dir.normalize();
		
		// Shoot ray from light source in direction 'dir'
		Ray r = Ray(lightPosition + dir * light.r, dir);
		double t;
		int id = 0;
		
		// Gathered color ('color') and gathered reflectance factor ('reflectance')
		Vector color(0,0,0);
		Vector reflectance(1,1,1);
		
		for(int i = 0; i < bounces; i++) {

			// Determine which sphere (id) and at what distance (t) the ray (r) intersects
			if(!intersect(r, t, id)) {
				continue;
			}
			// Hit object
			const Sphere &obj = spheres[id];	

			// Hit point
			Vector x = r.o + r.d*t;	

			// Normal at hit point
			Vector n = (x - obj.p).normalize();

			// If object was hit from inside, flip normal
			Vector nv;
			if (n.dot(r.d * -1) > 0) {nv = n;}
			else {nv = n * -1;}

			Color baseColor = obj.c;
			reflectance = reflectance.mult(baseColor);
			color = reflectance.mult(light.e);

			// Create new direction based on object BSDF
			// If hit object is diffuse, store information to lightPath
			Vector dir;
			if (obj.mat == diffuse) {
				lightPath.push_back(LightPath(x, color, it->first, id));
				dir = DirectionBSDF::DiffuseBSDF(nv);
			}
			else if (obj.mat == specular) {
				dir = DirectionBSDF::SpecularBSDF(r, nv);
			}
			else if(obj.mat == dielectric) {
				double eta, etai = 1.0, etat = 1.5;
				Vector wi = r.d * -1;

				double cosTi = (wi).dot(n);
				cosTi = Clamp(cosTi, -1, 1);
				bool out2in = cosTi > 0.f;
				if(!out2in){
					swap(etai, etat);
					cosTi = fabs(cosTi);
				}
				eta = etai/etat;

				double cosTt = snell(cosTi, eta);

				double F = dielectricFresnel(etai, etat, eta, cosTi, cosTt);

				bool reflect = dis(gen) < F;

				dir = DirectionBSDF::DielectricBSDF(r, wi, nv, eta, cosTi, cosTt, reflect);
			}
			else {
				dir = Vector();
			}

			// Update ray 'r'
			r = Ray(x, dir);
		}
	}

	return lightPath;
}

// Determine value of fs for diffuse surface
Color BRDF(Color c) {
	Color brdf = c * invPi;
	return brdf;
}

// Check for occlusion between light point and an object point
// If there's occlusion, return false, true otherwise
bool checkOcclusion(const Point &lightPoint, const Vector &objDirection, int id) {

	Ray r(lightPoint, objDirection);
	int id1 = 0;
	double t;
	if(intersect(r, t, id1)) {
		return (id1 == id);
	}
	return false;
}

// Check if ray hits a light source
bool checkHitLight(Ray &brdfRay) {
	double t;
	int id;

	if (intersect(brdfRay, t, id)) {
		const Sphere &obj = spheres[id];
		if (obj.e.x > 0.0 && obj.e.y > 0.0 && obj.e.z > 0.0) {
			return true;
		}
		else return false;
	}
	else return false;
}

// Shoot shadow ray from camera object intersection (if object material is diffuse) to each intersection of the light path and return geometric term
double Gterm(const std::vector<LightPath> &lightPath, const Sphere &obj, const Point &x, Vector &n, int id) {
	double G;

	if(obj.mat == diffuse) {
		for(int i = 0; i < lightPath.size(); i++) {
			// Direction vector between camera hit point (x) and light path hit point (lightPath[i].x)
			Vector dir = lightPath[i].x - x ;
			Vector dirNorm = dir.normalize();

			// Check for occlusion
			if(checkOcclusion(lightPath[i].x, dirNorm*-1., id)) {
				// Normal of light path hit point
				Vector lightPathNorm = (lightPath[i].x - spheres[lightPath[i].obj].p).normalize();

				// Dot product between normal at camera object hit point and direction to light path hit point
				double cameraObjDot = fabs(dirNorm.dot(n))*invPi;

				// Dot product between normal at light path hit point and direction to camera object hit point
				double lightObjDot = fabs((dirNorm * -1.).dot(lightPathNorm))*invPi;

				// Squared distance between x and lightPath[i].x
				double sqrdDist = x.squaredDistance(lightPath[i].x);

				G = G + ((lightObjDot * cameraObjDot) / sqrdDist);

			}
			else {
				//Ignore if there's occlusion
				continue;	
			}

		}
	}
	
	return G;
}


// Compute contribution from direct light sample and multiply it by MIS factor
Color directLightValue(Vector &x, Vector &n, Color &bsdf, double &continueprob){
	Color directLight;
	std::map<int,Sphere>::const_iterator it;
	for(it = lights.begin(); it != lights.end(); it++) {
		const Sphere &temp = ((it)->second);
		if (temp.e.x <= 0 && temp.e.y <= 0 && temp.e.z <= 0)
			continue;
		
		double theta1, phi1, cosTmax, probLight, probMat;
		paramSolidAngle(x, theta1, phi1, cosTmax, temp);
		Vector wc = (temp.p - x).normalize();
		Vector wl = sampleDir(wc, theta1, phi1).normalize();
		double t;
		int id = 0;
		if (intersect(Ray(x, wl), t, id) && id == it->first){
			Color Le = temp.e;
			double dotCos1 = n.dot(wl);
			probLight = probSolidAngle(cosTmax);
			probMat = probCosineHemisphere(wl, n);
			double wg = PowerHeuristic(probLight, probMat);
			directLight = directLight + Le.mult(bsdf * fabs(dotCos1) * (1.0 / (probLight*continueprob)))*wg;
		}
	}
	return directLight;
}



// Determine color value for a given ray
Color shade(
	const Ray &ray,
	const std::vector<LightPath> &lightPath,
	int bounce,
	int considerLightEmission,
	Vector gatheredColor,
	Vector gatheredRefl
) {
	Ray r = ray;
	double t;
	int id = 0;

	if (bounce++ > 5) return Color();

	// Determine which sphere (id) and at what distance (t) the ray (r) intersects
	if (!intersect(r, t, id))
		return Color();
	
	// Hit object
	const Sphere &obj = spheres[id];  //esfera sobre la que se intersecto

	// Hit point
	Point x = r.o + r.d*t;

	// Normal at hit point
	Vector n = (x - obj.p).normalize();

	// If object was hit from inside, flip normal
	Vector nv;
	if (n.dot(r.d * -1) >= 0) {nv = n;}
	else {nv = n * -1;}

	Color baseColor = obj.c;
	
	// Russian roulette
	double q = 0.1;
	double continueprob = 1.0 - q;
	if (dis(gen) < q) return Color();

	if (obj.mat == diffuse) {

		Color indirectLight;

		// Obtain new direction (newdir) using cosine hemisphere sampling
		Vector newDir = DirectionBSDF::DiffuseBSDF(n);
		Color bsdf = BRDF(baseColor);
		double dotCos = nv.dot(newDir);
		double probMat = probCosineHemisphere(newDir, nv);
		Ray newRay = Ray(x, newDir);

		// Compute geometric term contribution
		double G = Gterm(lightPath, obj, x, n, id);
		gatheredColor = gatheredColor + gatheredRefl*(G);

		// Compute indirect illumination, only if direction goes towards light sphere
		// Compute MIS factor for BSDF sample
		// If sampled direction does not hit light sphere, BSDF sample is zero
		if (checkHitLight(newRay)){
			double cosTmax = getCosTmax(newRay, x);
			double probLight = (probSolidAngle(cosTmax));
			double wf = PowerHeuristic(probMat, probLight);
			indirectLight = bsdf.mult(shade(newRay, lightPath, bounce + 1, 0, gatheredColor, gatheredRefl.mult(baseColor))) *(G/(probMat*continueprob))*wf;
		}
		else indirectLight = Color();

		// Compute direct illumination
		Color directLight = directLightValue(x, nv, bsdf, continueprob);

		// Only multiply by object emission if throwing first camera ray for a given sample (to display light emitter)
		return obj.e * considerLightEmission + directLight + indirectLight;
	}
	
	// material especular 

	if (obj.mat == specular){
		Vector newDir = DirectionBSDF::SpecularBSDF(r, nv);
		return obj.e * considerLightEmission + baseColor.mult(shade(Ray(x, newDir), lightPath, bounce++, 1, gatheredColor, gatheredRefl.mult(baseColor)));
	}

	// material dielectrico

	if (obj.mat == dielectric) {
		double eta, eta1 = 1.0, eta2 = 1.5;
		Vector wi = r.d * -1;

		// Snell's law to compute cosTt
		double cosTi = (wi).dot(n);
		cosTi = Clamp(cosTi, -1, 1);
		bool out2in = cosTi > 0.f;
		if(!out2in){
			swap(eta1, eta2);
			cosTi = fabs(cosTi);
		}
		eta = eta1/eta2;

		double cosTt = snell(cosTi, eta);

		// Fresnel computation
		double F = dielectricFresnel(eta1, eta2, eta, cosTi, cosTt);

		bool reflect = dis(gen) < F;

		Vector newDir = DirectionBSDF::DielectricBSDF(r, wi, nv, eta, cosTi, cosTt, reflect);

		double prob = probDielectric(F, reflect);

		double fr = F / fabs(cosTi);
		double ft = (((eta2*eta2)/(eta1*eta1))*(1 - F)) / fabs(cosTt);

		if (reflect) {
			return obj.e * considerLightEmission + baseColor.mult(shade(Ray(x, newDir), lightPath, bounce++, 1, gatheredColor, gatheredRefl.mult(baseColor))) * (fr*fabs(nv.dot(newDir))/prob);
		}

		return obj.e * considerLightEmission + baseColor.mult(shade(Ray(x, newDir), lightPath, bounce++, 1, gatheredColor, gatheredRefl.mult(baseColor)))*(ft*fabs(nv.dot(newDir))/prob);
	}
		

	else return Color();
}

int main(int argc, char *argv[]) {

	getLights();
	
	int w = 1024, h = 768; // image resolution

	int N = 512;  // numero de muestras

	// fija la posicion de la camara y la dirección en que mira
	Ray camera( Point(0, 11.2, 214), Vector(0, -0.042612, -1).normalize() );

	// parametros de la camara
	Vector cx = Vector( w * 0.5095 / h, 0., 0.); 
	Vector cy = (cx % camera.d).normalize() * 0.5095;
  
	// auxiliar para valor de pixel y matriz para almacenar la imagen
	Color *pixelColors = new Color[w * h];

	// PROYECTO 1
	// usar openmp para paralelizar el ciclo: cada hilo computara un renglon (ciclo interior),
	#pragma omp parallel
	#pragma omp for schedule(dynamic, 1)

	for(int y = 0; y < h; y++) 
	{ 
		// recorre todos los pixeles de la imagen
		fprintf(stderr,"\r%5.2f%%",100.*y/(h-1));

		for(int x = 0; x < w; x++ ) {
			for (int n=0; n<N; n++){

				std::vector<LightPath> lightPath = makeLightPath(5);
				int idx = (h - y - 1) * w + x; // index en 1D para una imagen 2D x,y son invertidos
				Color pixelValue = Color(); // pixelValue en negro por ahora
				// para el pixel actual, computar la dirección que un rayo debe tener
				Vector cameraRayDir = cx * ( double(x)/w - .5) + cy * ( double(y)/h - .5) + camera.d;

				// computar el color del pixel para el punto que intersectó el rayo desde la camara
				pixelValue = pixelValue + shade( Ray(camera.o, cameraRayDir.normalize()), lightPath, 0, 1, Vector(0,0,0), Vector(1,1,1)) * (1.0/N);

				// limitar los tres valores de color del pixel a [0,1]
				pixelColors[idx] = pixelColors[idx] + Color(clamp(pixelValue.x), clamp(pixelValue.y), clamp(pixelValue.z));
			}
		}
	}

	fprintf(stderr,"\n");

	FILE *f = fopen("image.ppm", "w");
	// escribe cabecera del archivo ppm, ancho, alto y valor maximo de color
	fprintf(f, "P3\n%d %d\n%d\n", w, h, 255); 
	for (int p = 0; p < w * h; p++) 
	{ // escribe todos los valores de los pixeles
    		fprintf(f,"%d %d %d ", toDisplayValue(pixelColors[p].x), toDisplayValue(pixelColors[p].y), 
				toDisplayValue(pixelColors[p].z));
  	}
  	fclose(f);

  	delete[] pixelColors;

	return 0;
}