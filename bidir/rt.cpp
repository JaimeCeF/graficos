// rt: un lanzador de rayos minimalista
// g++ -O3 -fopenmp rt.cpp -o rt

#include <math.h>
#include <stdlib.h>
#include <stdio.h>  
#include <omp.h>
#include <vector>
#include <random>
#include <algorithm> // std::min_element
#include <iterator>  // std::begin, std::end
#include <iostream>
#include <unistd.h>
#include <map>

#include "sphere.h"
#include "sampling_helpers.h"
#include "bsdf.h"
#include "light_path.h"

using namespace std;

// Sphere spheres[] = {
// 	//Escena: radio, posicion, color, emision, material
// 	Sphere(1e5,  Point(-1e5 - 49, 0, 0),   Color(.75, .25, .25), Color(),         diffuse), // pared izq
// 	Sphere(1e5,  Point(1e5 + 49, 0, 0),    Color(.25, .25, .75), Color(),	      diffuse), // pared der
// 	Sphere(1e5,  Point(0, 0, -1e5 - 81.6), Color(.25, .75, .25), Color(),	      diffuse), // pared detras
// 	Sphere(1e5,  Point(0, -1e5 - 40.8, 0), Color(.25, .75, .75), Color(),	      diffuse), // suelo
// 	Sphere(1e5,  Point(0, 1e5 + 40.8, 0),  Color(.75, .75, .25), Color(),	      diffuse), // techo
// 	Sphere(16.5, Point(-23, -24.3, -34.6), Color(1, 1, 1),	     Color(),	      specular), // esfera espejo
// 	Sphere(16.5, Point(23, -24.3, -3.6),   Color(1, 1, 1), 	     Color(),	      dielectric), // esfera dielectrica
// 	Sphere(1.5, Point(-40, -39.3, -60),        Color(0, 0, 0),       Color(600,600,600), diffuse)  // esfera de luz
// };

Sphere spheres[] = {
	//Escena: radio, posicion, color, emision, material
	Sphere(1e5,  Point(-1e5 - 49, 0, 0),   Color(.75, .25, .25), Color(),         diffuse), // pared izq
	Sphere(1e5,  Point(1e5 + 49, 0, 0),    Color(.25, .25, .75), Color(),	      diffuse), // pared der
	Sphere(1e5,  Point(0, 0, -1e5 - 81.6), Color(.25, .75, .25), Color(),	      diffuse), // pared detras
	Sphere(1e5,  Point(0, -1e5 - 40.8, 0), Color(.25, .75, .75), Color(),	      diffuse), // suelo
	Sphere(1e5,  Point(0, 1e5 + 40.8, 0),  Color(.75, .75, .25), Color(),	      diffuse), // techo
	Sphere(16.5, Point(-23, -24.3, -34.6), Color(1, 1, 1),	     Color(),	      specular), // esfera espejo
	Sphere(16.5, Point(23, -24.3, -3.6),   Color(1, 1, 1),       Color(), 		  dielectric), // esfera dielectrica
	Sphere(10.5, Point(0, 24.3, 0),        Color(0, 0, 0),       Color(10,10,10), diffuse)  // esfera de luz
};

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

/* Shoot shadow ray from object intersection from camera path (if object material is diffuse) to each intersection of the light path
 * lightPath: The path which was calculated by the makeLightPath() function
 * obj: the object that was hit by a camera ray
 * x: intersection point where the object was hit
 * id: id of the hit object
 * returns Color influence to Ray
 * */
Color Gterm(const std::vector<LightPath> &lightPath, const Sphere &obj, const Point &x, Vector &n, int id) {
	Vector G;

	if(obj.mat == diffuse) {
		for(int i = 0; i < lightPath.size(); i++) {
			//create directional vector between camera hitpoint and light path hitpoint
			Vector dir = lightPath[i].x - x ;
			Vector dirNorm = dir.normalize();
			//is there a shadowray?
			// std::cout << "Conection?: " << checkOcclusion(lightPath[i].x, dir_norm*-1., id) << std::endl;
			if(checkOcclusion(lightPath[i].x, dirNorm*-1., id)) {
				// normal of light path hit point
				Vector lightPathNorm = (lightPath[i].x - spheres[lightPath[i].obj].p).normalize();

				//the diffuse PDF for the enlightend object
				double diff_enlight_pdf = fabs((dirNorm * -1.).dot(lightPathNorm)) * invPi;
				//the diffuse PDF for the camera object
				double diff_eye_pdf = fabs(dirNorm.dot(n)) * invPi;

				double distance_sqr = x.squaredDistance(lightPath[i].x);
				// std::cout << "Sqr dist: " << distance_sqr << std::endl;
				//First light constant which has to be added
				G = G + lightPath[i].color * diff_enlight_pdf * (diff_eye_pdf / distance_sqr);
				//e = e + (light_paths[i].absorbed_color * (dir.Dot(object.GetNormal(hit_point) * diffPdf))) / M_PI;

			}
			else {
				//Ignore if they don't see eachother
				continue;	
			}

		}
	}
	
	return G;
}
  
Color directLightValue(Vector &x, Vector &n, Color &bsdf,double &continueprob){
	Color directLight;
	std::map<int,Sphere>::const_iterator it;
	for(it = lights.begin(); it != lights.end(); it++) {
		const Sphere &temp = ((it)->second);
		if (temp.e.x <= 0 && temp.e.y <= 0 && temp.e.z <= 0)
			continue;
		
		double theta1, phi1, cosTmax, probLight;
		paramSolidAngle(x, theta1, phi1, cosTmax, temp);
		Vector wc = (temp.p - x).normalize();
		Vector wl = sampleDir(wc, theta1, phi1).normalize();
		double t;
		int id = 0;
		if (intersect(Ray(x, wl), t, id) && id == it->first){	// si no hay oclusion, calcular iluminacion directa
			Color Le = temp.e;
			double dotCos1 = n.dot(wl);
			probLight = probSolidAngle(cosTmax);
			directLight = directLight + Le.mult(bsdf * fabs(dotCos1) * (1.0 / (probLight*continueprob)));
		}
	}
	return directLight;
}



// Calcula el valor de color para el rayo dado
// Color shade(const Ray &ray, const std::vector<LightPath> &lightPath) {
// 	double t;
// 	int id = 0;
// 	int bounce = 0;
// 	int maxBounce = 5;
// 	bool checkLightPath = true;
// 	Ray r = ray;
// 	Vector gatheredColor(0,0,0);
// 	Vector gatheredRefl(1,1,1);

// 	for(int i = 0;; i++) {

// 		// determinar que esfera (id) y a que distancia (t) el rayo intersecta
// 		if (!intersect(r, t, id))
// 			return Color();	// el rayo no intersecto objeto, return Vector() == negro
	
// 		const Sphere &obj = spheres[id];  //esfera sobre la que se intersecto

// 		// std::cout << "Sphere index:" << id << std::endl;

// 		if (++bounce > maxBounce) return gatheredColor;

// 		// std::cout << "Bounces " << bounce << std::endl;

// 		// determinar coordenadas del punto de interseccion
// 		Point x = r.o + r.d*t;

// 		// determinar la dirección normal en el punto de interseccion
// 		Vector n = (x - obj.p).normalize();

// 		// determinar si un rayo choca con un objeto por dentro
// 		// si es el caso,  voltear  la normal (nv)
// 		Vector nv;
// 		if (n.dot(r.d * -1) > 0) {nv = n;}
// 		else {nv = n * -1;}

// 		// color del objeto intersectado
// 		Color baseColor = obj.c;
		
// 		if(checkLightPath || obj.mat != diffuse)
// 			gatheredColor = gatheredColor + gatheredRefl.mult(obj.e);

// 		// ruleta rusa
// 		double q = 0.1;
// 		double continueprob = 1.0 - q;
// 		if (dis(gen) < q) return gatheredColor;

// 		gatheredRefl = gatheredRefl.mult(baseColor);

// 		// determinar el color que se regresara
		
// 		Vector newDir;

// 		// material difuso

// 		if (obj.mat == diffuse) {
// 			Vector newDir = DirectionBSDF::DiffuseBSDF(n);
// 			Color bsdf = BRDF(baseColor);
// 			bool notLight = lights.find(id) == lights.end();

// 			if(!notLight && i == 0){
// 				gatheredColor = gatheredColor + gatheredRefl.mult(obj.e);
// 			}
// 			else if(notLight || checkLightPath){
// 				gatheredColor = gatheredColor + gatheredRefl.mult(directLightValue(x, n, bsdf, continueprob));
// 			}
// 			checkLightPath = false;
// 			Vector G = shadowRay(lightPath, obj, x, n, id);
// 			// std::cout << "Geometric x value: " << G.x << std::endl;
// 			gatheredColor = gatheredColor + gatheredRefl.mult(G);

// 			// double probMat = probCosineHemisphere(wi, n);
// 			// double dotCos = n.dot(wi);
// 			// Ray newRay = Ray(x, wi);

// 			// Color indirectLight = bsdf.mult(shade(newRay, bounce, 0)) * (fabs(dotCos)/(probMat*continueprob));
// 			// Color directLight = directLightValue(x, n, bsdf, continueprob);
// 			// return obj.e * cond + directLight + indirectLight;
// 		}
		
// 		// material especular 

// 		else if (obj.mat == specular){
// 			newDir = DirectionBSDF::SpecularBSDF(r, n);  // direccion de refleccion especular ideal
// 			checkLightPath = true;
// 			// wr.normalize();
// 			// return baseColor.mult(shade(Ray(x, wr), bounce, 1));
// 		}

// 		// material dielectrico

// 		else if (obj.mat == dielectric) {
// 			double n2, ni = 1.0, nt = 1.5;
// 			Vector wi = r.d * -1;

// 			double cosTi = (wi).dot(n);
// 			cosTi = Clamp(cosTi, -1, 1);
// 			bool out2in = cosTi > 0.f;
// 			if(!out2in){
// 				swap(ni, nt);
// 				cosTi = fabs(cosTi);
// 			}
// 			n2 = ni/nt;

// 			double cosTt = snell(cosTi, n2);

// 			double F = dielectricFresnel(ni, nt, n2, cosTi, cosTt);

// 			bool reflect = dis(gen) < F;

// 			Vector newDir = DirectionBSDF::DielectricBSDF(r, wi, nv, n2, cosTi, cosTt, reflect);

// 			checkLightPath = true;

// 			// double prob = probDielectric(F, reflect);

// 			// Ray newRay = Ray(x, wo);

// 			// if (reflect){
// 			// 	double f = F / fabs(cosTi);
// 			// 	return obj.e + baseColor.mult(shade(newRay, bounce, 1)) * (f*fabs(nv.dot(wo))/prob);
// 			// }
// 			// else {
// 			// 	double f = (((nt*nt)/(ni*ni))*(1 - F)) / fabs(cosTt);
// 			// 	return obj.e + baseColor.mult(shade(newRay, bounce, 1))*(f*fabs(nv.dot(wo))/prob);
// 			}
// 		else{
// 			newDir = Vector();
// 			checkLightPath = true;
// 		}

// 		r = Ray(x, newDir);
// 	}
	
// }

Color shade(
	const Ray &ray,
	const std::vector<LightPath> &lightPath,
	int bounce,
	int isFirstBounce,
	Vector gatheredColor,
	bool checkLightPath,
	Vector gatheredRefl
) {
	Ray r = ray;
	double t;
	int id = 0;

	// determinar que esfera (id) y a que distancia (t) el rayo intersecta
	if (!intersect(r, t, id))
		return Color();	// el rayo no intersecto objeto, return Vector() == negro
  
	const Sphere &obj = spheres[id];  //esfera sobre la que se intersecto

	if (++bounce > 10) return Color();

	// determinar coordenadas del punto de interseccion
	Point x = r.o + r.d*t;

	// determinar la dirección normal en el punto de interseccion
	Vector n = (x - obj.p).normalize();

	// determinar si un rayo choca con un objeto por dentro
	// si es el caso,  voltear  la normal (nv)
	Vector nv;
	if (n.dot(r.d * -1) >= 0) {nv = n;}
	else {nv = n * -1;}

	// color del objeto intersectado
	Color baseColor = obj.c;
	
	// ruleta rusa
	double q = 0.1;
	double continueprob = 1.0 - q;
	if (dis(gen) < q) return Color();

	if(checkLightPath || obj.mat != diffuse) {
		gatheredColor = gatheredColor + gatheredRefl.mult(obj.e);
	}

	// determinar el color que se regresara
	
	// material difuso

	if (obj.mat == 0) {

		// obtener una direccion aleatoria con muestreo de coseno hemisferico, wi
		Vector newDir = DirectionBSDF::DiffuseBSDF(n);
		Color bsdf = BRDF(baseColor);
		double dotCos = nv.dot(newDir);
		double probMat = probCosineHemisphere(newDir, nv);
		Ray newRay = Ray(x, newDir);

		bool notLight = lights.find(id) == lights.end();

		if(!notLight && isFirstBounce){
			gatheredColor = gatheredColor + gatheredRefl.mult(obj.e);
		}
		// else if(notLight || checkLightPath){
		// 	gatheredColor = gatheredColor + gatheredRefl.mult(directLightValue(x, n, bsdf, continueprob));
		// }

		Vector G = Gterm(lightPath, obj, x, n, id);
		gatheredColor = gatheredColor + gatheredRefl.mult(G);

		// calculo de iluminacion indirecta

		Color indirectLight = bsdf.mult(shade(newRay, lightPath, bounce + 1, 0, gatheredColor, false, gatheredRefl.mult(baseColor))) * (fabs(dotCos)/(probMat*continueprob));

		// para cada esfera en la escena, checar cuales son fuentes de luz

		Color directLight;
		for (int i = 0; i < totalShperes; i++){
			
			const Sphere &temp = spheres[i];
			if (temp.e.x <= 0 && temp.e.y <= 0 && temp.e.z <= 0)	// si la esfera i no tiene emision, saltarla
				continue;

			// si la esfera i es una fuente de luz, realizar muestreo de angulo solido, wl
			
			double theta1, phi1, cosTmax, probLight;
			paramSolidAngle(x, theta1, phi1, cosTmax, temp);
			Vector wc = (temp.p - x).normalize();
			Vector wl = sampleDir(wc, theta1, phi1).normalize();
			if (intersect(Ray(x, wl), t, id) && id == i){	// si no hay oclusion, calcular iluminacion directa
				Color Le = temp.e;
				double dotCos1 = nv.dot(wl);
				probLight = probSolidAngle(cosTmax);
				directLight = directLight + Le.mult(bsdf * fabs(dotCos1) * (1.0 / (probLight*continueprob)));
			}
		}
		return obj.e * isFirstBounce + directLight + indirectLight;
	}
	
	// material especular 

	if (obj.mat == 1){
		Vector newDir = DirectionBSDF::SpecularBSDF(r, nv);
		return baseColor.mult(shade(Ray(x, newDir), lightPath, bounce + 1, 1, gatheredColor, false, gatheredRefl.mult(baseColor)));
	}

	// material dielectrico

	if (obj.mat == 2) {
		double n2, ni = 1.0, nt = 1.5, F, rpar, rper;
		Vector wi = r.d * -1;

		// ley de snell para calcular cosTt
		double cosTi = (wi).dot(n);
		cosTi = Clamp(cosTi, -1, 1);
		bool out2in = cosTi > 0.f;
		if(!out2in){
			swap(ni, nt);
			cosTi = fabs(cosTi);
		}
		n2 = ni/nt;

		double sinTi = sqrt(max(0.0, 1.0 - cosTi * cosTi));
		double sinTt = n2 * sinTi;
		double cosTt = sqrt(max(0.0, 1 - sinTt * sinTt));

		// calculo de Fresnel
		if (sinTt >= 1){
			F = 0;
		}
		else{
			rpar = ((nt*cosTi) - (ni*cosTt))/((nt*cosTi) + (ni*cosTt));
			rper = ((ni*cosTi) - (nt*cosTt))/((ni*cosTi) + (nt*cosTt));
			F = (rpar*rpar + rper*rper) * 0.5;
		}

		Vector wr = r.d - nv*2*((nv.dot(r.d)));  // direccion de refleccion especular ideal
		wr.normalize();
		double cosTr = wr.dot(nv);
		Vector wt;  							// direccion de transmision

		wt = ((wi*-1) * n2) + nv*(n2 * cosTi - cosTt);
		wt.normalize();

		Ray refractionRay = Ray(x, wt);
		Ray reflectionRay = Ray(x, wr);

		bool reflect = dis(gen) < F;
		double pr = F;
		double pt = 1.0 - F;

		double fr = F / fabs(cosTi);
		double ft = (((nt*nt)/(ni*ni))*(1 - F)) / fabs(cosTt);

		if (reflect) {
			return obj.e + baseColor.mult(shade(reflectionRay, lightPath, bounce + 1, 1, gatheredColor, false, gatheredRefl.mult(baseColor))) * (fr*fabs(nv.dot(wr))/pr);
		}

		return obj.e + baseColor.mult(shade(refractionRay, lightPath, bounce + 1, 1, gatheredColor, false, gatheredRefl.mult(baseColor)))*(ft*fabs(nv.dot(wt))/pt);

		// return baseColor.mult(shade(refractionRay, bounce, 1)*(ft/pt) + shade(Ray(x, wr), bounce, 1)*(fr/pr));
	}
		

	else return Color();
}

int main(int argc, char *argv[]) {

	getLights();
	std::cout << "Total lights: " << totalLights << std::endl;
	
	int w = 1024, h = 768; // image resolution

	int N = 32;  // numero de muestras

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
				// if (x == 0 && y == 0 && n < 10) {
				// 	std::cout << "Light Paths for the First Pixel:" << std::endl;
				// 	std::cout << "Light Path size: " << lightPath.size() << std::endl;
				// 	std::cout << "Iteration " << n << ": " << std::endl;
				// 	std::cout << "Position (x): " << lightPath[4].x << std::endl;
				// 	std::cout << "Color: " << lightPath[4].color << std::endl;
				// 	std::cout << "Associated Light: " << lightPath[4].light << std::endl;
				// 	std::cout << "Associated Object: " << lightPath[4].obj << std::endl;
            	// }
				int idx = (h - y - 1) * w + x; // index en 1D para una imagen 2D x,y son invertidos
				Color pixelValue = Color(); // pixelValue en negro por ahora
				// para el pixel actual, computar la dirección que un rayo debe tener
				Vector cameraRayDir = cx * ( double(x)/w - .5) + cy * ( double(y)/h - .5) + camera.d;

				// computar el color del pixel para el punto que intersectó el rayo desde la camara
				pixelValue = pixelValue + shade( Ray(camera.o, cameraRayDir.normalize()), lightPath, 0, 1, Vector(0,0,0), true, Vector(1,1,1)) * (1.0/N);

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