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
using namespace std;

double invPi = 1.0 / M_PI;

std::random_device rd;  // Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> dis(0.0, 1.0);


class Vector 
{
public:        
	double x, y, z; // coordenadas x,y,z 
  
	// Constructor del vector, parametros por default en cero
	Vector(double x_= 0, double y_= 0, double z_= 0){ x=x_; y=y_; z=z_; }
  
	// operador para suma y resta de vectores
	Vector operator+(const Vector &b) const { return Vector(x + b.x, y + b.y, z + b.z); }
	Vector operator-(const Vector &b) const { return Vector(x - b.x, y - b.y, z - b.z); }

	// operator multiplicacion vector y escalar 
	Vector operator*(double b) const { return Vector(x * b, y * b, z * b); }
  
	// operator % para producto cruz
	Vector operator%(Vector&b){return Vector(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x); }
	
	// producto punto con vector b
	double dot(const Vector &b) const { return x * b.x + y * b.y + z * b.z; }

	// producto elemento a elemento (Hadamard product)
	Vector mult(const Vector &b) const { return Vector(x * b.x, y * b.y, z * b.z); }
	
	// normalizar vector 
	Vector& normalize(){ return *this = *this * (1.0 / sqrt(x * x + y * y + z * z)); }

	// checar si dos vectores son iguales
	bool equals(const Vector &b){ return fabs(x - b.x) < 0.01 && fabs(y - b.y) < 0.01 && fabs(z - b.z) < 0.01; }
};
typedef Vector Point;
typedef Vector Color;

class Ray 
{ 
public:
	Point o; // origen del rayo
	Vector d; // direcccion del rayo
	Ray(Point o_, Vector d_) : o(o_), d(d_) {} // constructor
};

class Sphere 
{
public:
	double r;	// radio de la esfera
	Point p;	// posicion
	Color c;	// color  
	Color e;	// emision
	int mat;	// surface type, 0 = diffuse, 1 = specular, 2 = dielectric

	Sphere(double r_, Point p_, Color c_, Color e_, int mat_): r(r_), p(p_), c(c_), e(e_), mat(mat_){}
  
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

Sphere spheres[] = {
	//Escena: radio, posicion, color, emision
	Sphere(1e5,  Point(-1e5 - 49, 0, 0),   Color(.75, .25, .25), Color(),         0), // pared izq
	Sphere(1e5,  Point(1e5 + 49, 0, 0),    Color(.25, .25, .75), Color(),		  0), // pared der
	Sphere(1e5,  Point(0, 0, -1e5 - 81.6), Color(.25, .75, .25), Color(),		  0), // pared detras
	Sphere(1e5,  Point(0, -1e5 - 40.8, 0), Color(.25, .75, .75), Color(),		  0), // suelo
	Sphere(1e5,  Point(0, 1e5 + 40.8, 0),  Color(.75, .75, .25), Color(),		  0), // techo
	Sphere(16.5, Point(-23, -24.3, -34.6), Color(1, 1, 1),	     Color(),		  1), // esfera abajo-izq
	Sphere(16.5, Point(23, -24.3, -3.6),   Color(0.26, 0.53, 0.96), 	 	 Color(),		  0), // esfera abajo-der
	Sphere(10.5, Point(-26, -30.3, 10),     Color(1, 1, 1), 	 	 Color(),		  2), // esfera abajo-der-atras
	Sphere(10.5, Point(0, 24.3, 0),        Color(0, 0, 0),       Color(10,10,10), 0) // esfera de luz
};

double totalShperes = sizeof(spheres)/sizeof(Sphere);

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

// calcular la intersección del rayo r con todas las esferas
// regresar true si hubo una intersección, falso de otro modo
// almacenar en t la distancia sobre el rayo en que sucede la interseccion
// almacenar en id el indice de spheres[] de la esfera cuya interseccion es mas cercana
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

// funcion para hacer un vector a partir de theta y phi dependiendo de lo que se quiere muestrear
Vector makeVec(double &theta, double &phi){
	double x = sin(theta) * cos(phi);
    double y = sin(theta) * sin(phi);
    double z = cos(theta);

	Vector vec = Vector (x, y, z);
	return vec;
}

// funcion para obtener los parametros del muestreo uniforme en hemisferio
void paramCosineHemisphere(double &theta, double &phi, double &prob) {
    double rand1 = dis(gen);
    double rand2 = dis(gen);
    theta = acos(sqrt(1.0 - rand1));
    phi = 2.0 * M_PI * rand2;
    prob = invPi * cos(theta);
}

// funcion para obtener los parametros del muestreo del angulo solido
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

// funcion para calcular probabilidad de muestreo del angulo solido
double probSolidAngle(double &cosTmax) {
	return 1.0 / (2 * M_PI * (1 - cosTmax));
}

// funcion para determinar BRDF
Color BRDF(Color f) {
	Color brdf = f * invPi;
	return brdf;
}

Vector sampleDir(Vector &n, double &theta, double &phi){
	Vector s, t;
	coordinateSystem(n, s, t);
	Vector wi = makeVec(theta, phi);
	Vector wiglob = globalizeCoord(wi, n, s, t);
	return wiglob;
}

int bounceDepth = 10;

// Calcula el valor de color para el rayo dado
Color shade(const Ray &r, int bounce, int cond) {
	double t;
	int id = 0;

	// determinar que esfera (id) y a que distancia (t) el rayo intersecta
	if (!intersect(r, t, id))
		return Color();	// el rayo no intersecto objeto, return Vector() == negro
  
	const Sphere &obj = spheres[id];  //esfera sobre la que se intersecto

	// if (obj.e.x > 0 && obj.e.y > 0 && obj.e.z > 0 && cond == 1)		// si la esfera intersectada es una fuente de luz y es el primer camino del rayo, regresar luz
	// 	return obj.e;

	if (++bounce > bounceDepth) return Color();

	// determinar coordenadas del punto de interseccion
	Point x = r.o + r.d*t;

	// determinar la dirección normal en el punto de interseccion
	Vector n = (x - obj.p).normalize();

	// determinar si un rayo choca con un objeto por dentro
	// si es el caso,  voltear  la normal (nv)
	Vector nv;
	if (n.dot(r.d * -1) > 0) {nv = n;}
	else {nv = n * -1;}

	// color del objeto intersectado
	Color baseColor = obj.c;
	
	// ruleta rusa
	double q = 0.1;
	double continueprob = 1.0 - q;
	if (dis(gen) < q) return Color();

	// determinar el color que se regresara

	// material especular 

	if (obj.mat == 1){
		Vector wr = r.d - n*2*(n.dot(r.d));  // direccion de refleccion especular ideal
		wr.normalize();
		return baseColor.mult(shade(Ray(x, wr), bounce, 1));
	}
	
	// material difuso

	if (obj.mat == 0) {

		// obtener una direccion aleatoria con muestreo de coseno hemisferico, wi
		double theta, phi, probMat;
		paramCosineHemisphere(theta, phi, probMat);
		Vector wi = sampleDir(n, theta, phi).normalize();
		double dotCos = n.dot(wi);
		Color bsdf = BRDF(baseColor);
		Ray newRay = Ray(x, wi);

		// calculo de iluminacion indirecta

		Color indirectLight = bsdf.mult(shade(newRay, bounce, 0)) * (fabs(dotCos)/(probMat*continueprob));

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
				double dotCos1 = n.dot(wl);
				probLight = probSolidAngle(cosTmax);
				double probLuzBDRF=0.5 * wl.z;
				directLight = directLight + Le.mult(bsdf * fabs(dotCos1) * (1.0 / (probLight*continueprob)));
			}
		}
		return obj.e * cond + directLight + indirectLight;
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
			return obj.e + baseColor.mult(shade(reflectionRay, bounce, 1)) * (fr*fabs(nv.dot(wr))/pr);
		}

		return obj.e + baseColor.mult(shade(refractionRay, bounce, 1))*(ft*fabs(nv.dot(wt))/pt);

		// return baseColor.mult(shade(refractionRay, bounce, 1)*(ft/pt) + shade(Ray(x, wr), bounce, 1)*(fr/pr));
	}
		

	else return Color();
}

int main(int argc, char *argv[]) {
	int w = 1024, h = 768; // image resolution

	int N = 32;  // numero de muestras

	// fija la posicion de la camara y la dirección en que mira
	Ray camera( Point(0, 11.2, 214), Vector(0, -0.042612, -1).normalize() );

	// parametros de la camara
	Vector cx = Vector( w * 0.5095 / h, 0., 0.); 
	Vector cy = (cx % camera.d).normalize() * 0.5095;
  
	// auxiliar para valor de pixel y matriz para almacenar la imagen
	Color *pixelColors = new Color[w * h];

	// lista de distancias, se realizó una corrida inicial para obtener todas las distancias en la escena
	vector<double> distList;

	// PROYECTO 1
	// usar openmp para paralelizar el ciclo: cada hilo computara un renglon (ciclo interior),
	// omp_set_num_threads(h);
	#pragma omp parallel
	#pragma omp for schedule(dynamic, 1)

	for(int y = 0; y < h; y++) 
	{ 
		// recorre todos los pixeles de la imagen
		fprintf(stderr,"\r%5.2f%%",100.*y/(h-1));

		for(int x = 0; x < w; x++ ) {
			for (int n=0; n<N; n++){
				int idx = (h - y - 1) * w + x; // index en 1D para una imagen 2D x,y son invertidos
				Color pixelValue = Color(); // pixelValue en negro por ahora
				// para el pixel actual, computar la dirección que un rayo debe tener
				Vector cameraRayDir = cx * ( double(x)/w - .5) + cy * ( double(y)/h - .5) + camera.d;

				// computar el color del pixel para el punto que intersectó el rayo desde la camara
				pixelValue = pixelValue + shade( Ray(camera.o, cameraRayDir.normalize()), 0, 1) * (1.0/N);

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