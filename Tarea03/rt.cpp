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

double Pi = M_PI;
double invPi = 1 / Pi;

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

	Sphere(double r_, Point p_, Color c_, Color e_): r(r_), p(p_), c(c_), e(e_){}
  
	// determina si el rayo intersecta a esta esfera
	double intersect(const Ray &ray) const {
		// regresar distancia si hay intersección
		// regresar 0.0 si no hay interseccion
		Vector op = p - ray.o;
		Vector d = ray.d;
		double t;
		double eps = 1e-4;
		double b = op.dot(d);
		double ce = op.dot(op) - r * r;
		double disc = b * b - ce;
		if (disc < 0) return 0.0;
		else disc = sqrt(disc);

		return (t = b - disc) > eps ? t : ((t = b + disc) > eps ? t : 0);
	}
};

Sphere spheres[] = {
	//Escena: radio, posicion, color, emision
	Sphere(1e5,  Point(-1e5 - 49, 0, 0),   Color(.75, .25, .25), Color()), // pared izq
	Sphere(1e5,  Point(1e5 + 49, 0, 0),    Color(.25, .25, .75), Color()), // pared der
	Sphere(1e5,  Point(0, 0, -1e5 - 81.6), Color(.25, .75, .25), Color()), // pared detras
	Sphere(1e5,  Point(0, -1e5 - 40.8, 0), Color(.25, .75, .75), Color()), // suelo
	Sphere(1e5,  Point(0, 1e5 + 40.8, 0),  Color(.75, .75, .25), Color()), // techo
	Sphere(16.5, Point(-23, -24.3, -34.6), Color(.2, .3, .4),	 Color()), // esfera abajo-izq
	Sphere(16.5, Point(23, -24.3, -3.6),   Color(.4, .3, .2), 	 Color()), // esfera abajo-der
	Sphere(10.5, Point(0, 24.3, 0),        Color(1, 1, 1),       Color(10,10,10)) // esfera de luz
};

// limita el valor de x a [0,1]
inline double clamp(const double x) { 
	if(x < 0.0)
		return 0.0;
	else if(x > 1.0)
		return 1.0;
	return x;
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
	double n = sizeof(spheres)/sizeof(Sphere);
	double dist;
	double thresh = t = 100000000000;

	for (int i=0;i < n;i++) {
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
		t = Vector(n.z * invLen, 0.0f, -n.x * invLen);
	} else {
		float invLen = 1.0f / std::sqrt(n.y * n.y + n.z * n.z);
		t = Vector(0.0f, n.z * invLen, -n.y * invLen);
	}
	s = (t % n);
}

// funcion para obtener una direccion global a partir de una local
Vector globalVec(Vector &locVec, Vector &n, Vector &s, Vector &t){
	Vector coordGlobal;
	Vector row1 = Vector (s.x, t.x, n.x);
	Vector row2 = Vector (s.y, t.y, n.y);
	Vector row3 = Vector (s.z, t.z, n.z);

	coordGlobal = Vector (row1.dot(locVec), row2.dot(locVec), row3.dot(locVec));
	return coordGlobal;
}

// funcion para obtener los parametros del muestreo uniforme en hemisferio
void hemisfParam(double &rand1, double &rand2, double &theta, double &phi, double &prob, double &x, double &y, double &z) {
    rand1 = dis(gen);
    rand2 = dis(gen);
    theta = acos(rand1);
    phi = 2*Pi*rand2;
    prob = 1 / (2*Pi);
    x = sin(theta) * cos(phi);
    y = sin(theta) * sin(phi);
    z = cos(theta);
}

// funcion para determinar la emision del punto x'
Color radiancia(Point &x, Vector &wi) {
	double t1;
	int id1 = 0;
	Ray newRay = Ray (x, wi);
	const Sphere light = spheres[7];
	Color Le;

	if (intersect(newRay, t1, id1)) {
		if (id1 == 7) {
			Le = light.e;
		}
		else Le = Color();
	}
	else Le = Color();
	return Le;
}

// funcion para determinar BRDF
Color BRDF(const Sphere &obj) {
	Color brdf = obj.c * invPi;
	return brdf;
}

// funcion de estimador Monte Carlo
Vector monteCarlo(int &N, Vector &n, Point &x, const Sphere &obj){
	Color sum = Color (0, 0, 0);
	Vector s, t;
	double rand1, rand2, theta, phi, prob, xnew, ynew, znew;
	coordinateSystem(n, s, t);

	for(int i = 0; i < N; i++){
		hemisfParam(rand1, rand2, theta, phi, prob, xnew, ynew, znew);
		Vector wi = Vector(xnew, ynew, znew);
		Vector wiglob = globalVec(wi, n, s, t);
		wiglob.normalize();
		Color Le = radiancia(x, wiglob);
		Color fr = BRDF(obj);
		double dotCos = n.dot(wiglob);
		Color L = Le.mult(fr) * (dotCos/prob);
		sum = sum + L;
	}

	return sum;
}

// Calcula el valor de color para el rayo dado
Color shade(const Ray &r) {
	double t;
	int id = 0;
	int N = 32;		//numero de muestras por pixel

	// determinar que esfera (id) y a que distancia (t) el rayo intersecta
	if (!intersect(r, t, id))
		return Color();	// el rayo no intersecto objeto, return Vector() == negro
  
	const Sphere &obj = spheres[id];  //esfera sobre la que se intersecto

	// determinar coordenadas del punto de interseccion
	Point x = r.o + r.d*t;

	// determinar la dirección normal en el punto de interseccion
	Vector n = (x - obj.p).normalize();

	// determinar el color que se regresara

	Color colorValue = monteCarlo(N, n, x, obj)*(1.0/N);

	return obj.e + colorValue;
}

int main(int argc, char *argv[]) {
	int w = 1024, h = 768; // image resolution

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
	omp_set_num_threads(h);
	#pragma omp parallel
	#pragma omp for

	for(int y = 0; y < h; y++) 
	{ 
		// recorre todos los pixeles de la imagen
		fprintf(stderr,"\r%5.2f%%",100.*y/(h-1));

		for(int x = 0; x < w; x++ ) {
			int idx = (h - y - 1) * w + x; // index en 1D para una imagen 2D x,y son invertidos
			Color pixelValue = Color(); // pixelValue en negro por ahora
			// para el pixel actual, computar la dirección que un rayo debe tener
			Vector cameraRayDir = cx * ( double(x)/w - .5) + cy * ( double(y)/h - .5) + camera.d;

			// computar el color del pixel para el punto que intersectó el rayo desde la camara
			pixelValue = shade( Ray(camera.o, cameraRayDir.normalize()) );

			// limitar los tres valores de color del pixel a [0,1]
			pixelColors[idx] = Color(clamp(pixelValue.x), clamp(pixelValue.y), clamp(pixelValue.z));
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
