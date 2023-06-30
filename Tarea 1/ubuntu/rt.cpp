// rt: un lanzador de rayos minimalista
// g++ -O3 -fopenmp rt.cpp -o rt

#include <math.h>
#include <stdlib.h>
#include <stdio.h>  
#include <omp.h>
#include <vector>
#include <algorithm> // std::min_element
#include <iterator>  // std::begin, std::end
#include <iostream>
using namespace std;

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
	Vector operator%(Vector&b){return Vector(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);}
	
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
	Point o;
	Vector d; // origen y direcccion del rayo
	Ray(Point o_, Vector d_) : o(o_), d(d_) {} // constructor
};

class Sphere 
{
public:
	double r;	// radio de la esfera
	Point p;	// posicion
	Color c;	// color  

	Sphere(double r_, Point p_, Color c_): r(r_), p(p_), c(c_) {}
  
	// PROYECTO 1
	// determina si el rayo intersecta a esta esfera
	double intersect(const Ray &ray) const {
		// regresar distancia si hay intersección
		// regresar 0.0 si no hay interseccion
		Vector op = ray.o - p;
		Vector d = ray.d;
		double b = (op.dot(d));
		double ce = op.dot(op) - r*r;
		double disc = b*b - ce;
		if (disc < 0) return 0.0;
		else {
			double t0 = -b + sqrt(disc);
			double t1 = -b - sqrt(disc);
			double t = (t0 < t1) ? t0 : t1;
			if (t < 0) return 0;
			else return t;
		}
	}
};

Sphere spheres[] = {
	//Escena: radio, posicion, color 
	Sphere(1e5,  Point(-1e5 - 49, 0, 0),   Color(.75, .25, .25)), // pared izq
	Sphere(1e5,  Point(1e5 + 49, 0, 0),    Color(.25, .25, .75)), // pared der
	Sphere(1e5,  Point(0, 0, -1e5 - 81.6), Color(.75, .75, .75)), // pared detras
	Sphere(1e5,  Point(0, -1e5 - 40.8, 0), Color(.75, .75, .75)), // suelo
	Sphere(1e5,  Point(0, 1e5 + 40.8, 0),  Color(.75, .75, .75)), // techo
	Sphere(16.5, Point(-23, -24.3, -34.6), Color(.999, .999, .999)), // esfera abajo-izq
	Sphere(16.5, Point(23, -24.3, -3.6),   Color(.999, .999, .999) ), // esfera abajo-der
	Sphere(10.5, Point(0, 24.3, 0),        Color(1, 1, 1)) // esfera arriba
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

// PROYECTO 1
// calcular la intersección del rayo r con todas las esferas
// regresar true si hubo una intersección, falso de otro modo
// almacenar en t la distancia sobre el rayo en que sucede la interseccion
// almacenar en id el indice de spheres[] de la esfera cuya interseccion es mas cercana
inline bool intersect(const Ray &r, double &t, int &id) {
	double n = sizeof(spheres)/sizeof(Sphere);
	double dist;
	double thresh = t = 1000;

	for (int i=0;i < n;i++) {
		dist = spheres[i].intersect(r);
		if (dist && dist < t) {
			t = dist;
			id = i;
		}
	}
	if (t < thresh) return true;
	else return false;
}

// Regresar la distancia para el rayo dado
double giveDist(const Ray &r) {
	double t;
	int id = 0;
	
	// determinar que esfera (id) y a que distancia (t) el rayo intersecta
	if (!intersect(r, t, id))
		return t;	// el rayo no intersecto objeto, return distancia == 0
	
	return t;
}

// Calcula el valor de color para el rayo dado
Color shade(const Ray &r) {
	double t;
	int id = 0;
	// determinar que esfera (id) y a que distancia (t) el rayo intersecta
	if (!intersect(r, t, id))
		return Color();	// el rayo no intersecto objeto, return Vector() == negro
  
	const Sphere &obj = spheres[id];
	
	// PROYECTO 1
	// determinar coordenadas del punto de interseccion
	Point x = r.o + r.d*t;

	// determinar la dirección normal en el punto de interseccion
	Vector n = (x - obj.p).normalize();

	// determinar el color en función de la distancia
	double tNew = t / 304.111;      					// valor de distancia donde el rayo intersecta dividido entre la distancia máxima

	Color BWcol = Color (tNew, tNew, tNew);

	// determinar el color que se regresara
	// Color colorValue = obj.c;						// uncomment out para obtener el color de la esfera intersectada
	Color colorValue = obj.c + n;				// uncomment out para obtener la normal en el punto intersectado
	// Color colorValue = BWcol;					// uncomment out para obtener el color en función de la distancia

	return colorValue;
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
			
			// para el pixel actual, obtener la distancia entre el rayo y la intersección con la escena, necesario para obtener lista de distancias en la primera corrida
			// double dist = giveDist(Ray(camera.o, cameraRayDir.normalize()));		// uncomment out si se requiere obtener nueva lista de distancias

			// agregar la distancia obtenida a la lista de distancias
			// distList.push_back(dist);		// uncomment out si se requiere obtener nueva lista de distancias

			// computar el color del pixel para el punto que intersectó el rayo desde la camara
			pixelValue = shade( Ray(camera.o, cameraRayDir.normalize()) );

			// limitar los tres valores de color del pixel a [0,1]
			pixelColors[idx] = Color(clamp(pixelValue.x), clamp(pixelValue.y), clamp(pixelValue.z));
		}
	}

	// Impresión de distancia mínima		// uncomment out si se requiere obtener nueva lista de distancias
	// double minDist = *min_element(distList.begin(), distList.end());
	// std::cout<<"\nMin distance: " << minDist << std::endl;

	// // Impresión de distancia máxima		// uncomment out si se requiere obtener nueva lista de distancias
	// double maxDist = *max_element(distList.begin(), distList.end());
	// std::cout<<"Max distance: " << maxDist << std::endl;

	fprintf(stderr,"\n");

	// PROYECTO 1
	// Investigar formato ppm
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