#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <random>
#include <omp.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifndef M_PI
#define M_PI 3.14159265358979323856
#endif

static std::default_random_engine engine[32];
static std::uniform_real_distribution<double> uniform(0, 1);

double sqr(double x) { return x * x; };

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}
	double norm2() const {
		return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
	}
	double norm() const {
		return sqrt(norm2());
	}
	void normalize() {
		double n = norm();
		data[0] /= n;
		data[1] /= n;
		data[2] /= n;
	}
	double operator[](int i) const { return data[i]; };
	double& operator[](int i) { return data[i]; };
	double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
	return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

class Ray {
public:
	Ray(const Vector& origin, const Vector& unit_direction) : O(origin), u(unit_direction) {};
	Vector O, u;
};

class Object {
public:
	Object(const Vector& albedo, bool mirror = false, bool transparent = false) : albedo(albedo), mirror(mirror), transparent(transparent) {};

	virtual bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const = 0;

	Vector albedo;
	bool mirror, transparent;
};

class Sphere : public Object {
public:
	Sphere(const Vector& center, double radius, const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent), C(center), R(radius) {};

	// returns true iif there is an intersection between the ray and the sphere
	// if there is an intersection, also computes the point of intersection P, 
	// t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
	// and the unit normal N
	bool intersect(const Ray& ray, Vector& P, double &t, Vector& N) const {

        Vector OC = ray.O - C;
        double dot_u_OC = dot(ray.u, OC);
        double delta = dot_u_OC * dot_u_OC - (OC.norm2() - R * R); 

        if (delta < 0) {
            return false;
        }

        double sqrt_delta=sqrt(delta);
        
        double t1 = -dot_u_OC - sqrt_delta;
        double t2 = -dot_u_OC + sqrt_delta;

        if (t1 > 0) {
            t = t1; 
        } else if (t2 > 0) {
            t = t2; 
        } else {
            return false;
        }
        
        P = ray.O + t * ray.u;
        N = (P - C) / R;

        return true;
    }

	double R;
	Vector C;
};


// I will provide you with an obj mesh loader (labs 3 and 4)
class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent) {};

	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {
		// TODO (labs 3 and 4)
		return false;
	}
};

Vector random_cos(const Vector& N) {
    double r1 = uniform(engine[omp_get_thread_num()]);
	double r2 = uniform(engine[omp_get_thread_num()]);
    double lx = cos(2.0 * M_PI * r1) * sqrt(1.0 - r2);
    double ly = sin(2.0 * M_PI * r1) * sqrt(1.0 - r2);
    double lz = sqrt(r2);
    Vector T1;
    if (fabs(N[0]) <= fabs(N[1]) && fabs(N[0]) <= fabs(N[2]))
        T1 = Vector(0.0, -N[2], N[1]);
    else if (fabs(N[1]) <= fabs(N[0]) && fabs(N[1]) <= fabs(N[2]))
        T1 = Vector(-N[2], 0.0, N[0]);
    else
        T1 = Vector(-N[1], N[0], 0.0);
    T1.normalize();
    Vector T2 = cross(N, T1);
    return lx * T1 + ly * T2 + lz * N;
}


class Scene {
public:
	Scene() {};
	void addObject(const Object* obj) {
		objects.push_back(obj);
	}

	// returns true iif there is an intersection between the ray and any object in the scene
    // if there is an intersection, also computes the point of the *nearest* intersection P, 
    // t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
    // and the unit normal N. 
	// Also returns the index of the object within the std::vector objects in object_id
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N, int &object_id) const  {

		// TODO (lab 1): iterate through the objects and check the intersections with all of them, 
		// and keep the closest intersection, i.e., the one if smallest positive value of t
        double closest_t = 1e99;
        bool has_intersection = false;

        for (int i=0; i<objects.size(); i++) {
            Vector current_P, current_N;
            double current_t;
            
            if (objects[i]->intersect(ray, current_P, current_t, current_N)) {
                if (current_t<closest_t && current_t>0) {
                    closest_t=current_t;
                    object_id=i;
                    P=current_P;
                    t=current_t;
                    N=current_N;
                    has_intersection=true;
                }
            }
        }
        return has_intersection;
    }


	// return the radiance (color) along ray
	Vector getColor(const Ray& ray, int recursion_depth) {

		if (recursion_depth >= max_light_bounce) return Vector(0, 0, 0);

		// TODO (lab 1) : if intersect with ray, use the returned information to compute the color ; otherwise black 
		// in lab 1, the color only includes direct lighting with shadows

		Vector P, N;
		double t;
		int object_id;
		if (intersect(ray, P, t, N, object_id)) {

			if (objects[object_id]->mirror) {
                Vector I = ray.u;
                Vector R = I - N * 2.0 * dot(I, N);
                R.normalize();

                Vector reflected_origin = P + N * 1e-4;
                Ray reflected_ray(reflected_origin, R);

                return getColor(reflected_ray, recursion_depth + 1);
            }
			
			if (objects[object_id]->transparent) {
				double n1 = 1.0, n2 = 1.5;
				Vector Nrefract = N;

				if (dot(ray.u, N) > 0) {
					n1 = 1.5; n2 = 1.0;
					Nrefract = Vector(-N[0], -N[1], -N[2]);
				}

				double ratio = n1 / n2;
				double cosI = dot(ray.u, Nrefract);
				double sin2T = ratio * ratio * (1.0 - cosI * cosI);

				if (sin2T > 1.0) {
					Vector R = ray.u - 2.0 * dot(ray.u, Nrefract) * Nrefract;
					R.normalize();
					return getColor(Ray(P + Nrefract * 1e-4, R), recursion_depth + 1);
				}

				Vector T = ratio * ray.u - (ratio * cosI + sqrt(1.0 - sin2T)) * Nrefract;
				T.normalize();
				return getColor(Ray(P - Nrefract * 1e-4, T), recursion_depth + 1);
			}

			// test if there is a shadow by sending a new ray
			// if there is no shadow, compute the formula with dot products etc.
			Vector L_dir = light_position - P;
            double dist_to_light2 = L_dir.norm2();
            double dist_to_light = sqrt(dist_to_light2);
            Vector L = L_dir / dist_to_light;

            Vector shadow_origin = P + N * 1e-4; 
            Ray shadow_ray(shadow_origin, L);
            
            Vector shadow_P, shadow_N;
            double shadow_t;
            int shadow_object_id;
            bool in_shadow = intersect(shadow_ray, shadow_P, shadow_t, shadow_N, shadow_object_id);

            Vector direct(0, 0, 0);
			if (!in_shadow || shadow_t >= dist_to_light) {
				double dot_product = dot(N, L);
				double n_dot_l = dot_product > 0.0 ? dot_product : 0.0;
				double intensity_factor = light_intensity / (4.0*M_PI*dist_to_light2);
				direct = (objects[object_id]->albedo / M_PI) * intensity_factor * n_dot_l;
			}

			Vector indirect_dir = random_cos(N);
			Ray indirect_ray(P + N * 1e-4, indirect_dir);
			Vector Li = getColor(indirect_ray, recursion_depth + 1);
			Vector albedo = objects[object_id]->albedo;
			Vector indirect(albedo[0]*Li[0], albedo[1]*Li[1], albedo[2]*Li[2]);

			return direct + indirect;
        }
		
		return Vector(0,0,0);
    }

	std::vector<const Object*> objects;

	Vector camera_center, light_position;
	double fov, gamma, light_intensity;
	int max_light_bounce;
};


int main() {
	int W = 512;
	int H = 512;

	for (int i = 0; i<32; i++) {
		engine[i].seed(i);
	}

	Sphere sphere_diffuse(Vector(-20, 0, 0), 10., Vector(0.8, 0.8, 0.8));
	Sphere sphere_mirror(Vector(0, 0, 0), 10., Vector(0.8, 0.8, 0.8), true, false);
	Sphere sphere_transparent(Vector(20, 0, 0), 10., Vector(0.8, 0.8, 0.8), false, true);

	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

	Scene scene;
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10,20,40);
	scene.light_intensity = 3E7;
	scene.fov = 60 * M_PI / 180.;
	scene.gamma = 2.2;    // TODO (lab 1) : play with gamma ; typically, gamma = 2.2
	scene.max_light_bounce = 5;

	scene.addObject(&sphere_diffuse);
	scene.addObject(&sphere_mirror);
	scene.addObject(&sphere_transparent);

	scene.addObject(&wall_left);
	scene.addObject(&wall_right);
	scene.addObject(&wall_front);
	scene.addObject(&wall_behind);
	scene.addObject(&ceiling);
	scene.addObject(&floor);

	std::vector<unsigned char> image(W * H * 3, 0);

	double d = (W / 2.0) / std::tan(scene.fov / 2.0);

#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color(0, 0, 0);
			int N_SAMPLES = 32;
			for (int k = 0; k < N_SAMPLES; k++) {
				double r1 = uniform(engine[omp_get_thread_num()]);
				double r2 = uniform(engine[omp_get_thread_num()]);
				double dx = 0.5 * sqrt(-2.0 * log(r1)) * cos(2.0 * M_PI * r2);
				double dy = 0.5 * sqrt(-2.0 * log(r1)) * sin(2.0 * M_PI * r2);
				double x = j - W / 2.0 + 0.5 + dx;
				double y = H / 2.0 - i - 0.5 + dy;
				double z = -d;
				Vector ray_direction(x, y, z);
				ray_direction.normalize();
				Ray ray(scene.camera_center, ray_direction);
				color = color + scene.getColor(ray, 0);
			}
			color = color / N_SAMPLES;

			double c0 = 255. * std::pow(color[0] / 255., 1. / 2.2);
			double c1 = 255. * std::pow(color[1] / 255., 1. / 2.2);
			double c2 = 255. * std::pow(color[2] / 255., 1. / 2.2);

			// TODO (lab 2) : add Monte Carlo / averaging of random ray contributions here
			// TODO (lab 2) : add antialiasing by altering the ray_direction here
			// TODO (lab 2) : add depth of field effect by altering the ray origin (and direction) here


			image[(i * W + j) * 3 + 0] = c0 > 255. ? 255. : (c0 < 0. ? 0. : c0);
            image[(i * W + j) * 3 + 1] = c1 > 255. ? 255. : (c1 < 0. ? 0. : c1);
            image[(i * W + j) * 3 + 2] = c2 > 255. ? 255. : (c2 < 0. ? 0. : c2);
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}
