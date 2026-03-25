#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <random>

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
			
			if (objects[object_id]->transparent) { // optional

				// return getColor in the refraction direction, with recursion_depth+1 (recursively)
			} // else

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

            if (in_shadow && shadow_t < dist_to_light) {
                return Vector(0, 0, 0); 
            }
            double dot_product = dot(N, L);
            double n_dot_l = dot_product > 0.0 ? dot_product : 0.0; 
            double intensity_factor = light_intensity / (4.0*M_PI*dist_to_light2);
            Vector albedo = objects[object_id]->albedo;
            
            return (albedo / M_PI) * intensity_factor * n_dot_l;
        }

        return Vector(0, 0, 0);
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

	Sphere center_sphere(Vector(0, 0, 0), 10., Vector(0.8, 0.8, 0.8));
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

	scene.addObject(&center_sphere);

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
			Vector color;

			// TODO (lab 1) : correct ray_direction so that it goes through each pixel (j, i)
			double x = j - W / 2.0 + 0.5;
            double y = H / 2.0 - i - 0.5;
            double z = -d;


			Vector ray_direction(x, y, z);
            ray_direction.normalize();

            Ray ray(scene.camera_center, ray_direction);
            color = scene.getColor(ray, 0);

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
