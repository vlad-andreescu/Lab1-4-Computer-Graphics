#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <random>
#include <omp.h>
#include <fstream>
#include <sstream>
#include <map>
#include <string>
#include <algorithm>

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

	bool intersect(const Ray& ray, Vector& P, double &t, Vector& N) const {

        Vector OC = ray.O - C;
        double dot_u_OC = dot(ray.u, OC);
        double delta = dot_u_OC * dot_u_OC - (OC.norm2() - R * R); 

        if (delta < 0) return false;

        double sqrt_delta = sqrt(delta);
        double t1 = -dot_u_OC - sqrt_delta;
        double t2 = -dot_u_OC + sqrt_delta;

        if      (t1 > 0) t = t1;
        else if (t2 > 0) t = t2;
        else             return false;
        
        P = ray.O + t * ray.u;
        N = (P - C) / R;
        return true;
    }

	double R;
	Vector C;
};


// ============================================================
//  Mesh support (labs 3 and 4)
// ============================================================

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1,
	                int ni = -1, int nj = -1, int nk = -1,
	                int uvi = -1, int uvj = -1, int uvk = -1, int group = -1) {
		vtx[0] = vtxi; vtx[1] = vtxj; vtx[2] = vtxk;
		uv[0]  = uvi;  uv[1]  = uvj;  uv[2]  = uvk;
		n[0]   = ni;   n[1]   = nj;   n[2]   = nk;
		this->group = group;
	};
	int vtx[3];   // vertex indices
	int uv[3];    // uv-coordinate indices
	int n[3];     // normal indices
	int group;
};


class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false)
	    : ::Object(albedo, mirror, transparent) {
		bmin = Vector( 1e18,  1e18,  1e18);
		bmax = Vector(-1e18, -1e18, -1e18);
	};

	// scale then translate every vertex
	void scale_translate(double s, const Vector& t) {
		for (int i = 0; i < (int)vertices.size(); i++)
			vertices[i] = vertices[i] * s + t;

		// rebuild bounding box
		bmin = Vector( 1e18,  1e18,  1e18);
		bmax = Vector(-1e18, -1e18, -1e18);
		for (int i = 0; i < (int)vertices.size(); i++)
			for (int k = 0; k < 3; k++) {
				bmin[k] = std::min(bmin[k], vertices[i][k]);
				bmax[k] = std::max(bmax[k], vertices[i][k]);
			}
	}

	// read an .obj file
	void readOBJ(const char* obj) {
		std::ifstream f(obj);
		if (!f) return;

		std::map<std::string, int> mtls;
		int curGroup = -1, maxGroup = -1;

		auto resolveIdx = [](int i, int size) {
			return i < 0 ? size + i : i - 1;
		};
		auto setFaceVerts = [&](TriangleIndices& t, int i0, int i1, int i2) {
			t.vtx[0] = resolveIdx(i0, vertices.size());
			t.vtx[1] = resolveIdx(i1, vertices.size());
			t.vtx[2] = resolveIdx(i2, vertices.size());
		};
		auto setFaceUVs = [&](TriangleIndices& t, int j0, int j1, int j2) {
			t.uv[0] = resolveIdx(j0, uvs.size());
			t.uv[1] = resolveIdx(j1, uvs.size());
			t.uv[2] = resolveIdx(j2, uvs.size());
		};
		auto setFaceNormals = [&](TriangleIndices& t, int k0, int k1, int k2) {
			t.n[0] = resolveIdx(k0, normals.size());
			t.n[1] = resolveIdx(k1, normals.size());
			t.n[2] = resolveIdx(k2, normals.size());
		};

		std::string line;
		while (std::getline(f, line)) {
			line.erase(line.find_last_not_of(" \r\t\n") + 1);
			if (line.empty()) continue;
			const char* s = line.c_str();

			if (line.rfind("usemtl ", 0) == 0) {
				std::string matname = line.substr(7);
				auto result = mtls.emplace(matname, maxGroup + 1);
				curGroup = result.second ? ++maxGroup : result.first->second;
			} else if (line.rfind("vn ", 0) == 0) {
				Vector v;
				sscanf(s, "vn %lf %lf %lf", &v[0], &v[1], &v[2]);
				normals.push_back(v);
			} else if (line.rfind("vt ", 0) == 0) {
				Vector v;
				sscanf(s, "vt %lf %lf", &v[0], &v[1]);
				uvs.push_back(v);
			} else if (line.rfind("v ", 0) == 0) {
				Vector pos, col;
				if (sscanf(s, "v %lf %lf %lf %lf %lf %lf",
				           &pos[0], &pos[1], &pos[2],
				           &col[0], &col[1], &col[2]) == 6) {
					for (int i = 0; i < 3; i++)
						col[i] = std::min(1.0, std::max(0.0, col[i]));
					vertexcolors.push_back(col);
				} else {
					sscanf(s, "v %lf %lf %lf", &pos[0], &pos[1], &pos[2]);
				}
				vertices.push_back(pos);
			} else if (line[0] == 'f') {
				int i[4], j[4], k[4], offset, nn;
				const char* cur = s + 1;
				TriangleIndices t;
				t.group = curGroup;

				if ((nn = sscanf(cur, "%d/%d/%d %d/%d/%d %d/%d/%d%n",
				                 &i[0],&j[0],&k[0], &i[1],&j[1],&k[1], &i[2],&j[2],&k[2], &offset)) == 9) {
					setFaceVerts(t,i[0],i[1],i[2]); setFaceUVs(t,j[0],j[1],j[2]); setFaceNormals(t,k[0],k[1],k[2]);
				} else if ((nn = sscanf(cur, "%d/%d %d/%d %d/%d%n",
				                       &i[0],&j[0], &i[1],&j[1], &i[2],&j[2], &offset)) == 6) {
					setFaceVerts(t,i[0],i[1],i[2]); setFaceUVs(t,j[0],j[1],j[2]);
				} else if ((nn = sscanf(cur, "%d//%d %d//%d %d//%d%n",
				                       &i[0],&k[0], &i[1],&k[1], &i[2],&k[2], &offset)) == 6) {
					setFaceVerts(t,i[0],i[1],i[2]); setFaceNormals(t,k[0],k[1],k[2]);
				} else if ((nn = sscanf(cur, "%d %d %d%n",
				                       &i[0],&i[1],&i[2], &offset)) == 3) {
					setFaceVerts(t,i[0],i[1],i[2]);
				} else continue;

				indices.push_back(t);
				cur += offset;

				// fan-triangulate polygons with > 3 vertices
				while (*cur && *cur != '\n') {
					TriangleIndices t2; t2.group = curGroup;
					if ((nn = sscanf(cur, " %d/%d/%d%n", &i[3],&j[3],&k[3], &offset)) == 3) {
						setFaceVerts(t2,i[0],i[2],i[3]); setFaceUVs(t2,j[0],j[2],j[3]); setFaceNormals(t2,k[0],k[2],k[3]);
					} else if ((nn = sscanf(cur, " %d/%d%n", &i[3],&j[3], &offset)) == 2) {
						setFaceVerts(t2,i[0],i[2],i[3]); setFaceUVs(t2,j[0],j[2],j[3]);
					} else if ((nn = sscanf(cur, " %d//%d%n", &i[3],&k[3], &offset)) == 2) {
						setFaceVerts(t2,i[0],i[2],i[3]); setFaceNormals(t2,k[0],k[2],k[3]);
					} else if ((nn = sscanf(cur, " %d%n", &i[3], &offset)) == 1) {
						setFaceVerts(t2,i[0],i[2],i[3]);
					} else { cur++; continue; }
					indices.push_back(t2);
					cur += offset;
					i[2] = i[3]; j[2] = j[3]; k[2] = k[3];
				}
			}
		}

		// build bounding box (used in Lab 3 for the AABB test, slide 7)
		bmin = Vector( 1e18,  1e18,  1e18);
		bmax = Vector(-1e18, -1e18, -1e18);
		for (int i = 0; i < (int)vertices.size(); i++)
			for (int k = 0; k < 3; k++) {
				bmin[k] = std::min(bmin[k], vertices[i][k]);
				bmax[k] = std::max(bmax[k], vertices[i][k]);
			}
	}

	// ------------------------------------------------------------------
	// Lab 3 - Step 1: Ray vs. axis-aligned bounding box  (slide 7)
	//
	// For each axis k, the two slab planes give:
	//     t_k = (B_k - O_k) / u_k          (slide 7 formula)
	// We compute t_entry = max over axes of min(t0,t1)
	//            t_exit  = min over axes of max(t0,t1)
	// The ray hits the box iff t_entry <= t_exit  AND  t_exit > 0.
	// ------------------------------------------------------------------
	bool intersectBBox(const Ray& ray) const {
		double t_entry = -1e18;
		double t_exit  =  1e18;

		for (int k = 0; k < 3; k++) {
			// slide 7: t^x = (B^x - O^x) / u^x   (same for y and z)
			double t0 = (bmin[k] - ray.O[k]) / ray.u[k];
			double t1 = (bmax[k] - ray.O[k]) / ray.u[k];

			if (t0 > t1) std::swap(t0, t1);   // ensure t0 <= t1

			t_entry = std::max(t_entry, t0);   // latest entry across axes
			t_exit  = std::min(t_exit,  t1);   // earliest exit across axes

			if (t_exit < t_entry) return false; // intervals don't overlap -> miss
		}
		return t_exit > 0; // at least part of intersection is in front of ray
	}

	// ------------------------------------------------------------------
	// Lab 3 - Step 2: Ray-mesh intersection
	//   (a) fast-reject with the bounding box     (slide 7)
	//   (b) Moller-Trumbore on every triangle     (slides 5 and 36)
	//
	// Moller-Trumbore formulas (slide 5 / lab recap slide 36):
	//   e1 = B - A,   e2 = C - A,   N = e1 x e2
	//   beta  =  < e2 , (A-O) x u > / < u , N >
	//   gamma = -< e1 , (A-O) x u > / < u , N >
	//   alpha = 1 - beta - gamma
	//   t     =  < A-O , N >        / < u , N >
	//
	// Valid intersection: alpha, beta, gamma >= 0  and  t > 0.
	// ------------------------------------------------------------------
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {

		// ---- (a) bounding-box early-out (slide 7) ----
		if (!intersectBBox(ray)) return false;

		double closest_t = 1e18;
		bool   has_isect = false;

		for (int tri = 0; tri < (int)indices.size(); tri++) {

			// Vertices A, B, C  (slide 5 notation)
			const Vector& A = vertices[indices[tri].vtx[0]];
			const Vector& B = vertices[indices[tri].vtx[1]];
			const Vector& C = vertices[indices[tri].vtx[2]];

			// e1 = B - A,  e2 = C - A   (slide 5)
			Vector e1 = B - A;
			Vector e2 = C - A;

			// N = e1 x e2   (slide 5)
			Vector N_tri = cross(e1, e2);

			// denominator = < u , N >   (slide 5)
			double denom = dot(ray.u, N_tri);

			// ray parallel to triangle plane -> skip
			if (std::abs(denom) < 1e-12) continue;

			// A - O  (used in all three formulas)
			Vector AmO = A - ray.O;

			// (A-O) x u   (shared sub-expression for beta and gamma)
			Vector AmO_cross_u = cross(AmO, ray.u);

			// beta  =  < e2 , (A-O) x u > / < u , N >   (slide 5)
			double beta  =  dot(e2, AmO_cross_u) / denom;

			// gamma = -< e1 , (A-O) x u > / < u , N >   (slide 5)
			double gamma = -dot(e1, AmO_cross_u) / denom;

			// alpha = 1 - beta - gamma   (slide 5)
			double alpha = 1.0 - beta - gamma;

			// t = < A-O , N > / < u , N >   (slide 5)
			double t_cand = dot(AmO, N_tri) / denom;

			// valid hit: all barycentric coords >= 0, t in front, closer than current best
			if (alpha >= 0.0 && beta >= 0.0 && gamma >= 0.0 &&
			    t_cand > 1e-6 && t_cand < closest_t)
			{
				closest_t = t_cand;
				has_isect = true;
				t = t_cand;
				P = ray.O + t_cand * ray.u;

				// Phong normal interpolation (slide 15):
				//   N(P) = alpha*N_A + beta*N_B + gamma*N_C
				if (!normals.empty() &&
				    indices[tri].n[0] >= 0 &&
				    indices[tri].n[1] >= 0 &&
				    indices[tri].n[2] >= 0)
				{
					N = alpha * normals[indices[tri].n[0]]
					  + beta  * normals[indices[tri].n[1]]
					  + gamma * normals[indices[tri].n[2]];
				} else {
					N = N_tri;   // flat shading fallback
				}
				N.normalize();
			}
		}

		return has_isect;
	}


	std::vector<TriangleIndices> indices;
	std::vector<Vector>          vertices;
	std::vector<Vector>          normals;
	std::vector<Vector>          uvs;
	std::vector<Vector>          vertexcolors;

	// axis-aligned bounding box corners (lab 3)
	Vector bmin, bmax;
};


// ============================================================
//  Helpers / Scene
// ============================================================

Vector random_cos(const Vector& N) {
    double r1 = uniform(engine[omp_get_thread_num()]);
	double r2 = uniform(engine[omp_get_thread_num()]);
    double lx = cos(2.0 * M_PI * r1) * sqrt(1.0 - r2);
    double ly = sin(2.0 * M_PI * r1) * sqrt(1.0 - r2);
    double lz = sqrt(r2);
    Vector T1;
    if      (fabs(N[0]) <= fabs(N[1]) && fabs(N[0]) <= fabs(N[2])) T1 = Vector(0.0,  -N[2], N[1]);
    else if (fabs(N[1]) <= fabs(N[0]) && fabs(N[1]) <= fabs(N[2])) T1 = Vector(-N[2],  0.0, N[0]);
    else                                                              T1 = Vector(-N[1],  N[0], 0.0);
    T1.normalize();
    Vector T2 = cross(N, T1);
    return lx * T1 + ly * T2 + lz * N;
}


class Scene {
public:
	Scene() {};
	void addObject(const Object* obj) { objects.push_back(obj); }

	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N, int& object_id) const {
        double closest_t = 1e99;
        bool   has_isect = false;
        for (int i = 0; i < (int)objects.size(); i++) {
            Vector cur_P, cur_N; double cur_t;
            if (objects[i]->intersect(ray, cur_P, cur_t, cur_N) && cur_t < closest_t && cur_t > 0) {
                closest_t = cur_t; object_id = i; P = cur_P; t = cur_t; N = cur_N; has_isect = true;
            }
        }
        return has_isect;
    }

	Vector getColor(const Ray& ray, int depth) {
		if (depth >= max_light_bounce) return Vector(0,0,0);

		Vector P, N; double t; int object_id;
		if (!intersect(ray, P, t, N, object_id)) return Vector(0,0,0);

		// mirror
		if (objects[object_id]->mirror) {
			Vector R = ray.u - 2.0 * dot(ray.u, N) * N; R.normalize();
			return getColor(Ray(P + N * 1e-4, R), depth + 1);
		}

		// transparent / refraction
		if (objects[object_id]->transparent) {
			double n1 = 1.0, n2 = 1.5;
			Vector Nref = N;
			if (dot(ray.u, N) > 0) { n1 = 1.5; n2 = 1.0; Nref = Vector(-N[0],-N[1],-N[2]); }
			double ratio  = n1 / n2;
			double cosI   = dot(ray.u, Nref);
			double sin2T  = ratio * ratio * (1.0 - cosI * cosI);
			if (sin2T > 1.0) {
				Vector R = ray.u - 2.0 * dot(ray.u, Nref) * Nref; R.normalize();
				return getColor(Ray(P + Nref * 1e-4, R), depth + 1);
			}
			Vector T = ratio * ray.u - (ratio * cosI + sqrt(1.0 - sin2T)) * Nref; T.normalize();
			return getColor(Ray(P - Nref * 1e-4, T), depth + 1);
		}

		// direct lighting + shadow test
		Vector L_dir = light_position - P;
		double dist2 = L_dir.norm2();
		double dist  = sqrt(dist2);
		Vector L     = L_dir / dist;

		Vector shP, shN; double sh_t; int sh_id;
		bool in_shadow = intersect(Ray(P + N * 1e-4, L), shP, sh_t, shN, sh_id);

		Vector direct(0,0,0);
		if (!in_shadow || sh_t >= dist) {
			double ndotl = std::max(0.0, dot(N, L));
			direct = (objects[object_id]->albedo / M_PI) * (light_intensity / (4.0 * M_PI * dist2)) * ndotl;
		}

		// indirect lighting
		Vector wi = random_cos(N);
		Vector Li = getColor(Ray(P + N * 1e-4, wi), depth + 1);
		Vector alb = objects[object_id]->albedo;
		Vector indirect(alb[0]*Li[0], alb[1]*Li[1], alb[2]*Li[2]);

		return direct + indirect;
    }

	std::vector<const Object*> objects;
	Vector camera_center, light_position;
	double fov, gamma, light_intensity;
	int    max_light_bounce;
};



int main() {
	int W = 512, H = 512; 
	for (int i = 0; i < 32; i++) engine[i].seed(i);

	// Cornell-box walls
	Sphere wall_left  (Vector(-1000, 0,    0),  940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right (Vector( 1000, 0,    0),  940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front (Vector(0,     0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0,     0,  1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling    (Vector(0,  1000,    0),  940, Vector(0.3, 0.5, 0.3));
	Sphere floor_s    (Vector(0, -1000,    0),  990, Vector(0.6, 0.5, 0.7));

	// Triangle mesh – adjust path, scale, and translation to match your cat.obj
	TriangleMesh mesh(Vector(0.8, 0.8, 0.8));
	mesh.readOBJ("cat.obj");
	mesh.scale_translate(0.6, Vector(0, -10, 0));

	Scene scene;
	scene.camera_center   = Vector(0, 0, 55);
	scene.light_position  = Vector(-10, 20, 40);
	scene.light_intensity = 1e7;
	scene.fov             = 60. * M_PI / 180.;
	scene.gamma           = 2.2;
	scene.max_light_bounce = 5; 

	scene.addObject(&wall_left);
	scene.addObject(&wall_right);
	scene.addObject(&wall_front);
	scene.addObject(&wall_behind);
	scene.addObject(&ceiling);
	scene.addObject(&floor_s);
	scene.addObject(&mesh);

	std::vector<unsigned char> image(W * H * 3, 0);
	double d = (W / 2.0) / std::tan(scene.fov / 2.0);

#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color(0,0,0);
			int N_SAMPLES = 32; 
			for (int k = 0; k < N_SAMPLES; k++) {
				double r1 = uniform(engine[omp_get_thread_num()]);
				double r2 = uniform(engine[omp_get_thread_num()]);
				double dx = 0.5 * sqrt(-2.0 * log(r1)) * cos(2.0 * M_PI * r2);
				double dy = 0.5 * sqrt(-2.0 * log(r1)) * sin(2.0 * M_PI * r2);
				double x  = j - W / 2.0 + 0.5 + dx;
				double y  = H / 2.0 - i - 0.5 + dy;
				Vector ray_dir(x, y, -d);
				ray_dir.normalize();
				color = color + scene.getColor(Ray(scene.camera_center, ray_dir), 0);
			}
			color = color / N_SAMPLES;

			double c0 = 255. * std::pow(color[0] / 255., 1. / 2.2);
			double c1 = 255. * std::pow(color[1] / 255., 1. / 2.2);
			double c2 = 255. * std::pow(color[2] / 255., 1. / 2.2);

			image[(i * W + j) * 3 + 0] = (unsigned char)(c0 > 255. ? 255. : (c0 < 0. ? 0. : c0));
			image[(i * W + j) * 3 + 1] = (unsigned char)(c1 > 255. ? 255. : (c1 < 0. ? 0. : c1));
			image[(i * W + j) * 3 + 2] = (unsigned char)(c2 > 255. ? 255. : (c2 < 0. ? 0. : c2));
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);
	return 0;
}