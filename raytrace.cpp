#pragma once
#define NOMINMAX
#include <vector>
#include <cmath>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].
#include <Eigen/StdVector>
#include <Eigen_unsupported/Eigen/BVH>

#define EPSILON 0.000001

Box3d bounding_box(const Shape* object);

Scene::Scene() 
{ 
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    Vector4f sum(0, 0, 0, 0);
    int i = 0;
    Vector3f V[3];
    Vector3f N[3];
    for (auto v : mesh->triangles) // C++11 for loop
    {
        V[0] = mesh->vertices[v[0]].pnt;
        V[1] = mesh->vertices[v[1]].pnt;
        V[2] = mesh->vertices[v[2]].pnt;
        N[0] = mesh->vertices[v[0]].nrm;
        N[1] = mesh->vertices[v[1]].nrm;
        N[2] = mesh->vertices[v[2]].nrm;
        objects.push_back(new Triangle(V[0], V[1], V[2], N[0], N[1], N[2], mesh->mat));
    }
    //center = (sum / meshdata->vertices.size()).block<3, 1>(0, 0);
}

Quaternionf Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    Quaternionf q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitZ());
        else if (c == "q")  {
            q *= Quaternionf(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3f(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }
    return q;
}

////////////////////////////////////////////////////////////////////////
// Material: encapsulates surface properties
void Material::setTexture(const std::string path)
{
    int width, height, n;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* image = stbi_load(path.c_str(), &width, &height, &n, 0);
    stbi_image_free(image);
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        width = int(f[1]);
        height = int(f[2]); }

    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        camera = new Camera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
    }

    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        ambient = Vector3f(f[1], f[2], f[3]);
    }

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]); }

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(Vector3f(f[1], f[2], f[3])); }
   
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        objects.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat));
    }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        objects.push_back(new Box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat));
    }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        objects.push_back(new Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat));
    }

    else if (c == "capsule") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        objects.push_back(new Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat));
    }

    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        Matrix4f modelTr = translate(Vector3f(f[2],f[3],f[4]))
                          *scale(Vector3f(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);
	}


    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

Minimizer Scene::TraceRay(Ray& ray, KdBVH<float, 3, Shape*>& tree)
{
	ray.D.normalize();
	Minimizer minimizer(ray);
	float minDist = BVMinimize(tree, minimizer);

	return minimizer;
}

void Scene::TraceImage(Color* image, const int pass)
{
    KdBVH<float, 3, Shape*> Tree(objects.begin(), objects.end());
	float rx = camera->ry * (float(width) / float(height));
	Vector3f X = rx * camera->viewOrientation._transformVector(Vector3f::UnitX());
	Vector3f Y = camera->ry * camera->viewOrientation._transformVector(Vector3f::UnitY());
	Vector3f Z = -1 * camera->viewOrientation._transformVector(Vector3f::UnitZ());

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height;  y++) {

        fprintf(stderr, "Rendering %4d\r", y);
        for (int x=0;  x<width;  x++) {
			float dx = 2 * (float(x) + 0.5f) / width - 1;
			float dy = 2 * (float(y) + 0.5f) / height - 1;
			Ray ray(camera->eyePoint, dx * X + dy * Y + Z);
			
			Minimizer minimizer = TraceRay(ray, Tree);

			Color color = minimizer.minIntersection.object->shapeMaterial->Kd/5.0f + (minimizer.minIntersection.N).dot((Vector3f::UnitY()/2.0f + Vector3f::UnitX()/ 2.0f + Vector3f::UnitZ()).normalized()/ 2.0f) * minimizer.minIntersection.object->shapeMaterial->Kd;

            image[y*width + x] = color;
        }
    }
    fprintf(stderr, "\n");
}
//work in progress here
Color Scene::TracePath(Ray ray, KdBVH<float, 3, Shape*>& tree)
{
	Color C(0.0, 0.0, 0.0);
	Vector3f W(1.0, 1.0, 1.0);

	Minimizer minimizer = TraceRay(ray, tree);

	if (minimizer.minIntersection.t == INFINITY)
		return Color(0.0, 0.0, 0.0);

	Shape* shape = minimizer.minIntersection.object;
	if (shape->shapeMaterial->isLight())
		return Color(shape->shapeMaterial->Kd);
	
	Vector3f N = minimizer.minIntersection.N;
	while (myrandom(RNGen) < 0.8)
	{
		Vector3f w = shape->SampleBRDF(N);

		Minimizer Q = TraceRay(Ray(, w), tree);

		if (Q.minIntersection.t == INFINITY)
			break;

		N = Q.minIntersection.N;
		shape = Q.minIntersection.object;

		Vector3f f = fabsf(N.dot(w)) * shape->EvalBRDF();
		float p = shape->PdfBRDF(N, w) * 0.8;
		
		if (p < EPSILON)
			break;

		W *= f / p;

		if (shape->shapeMaterial->isLight)
			C += Color(shape->shapeMaterial->Kd.x * W.x, shape->shapeMaterial->Kd.y * W.y, shape->shapeMaterial->Kd.z * W.z);

	}
}

float Shape::PdfBRDF(const Vector3f& N, const Vector3f& w)
{
	return fabsf(w.dot(N)) / PI;
}

Vector3f Shape::EvalBRDF()
{
	return shapeMaterial->Kd / PI;
}

Vector3f Shape::SampleBRDF(const Vector3f& N)
{
	float xi_1 = myrandom(RNGen);
	float xi_2 = myrandom(RNGen);

	return SampleLobe(N, xi_1, xi_2);
}

Vector3f Shape::SampleLobe(const Vector3f& N, float c, float phi)
{
	float s = sqrtf(1 - c * c);
	Vector3f K(s * cosf(phi), s * sinf(phi), c);
	Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), N);

	return q._transformVector(K);
}

Box3d bounding_box(const Shape * object)
{
    return object->bbox();
}

Vector3f Ray::eval(float t)
{
	return Q + t * D;
}

Sphere::Sphere(Vector3f C, float r, Material* material): C(C), r(r)
{
    shapeMaterial = material;
}

bool Sphere::intersect(Ray ray, Intersecton & intersection)
{
    Vector3f Q = ray.Q - C;
	
    float QD = Q.dot(ray.D);
    float QQ = Q.dot(Q);
	
    float d = 4 * QD * QD - 4 * ray.D.dot(ray.D) * (QQ - r * r);
	
    if (d < 0)
        return false;
	
    float t1 = - QD + sqrt(QD * QD - QQ + r * r);
    float t2 = - QD - sqrt(QD * QD - QQ + r * r);
	
    if (t1 < 0 && t2 < 0)
        return false;
    else if (t1 < 0)
        intersection.t = t2;
    else if (t2 < 0)
        intersection.t = t1;
    else
        intersection.t = fminf(t1, t2);
	
    intersection.P = ray.eval(intersection.t);
    intersection.N = intersection.P - C;
    intersection.N.normalize();
    intersection.object = this;
	
	return true;
}

Box3d Sphere::bbox() const
{
    return Box3d(C - Vector3f(r, r, r), C + Vector3f(r, r, r));
}

Triangle::Triangle(Vector3f V1, Vector3f V2, Vector3f V3, Vector3f N1, Vector3f N2, Vector3f N3, Material* material)
{
    V[0] = V1; V[1] = V2; V[2] = V3;
    N[0] = N1; N[1] = N2; N[2] = N3;
    shapeMaterial = material;
}

bool Triangle::intersect(Ray ray, Intersecton & intersection)
{
    Vector3f E1 = V[1] - V[0];
    Vector3f E2 = V[2] - V[0];
    Vector3f p = ray.D.cross(E2);
	
    float d = p.dot(E1);
    
    if (d == 0)
        return false;
	
    Vector3f S = ray.Q - V[0];
    float u = p.dot(S) / d;
	
    if (u < 0 || u > 1)
        return false;
	
    Vector3f q = S.cross(E1);
    float v = ray.D.dot(q) / d;
	
    if (v < 0 || v + u > 1)
        return false;
	
    float t = E2.dot(q) / d;
	
    if (t < 0)
        return false;
	
    intersection.t = t;
    intersection.P = ray.eval(intersection.t);
	
    intersection.N = (1 - u - v) * N[0] + u * N[1] + v * N[2];
	intersection.N.normalize();
	
    intersection.object = this;
	
    return true;
}

Box3d Triangle::bbox() const
{
    return Box3d(Vector3f(fminf(fminf(V[0].x(), V[1].x()), V[2].x()),
                          fminf(fminf(V[0].y(), V[1].y()), V[2].y()),
                          fminf(fminf(V[0].z(), V[1].z()), V[2].z())),
                 Vector3f(fmaxf(fmaxf(V[0].x(), V[1].x()), V[2].x()) + 0.001f,
                          fmaxf(fmaxf(V[0].y(), V[1].y()), V[2].y()) + 0.001f,
                          fmaxf(fmaxf(V[0].z(), V[1].z()), V[2].z()) + 0.001f));
}

Cylinder::Cylinder(Vector3f B, Vector3f A, float r, Material* material) : B(B), A(A), r(r)
{
    shapeMaterial = material;
}

bool Cylinder::intersect(Ray ray, Intersecton & intersection)
{
    Quaternionf q = Quaternionf::FromTwoVectors(A, Vector3f::UnitZ());
    Ray rRay(q._transformVector(ray.Q - B), q._transformVector(ray.D));
	
    Interval* interval2 = new Interval();
    Interval interval = interval2->intersect(rRay, Slab(Vector3f(0.0f,0.0f,1.0f), 0, -A.norm()));
	
    float a = rRay.D.x() * rRay.D.x() + rRay.D.y() * rRay.D.y();
    float b = 2 * (rRay.D.x() * rRay.Q.x() + rRay.D.y() * rRay.Q.y());
    float c = rRay.Q.x() * rRay.Q.x() + rRay.Q.y() * rRay.Q.y() - r * r;
	
    if (b * b - 4 * a * c < 0)
        return false;
	
	if (a == 0)
		return false;
	
    float t0 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    float t1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
	
    Interval* interval3 = new Interval(t0, t1, Vector3f(), Vector3f());
	
    //float max = fmaxf(fmaxf(0, interval2->t0), interval3->t0);
    //float min = fminf(fminf(INFINITY, interval2->t1), interval3->t1);
	
	interval3->intersect(interval);
	interval3->intersect(Interval());
	
    if (interval3->t0 > interval3->t1)
        return false;
    
    if (interval3->t0 <= 0 && interval3->t1 <= 0)
        return false;
	
	//if (interval3->t0 < 0)
	//	return false;
	//if (interval3->t1 < interval3->t0)
	//	return false;
	
    if (fminf(interval3->t0, interval3->t1) <= 0)
        intersection.t = fmaxf(interval3->t0, interval3->t1);
    else
        intersection.t = fminf(interval3->t0, interval3->t1);
	
    intersection.P = ray.eval(intersection.t);
	
	Vector3f M = rRay.Q + intersection.t * rRay.D;
	Vector3f rN(M.x(), M.y(), 0.0f);
	
	intersection.N = q.conjugate()._transformVector(rN);
	intersection.N.normalize();
    intersection.object = this;
	
    return true;
}

Box3d Cylinder::bbox() const
{
    Vector3f BaseBoxMin(    B - Vector3f(r, r, r));
    Vector3f BaseBoxMax(    B + Vector3f(r, r, r));
    Vector3f TopBoxMin( A + B - Vector3f(r, r, r));
    Vector3f TopBoxMax( A + B + Vector3f(r, r, r));
    return Box3d(BaseBoxMin, BaseBoxMax).extend(Box3d(TopBoxMin, TopBoxMax));
}

Box::Box(Vector3f P, Vector3f Pd, Material* material) : P(P), Pd(Pd)
{
    shapeMaterial = material;
}

bool Box::intersect(Ray ray, Intersecton & intersection)
{
    std::vector<Slab> box;
    box.push_back(Slab(Vector3f(1.0f, 0.0f, 0.0f), -P.x(), -P.x() - Pd.x()));
    box.push_back(Slab(Vector3f(0.0f, 1.0f, 0.0f), -P.y(), -P.y() - Pd.y()));
    box.push_back(Slab(Vector3f(0.0f, 0.0f, 1.0f), -P.z(), -P.z() - Pd.z()));

    Interval interval;
    interval.t0 = 0;
    interval.t1 = INFINITY;

    interval = interval.intersect(ray, box);

	//if (interval.t0 < 0)
	//	return false;

    if (interval.t0 > interval.t1)
        return false;

    if (fmaxf(interval.t0, interval.t1) <= 0)
        return false;

	if (fminf(interval.t0, interval.t1) <= 0)
		intersection.t = fmaxf(interval.t0, interval.t1);
	else
		intersection.t = fminf(interval.t0, interval.t1);

	if (intersection.t == interval.t0)
		intersection.N = interval.N0;
	else
		intersection.N = interval.N1;

    intersection.P = ray.eval(intersection.t);
    intersection.object = this;

    return true;
}

Box3d Box::bbox() const
{
    return Box3d(P, P + Pd);
}

Interval::Interval()
    : t0(0.0f), t1(INFINITY)
{
}

Interval::Interval(float pt0, float pt1, Vector3f pN0, Vector3f pN1)
{
    if (pt0 <= pt1)
    {
        t0 = pt0;
        t1 = pt1;
        N0 = pN0;
        N1 = pN1;
    }
    else
    {
        t1 = pt0;
        t0 = pt1;
        N1 = pN0;
        N0 = pN1;
    }
}

void Interval::empty()
{
    t0 = 0.0f;
    t1 = -1.0f;
}

void Interval::intersect(Interval other)
{
    float t0max = fmaxf(t0, other.t0);
    float t1min = fminf(t1, other.t1);

    float d = t1min - t0max;

    if (d <= 0)
    {
        empty();
        return;
    }

    if (t0max == t0 && t1min == t1)
        return;

    if (t0max == t0)
    {
        t1 = other.t1;
        N1 = other.N1;
        return;
    }

    if (t1min == t1)
    {
        t0 = other.t0;
        N0 = other.N0;
        return;
    }

    t1 = other.t1;
    N1 = other.N1;
    t0 = other.t0;
    N0 = other.N0;
    return;
}

Interval Interval::intersect(Ray ray, Slab slab)
{
    if (slab.N.dot(ray.D) != 0)
    {
        float t0 = -(slab.d0 + slab.N.dot(ray.Q)) / slab.N.dot(ray.D);
        float t1 = -(slab.d1 + slab.N.dot(ray.Q)) / slab.N.dot(ray.D);
        return Interval(t0, t1, -slab.N, slab.N);
    }
    else
    {
        if (signbit(slab.N.dot(ray.Q) + slab.d0) != signbit(slab.N.dot(ray.Q) + slab.d1))
            return Interval();
        else
            return Interval(1.0f, 0.0f, Vector3f(), Vector3f());
    }
}

Interval Interval::intersect(Ray ray, std::vector<Slab> slabs)
{
    Interval result;
    result.t0 = 0;
    result.t1 = INFINITY;
    std::vector<Slab>::iterator it = slabs.begin();

    for (; it != slabs.end(); ++it)
    {
        Interval current = intersect(ray, *it);
		result.intersect(current);
        //result.t0 = fmaxf(result.t0, current.t0);
        //result.t1 = fminf(result.t1, current.t1);
    }

    return result;
}

Minimizer::Minimizer(const Ray & r)
    : ray(r) 
{
    minIntersection.t = INFINITY;
}

float Minimizer::minimumOnObject(Shape * obj)
{
    Intersecton curIntersection;
    if (obj->intersect(ray, curIntersection))
    {
        if (curIntersection.t < minIntersection.t)
            minIntersection = curIntersection;
        return curIntersection.t;
    }
    return INFINITY;
}

float Minimizer::minimumOnVolume(const Box3d & box)
{
    std::vector<Slab> sbox;
    sbox.push_back(Slab(Vector3f(1.0f, 0.0f, 0.0f), -box.min().x(), -box.max().x()));
    sbox.push_back(Slab(Vector3f(0.0f, 1.0f, 0.0f), -box.min().y(), -box.max().y()));
    sbox.push_back(Slab(Vector3f(0.0f, 0.0f, 1.0f), -box.min().z(), -box.max().z()));

    Interval interval;
    interval.t0 = 0;
    interval.t1 = INFINITY;

    interval = interval.intersect(ray, sbox);

	//if (interval.t0 < 0)
	//	return INFINITY;

    if (interval.t0 > interval.t1)
        return INFINITY;

    if (fmaxf(interval.t0, interval.t1) <= 0)
        return INFINITY;

    float cur_t;

	if (fminf(interval.t0, interval.t1) <= 0)
		return 0;
    else
        cur_t = fminf(interval.t0, interval.t1);

    return cur_t;
}
