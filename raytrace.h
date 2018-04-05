typedef Eigen::AlignedBox<float, 3> Box3d;  // The BV type provided by Eigen

class Intersecton;
class Ray;
class Shape;
class Material;

class Ray
{
public:
    Vector3f Q;
    Vector3f D;
    Vector3f eval(float t);
    Ray(const Vector3f& q, const Vector3f& d)
        : Q(q), D(d)
    {}
};

class Intersecton
{
public:
    float t;
    Shape* object;
    Vector3f P;
    Vector3f N;
    Intersecton()
        : t(0), object(nullptr), P(), N()
    {}
    Intersecton(float t, Shape* object, const Vector3f& p, const Vector3f& n)
        : t(t), object(object), P(p), N(n)
    {}
};

class Shape
{
public:
    virtual bool intersect(Ray ray, Intersecton& intersection) = 0;
    virtual Box3d bbox() const = 0;
    Material* shapeMaterial;
	float PdfBRDF(const Vector3f & N, const Vector3f & w);
	Vector3f EvalBRDF();
	Vector3f SampleBRDF(const Vector3f & N);
	Vector3f SampleLobe(const Vector3f & N, float c, float phi);
};

class Sphere : public Shape
{
public:
    Sphere(Vector3f C, float r, Material* material);
    bool intersect(Ray ray, Intersecton& intersection);
    Box3d bbox() const;
    Vector3f C;
    float r;
};

class Triangle : public Shape
{
public:
    Triangle(Vector3f V1, Vector3f V2, Vector3f V3,
        Vector3f N1, Vector3f N2, Vector3f N3, Material* material);
    bool intersect(Ray ray, Intersecton& intersection);
    Box3d bbox() const;
    Vector3f V[3];
    Vector3f N[3];
};

class Cylinder : public Shape
{
public:
    Cylinder(Vector3f B, Vector3f A, float r, Material* material);
    bool intersect(Ray ray, Intersecton& intersection);
    Box3d bbox() const;
    Vector3f B;
    Vector3f A;
    float r;
};

class Box : public Shape
{
public:
    Box(Vector3f P, Vector3f Pd, Material* material);
    bool intersect(Ray ray, Intersecton& intersection);
    Box3d bbox() const;
    Vector3f P;
    Vector3f Pd;
};

class Slab
{
public:
    Slab(Vector3f N, float d0, float d1) :
        N(N), d0(d0), d1(d1) {}
    Vector3f N;
    float d0;
    float d1;
};

class Interval
{
public:
    float t0, t1;
    Vector3f N0, N1;
    Interval();
    Interval(float pt0, float pt1, Vector3f pN0, Vector3f pN1);
    void empty();
    void intersect(Interval other);
    Interval intersect(Ray ray, Slab slab);
    Interval intersect(Ray ray, std::vector<Slab> slabs);
};

class Camera
{
public:
    Camera(Vector3f eyePoint, Quaternionf viewOrientation, float ry) :
        eyePoint(eyePoint), viewOrientation(viewOrientation), ry(ry) {}
	Vector3f eyePoint;
	Quaternionf viewOrientation;
	float ry;
};

const float PI = 3.14159f;
const float Radians = PI / 180.0f;    // Convert degrees to radians

class Material
{
 public:
    Vector3f Kd, Ks;
    float alpha;
    unsigned int texid;

    virtual bool isLight() { return false; }

    Material()  : Kd(Vector3f(1.0, 0.5, 0.0)), Ks(Vector3f(1,1,1)), alpha(1.0), texid(0) {}
    Material(const Vector3f d, const Vector3f s, const float a) 
        : Kd(d), Ks(s), alpha(a), texid(0) {}
    Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha;  texid=o.texid; }

    void setTexture(const std::string path);
    //virtual void apply(const unsigned int program);
};

class Light : public Material
{
public:

    Light(const Vector3f e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (TriData: consisting of three
// indices into the vertex array).
typedef Eigen::Matrix<unsigned int, 3, 1 > TriData;
    
class VertexData
{
 public:
    Vector3f pnt;
    Vector3f nrm;
    Vector2f tex;
    Vector3f tan;
    VertexData(const Vector3f& p, const Vector3f& n, const Vector2f& t, const Vector3f& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<TriData> triangles;
    Material *mat;
};


class Minimizer
{
public:
    typedef float Scalar;
    // KdBVH needs Minimizer::Scalar defined
    Ray ray;
    Intersecton minIntersection;
    // Constructor
    Minimizer(const Ray& r);
    // Called by BVMinimize to intersect the ray with a Shape.  This
    // should return the intersection t, but should also track
    // the minimum t and it's corresponding intersection info.
    // Return INF to indicate no intersection.
    float minimumOnObject(Shape* obj);
    // Called by BVMinimize to intersect the ray with a Box3d and
    // returns the t value.  This should be similar to the already
    // written box (3 slab) intersection.  (The difference begin a 
    // distance of zero should be returned if the ray starts within the bbox.)
    // Return INF to indicate no intersection.
    float minimumOnVolume(const Box3d& box);
};

////////////////////////////////////////////////////////////////////////////////
// Scene
//class Realtime;

class Scene {
public:
    int width, height;
    Vector3f ambient;
	Camera* camera;
	std::vector<Shape*> objects;
	std::vector<Shape*> emitObjects;
    //Realtime* realtime;         // Remove this (realtime stuff)
    Material* currentMat;

    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

	Minimizer TraceRay(Ray& ray, KdBVH<float, 3, Shape*>& tree);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const Matrix4f& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);
	Color TracePath(Ray ray, KdBVH<float, 3, Shape*> & tree);
};