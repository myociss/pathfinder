#include <array>
#include <vector>
#include "plane3d.hpp"
#include <Eigen/Dense>

using namespace Eigen;
//using namespace plane3d;

class Vertex3d;
class Face;
class Tetrahedron;

bool intersectsVertices(Plane3d plane, std::vector<Vertex3d *> vertices);

class Mesh {
    std::vector<Vertex3d *> vertices;
    std::vector<Face *> faces;
    std::vector<Tetrahedron *> tetrahedrons;
    int numThreads;
    Vector3f target;
    Tetrahedron * targetTet;
  public:
    Mesh (const int numVertices, const int numFaces, const int numCells, const int _numThreads);
    void setVertices(const std::vector<std::array<float, 3>> & _vertices);
    void addTetrahedron(const int id, const std::array<int, 4> vertexIds, const std::vector<int> neighborIds, const float weight);
    bool setTarget(const std::array<float, 3> _target);
    void addFace(const std::array<int, 3> vertexIds, const int tetId);
    std::vector<Tetrahedron *> findIntersectingOuterTets(float alpha, float theta);

    //these functions are primarily exposed to Python for testing
    int getTargetTetId();
};

class Vertex3d {
    Vector3f vec;
    std::vector<Tetrahedron *> tetrahedrons;
  public:
    Vertex3d (std::array<float, 3>);
    Vector3f Vec();
};


class Face {
    std::vector<Vertex3d *> vertices;
    Tetrahedron * tetrahedron;
  public:
    Face (std::array<Vertex3d *, 3> _vertices, Tetrahedron * tetrahdron);
    //bool intersects(Plane3d plane, Vector3f target);
    std::vector<Vertex3d *> Vertices();
    Tetrahedron* getTetrahedron();
};

class Tetrahedron {
    int id;
    std::vector<Vertex3d *> vertices;
    std::vector<Tetrahedron *> neighbors;
    std::vector<int> threads;
    std::array<float, 3> sphereCenter;
    float sphereRadius;
    float weight;
  public:
    Tetrahedron (const int id, const std::array<Vertex3d *, 4> _vertices, const float 	  _weight, const int numThreads);
    void addNeighbor(Tetrahedron * neighbor);
    bool contains(const std::array<float, 3>);
    int getId();
};
