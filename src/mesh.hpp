#include <array>
#include <vector>
#include "plane3d.hpp"
#include "mesh_components.hpp"
#include <Eigen/Dense>

using namespace Eigen;

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
    std::vector<Tetrahedron *> findIntersectingOuterTets(Plane3d plane);
    std::vector<std::vector<std::array<float, 3>>> slice(float alpha, float theta);
    //these functions are primarily exposed to Python for testing
    int getTargetTetId();
};

