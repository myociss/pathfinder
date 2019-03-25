#include <Eigen/Dense>
#include <vector>
#include <array>

using namespace Eigen;

#ifndef MESH_COMPONENTS_HPP
#define MESH_COMPONENTS_HPP


class Vertex3d {
    Vector3f vec;
    //std::vector<Tetrahedron *> tetrahedrons;
  public:
    Vertex3d (std::array<float, 3>);
    Vector3f Vec();
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
    std::vector<Vertex3d *> Vertices();
    std::vector<Tetrahedron *> getNeighbors();
    bool contains(const std::array<float, 3>);
    int getId();
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

#endif
