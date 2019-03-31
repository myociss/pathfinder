#include <Eigen/Dense>
#include <vector>
#include <array>
#include "plane3d.hpp"

using namespace Eigen;
using namespace std;

#ifndef MESH_COMPONENTS_HPP
#define MESH_COMPONENTS_HPP


class Vertex3d {
    Vector3f vec;
    //std::vector<Tetrahedron *> tetrahedrons;
  public:
    Vertex3d (array<float, 3>);
    Vector3f Vec();
};

class Tetrahedron {
    int id;
    vector<Vertex3d *> vertices;
    vector<Tetrahedron *> neighbors;
    array<float, 3> sphereCenter;
    float sphereRadius;
    float weight;
  public:
    Tetrahedron (const int id, const array<Vertex3d *, 4> _vertices, const float _weight);
    void addNeighbor(Tetrahedron * neighbor);
    vector<Vertex3d *> Vertices();
    vector<Tetrahedron *> getNeighbors();
    vector<array<float, 3>> intersectsPlane(Plane3d plane);
    bool contains(const array<float, 3>);
    int getId();
};


class Face {
    vector<Vertex3d *> vertices;
    Tetrahedron * tetrahedron;
  public:
    Face (array<Vertex3d *, 3> _vertices, Tetrahedron * tetrahdron);
    //bool intersects(Plane3d plane, Vector3f target);
    vector<Vertex3d *> Vertices();
    Tetrahedron* getTetrahedron();
};

#endif
