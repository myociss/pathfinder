#include <Eigen/Dense>
#include <vector>
#include <array>
#include <functional>
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
    vector<reference_wrapper<Vertex3d>> vertices;
    vector<reference_wrapper<Tetrahedron>> neighbors;
    array<float, 3> sphereCenter;
    float sphereRadius;
    float weight;
  public:
    Tetrahedron (const int id, const vector<reference_wrapper<Vertex3d>> _vertices, const float _weight);
    void addNeighbor(reference_wrapper<Tetrahedron> neighbor);
    bool contains(const array<float, 3>);
    reference_wrapper<Vertex3d> Vertices();
    //vector<reference_wrapper<Tetrahedron>> getNeighbors();
    //vector<array<float, 3>> intersectsPlane(Plane3d plane);
    int getId();
};

class Face {
    vector<reference_wrapper<Vertex3d>> vertices;
    unsigned long int tetId;
  public:
    Face (vector<reference_wrapper<Vertex3d>> _vertices, unsigned long int _tetId);
    //bool intersects(Plane3d plane, Vector3f target);
    //vector<Vertex3d *> Vertices();
    //Tetrahedron* getTetrahedron();
};

#endif
