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
    Vector3d vec;
    //std::vector<Tetrahedron *> tetrahedrons;
  public:
    Vertex3d (array<double, 3>);
    Vector3d Vec();
};

class Tetrahedron {
    unsigned long int id;
    vector<reference_wrapper<Vertex3d>> vertices;
    vector<unsigned long int> neighbors;
    array<double, 3> sphereCenter;
    double sphereRadius;
    double weight;
  public:
    Tetrahedron (const int id, const vector<reference_wrapper<Vertex3d>> _vertices, const double _weight);
    void addNeighbor(unsigned long int neighborId);
    bool contains(const array<double, 3>);
    vector<reference_wrapper<Vertex3d>> Vertices();
    vector<unsigned long int> Neighbors();
    vector<array<double, 3>> intersectsPlane(Plane3d plane);
    unsigned long int Id();
    double Weight();
};

class Face {
    vector<reference_wrapper<Vertex3d>> vertices;
    unsigned long int tetId;
  public:
    Face (vector<reference_wrapper<Vertex3d>> _vertices, unsigned long int _tetId);
    vector<reference_wrapper<Vertex3d>> Vertices();
    unsigned long int TetId();
    bool intersectsPlane(Plane3d plane);
    //vector<Vertex3d *> Vertices();
    //Tetrahedron* getTetrahedron();
};

#endif
