#include <Eigen/Dense>
#include <vector>
#include <array>

using namespace Eigen;
using namespace std;

#ifndef PLANE3D_HPP
#define PLANE3D_HPP

class Plane3d {
    int id;
    Vector3d axisX;
    Vector3d axisY;
    Vector3d normal;
    Vector3d target;
    Matrix3d inverse;
  public:
    Plane3d(int _id, double alpha, double theta, Vector3d _target);
    bool intersectsEdge(Vector3d v0, Vector3d v1);
    bool containsPoint(Vector3d v0);
    array<double, 3> findIntersection(Vector3d v0, Vector3d v1);
    int Id();
    Vector2d Rotate(array<double, 3> _vec);
    Vector3d Get3dPoint(Vector3d pt2d);
};

class Shape3d {
    unsigned long int tetId;
    vector<array<double, 3>> vertices;
    int label;
    //tissueId
    //these should probably all be doubles?
    double weight;
  public:
    Shape3d(unsigned long int _tetId, vector<array<double, 3>> _vertices, double _weight, int _label);
    unsigned long int TetId();
    vector<array<double, 3>> Vertices();
    int Label();
    double Weight();
};

#endif

