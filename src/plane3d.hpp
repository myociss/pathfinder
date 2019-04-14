#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#ifndef PLANE3D_HPP
#define PLANE3D_HPP

class Plane3d {
    /*Vector3f axisX;
    Vector3f axisY;
    Vector3f normal;
    Matrix3f rotationInverse;*/
    //Matrix3f rotation;
    int id;
    Vector3f axisX;
    Vector3f axisY;
    Vector3f normal;
    Vector3f target;
    //float normalDist;
  public:
    Plane3d (int _id, float alpha, float theta, Vector3f _target);
    bool intersectsEdge(Vector3f v0, Vector3f v1);
    //bool intersectsFace(Vector3f v0, Vector3f v1, Vector3f v2);
    //bool containsEdge(Vector3f v0, Vector3f v1);
    bool containsPoint(Vector3f v0);
    array<float, 3> findIntersection(Vector3f v0, Vector3f v1);
    int Id();
};
#endif
