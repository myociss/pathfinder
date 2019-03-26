#include <Eigen/Dense>

using namespace Eigen;

#ifndef PLANE3D_HPP
#define PLANE3D_HPP

class Plane3d {
    /*Vector3f axisX;
    Vector3f axisY;
    Vector3f normal;
    Matrix3f rotationInverse;*/
    //Matrix3f rotation;
    Vector3f axisX;
    Vector3f axisY;
    Vector3f normal;
    Vector3f target;
    //float normalDist;
  public:
    Plane3d (float alpha, float theta, Vector3f _target);
    bool intersects(Vector3f v0, Vector3f v1);
    bool containsEdge(Vector3f v0, Vector3f v1);
    std::array<float, 3> findIntersection(Vector3f v0, Vector3f v1);
};
#endif
