#include <Eigen/Dense>

using namespace Eigen;

#ifndef PLANE3D_HPP
#define PLANE3D_HPP

class Plane3d {
    /*Vector3f axisX;
    Vector3f axisY;
    Vector3f normal;
    Matrix3f rotationInverse;*/
    Matrix3f rotation;
  public:
    Plane3d (float alpha, float theta);
    Vector3f getNormal();
};
#endif
