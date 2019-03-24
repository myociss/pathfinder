#include <Eigen/Dense>
#include <math.h>
#include "plane3d.hpp"

using namespace Eigen;

Plane3d::Plane3d(float alpha, float theta){
    Vector3f u0(1, 0, 0);
    Vector3f v0(0, cos(alpha), sin(alpha));
    Vector3f w0(0, -sin(alpha), cos(alpha));

    Matrix3f rotationX;
    rotationX << u0[0], u0[1], u0[2],
		 v0[0], v0[1], v0[2],
		 w0[0], w0[1], w0[2];

    Vector3f u1(cos(theta), 0, sin(theta));
    Vector3f v1(0, 1, 0);
    Vector3f w1(-sin(theta), 0, cos(theta));

    Matrix3f rotationY;
    rotationY << u1[0], u1[1], u1[2],
		 v1[0], v1[1], v1[2],
		 w1[0], w1[1], w1[2];

    rotation = rotationX * rotationY;

    //axisX = totalRotation[0];
    //axisY = totalRotation[1];
    //normal = totalRotation[2];

    //rotationInverse = totalRotation.inverse();
}

Vector3f Plane3d::getNormal(){
    return Vector3f(rotation.row(2)[0], rotation.row(2)[2], rotation.row(2)[2]);
}


    
