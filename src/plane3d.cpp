#include <Eigen/Dense>
#include <math.h>
#include "plane3d.hpp"

using namespace Eigen;

Plane3d::Plane3d(float alpha, float theta, Vector3f _target){
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

    Matrix3f rotation = rotationX * rotationY;
    axisX = Vector3f(rotation.row(0)[0], rotation.row(0)[2], rotation.row(0)[2]);
    axisY = Vector3f(rotation.row(1)[0], rotation.row(1)[2], rotation.row(1)[2]);
    normal = Vector3f(rotation.row(2)[0], rotation.row(2)[2], rotation.row(2)[2]);
    target = _target;

    //normalDist = target.dot(getNormal());

    //axisX = totalRotation[0];
    //axisY = totalRotation[1];
    //normal = totalRotation[2];

    //rotationInverse = totalRotation.inverse();
}

bool Plane3d::intersects(Vector3f v0, Vector3f v1){
    Vector3f diffVec0 = v0 - target;
    float dotProduct0 = diffVec0.dot(normal);

    Vector3f diffVec1 = v1 - target;	 
    float dotProduct1 = diffVec1.dot(normal);

    return dotProduct0 * dotProduct1 < 0;
}

std::array<float, 3> Plane3d::findIntersection(Vector3f v0, Vector3f v1){
    Vector3f w = v0 - target;
    Vector3f u = v1 - target;

    float N = -normal.dot(w);
    float D = normal.dot(u);

    Vector3f coords3d(v1[0] + (N/D) * u[0], v1[1] + (N/D) * u[1], v1[2] + (N/D) * u[2]);

    Vector3f dist = coords3d-target;
    float x2d = dist.dot(axisX);
    float y2d = dist.dot(axisY);

    return {x2d, y2d, atan2(y2d, x2d)};
}

//Vector3f Plane3d::getNormal(){
//    return Vector3f(rotation.row(2)[0], rotation.row(2)[2], rotation.row(2)[2]);
//}


    
