#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "plane3d.hpp"

using namespace Eigen;
using namespace std;

Plane3d::Plane3d(int _id, double alpha, double theta, Vector3d _target){
    id = _id;
    Vector3d u0(1, 0, 0);
    Vector3d v0(0, cos(alpha), sin(alpha));
    Vector3d w0(0, -sin(alpha), cos(alpha));

    Matrix3f rotationX;
    rotationX << u0[0], u0[1], u0[2],
		 v0[0], v0[1], v0[2],
		 w0[0], w0[1], w0[2];


    Vector3d u1(cos(theta), 0, sin(theta));
    Vector3d v1(0, 1, 0);
    Vector3d w1(-sin(theta), 0, cos(theta));

    Matrix3f rotationY;
    rotationY << u1[0], u1[1], u1[2],
		 v1[0], v1[1], v1[2],
		 w1[0], w1[1], w1[2];


    Matrix3f rotation = rotationX * rotationY;

    axisX = Vector3d(rotation.row(0)[0], rotation.row(0)[1], rotation.row(0)[2]);
    axisY = Vector3d(rotation.row(1)[0], rotation.row(1)[1], rotation.row(1)[2]);
    normal = Vector3d(rotation.row(2)[0], rotation.row(2)[1], rotation.row(2)[2]);
    target = _target;
}

bool Plane3d::intersectsEdge(Vector3d v0, Vector3d v1){
    Vector3d diffVec0 = v0 - target;
    double dotProduct0 = diffVec0.dot(normal);

    Vector3d diffVec1 = v1 - target;	 
    double dotProduct1 = diffVec1.dot(normal);

    return dotProduct0 * dotProduct1 < 0;
}

/*bool Plane3d::intersectsFace(Vector3f v0, Vector3f v1, Vector3f v2){
    Vector3f diffVec0 = v0 - target;
    float dotProduct0 = diffVec0.dot(normal);

    Vector3f diffVec1 = v1 - target;	 
    float dotProduct1 = diffVec1.dot(normal);

    Vector3f diffVec2 = v2 - target;	 
    float dotProduct2 = diffVec2.dot(normal);

    return (dotProduct0 * dotProduct1 < 0 || dotProduct0 * dotProduct2 < 0);
}*/

bool Plane3d::containsPoint(Vector3d v0){
    Vector3d diffVec0 = v0 - target;
    double dotProduct0 = diffVec0.dot(normal);
    return dotProduct0 == 0;
}

/*bool Plane3d::containsEdge(Vector3f v0, Vector3f v1){
    Vector3f diffVec0 = v0 - target;
    float dotProduct0 = diffVec0.dot(normal);

    Vector3f diffVec1 = v1 - target;	 
    float dotProduct1 = diffVec1.dot(normal);

    return (dotProduct0 == 0 && dotProduct1 == 0);
}*/

array<double, 3> Plane3d::findIntersection(Vector3d v0, Vector3d v1){
    Vector3d w = v0 - target;
    Vector3d u = v1 - v0;

    double N = -(normal.dot(w));
    double D = normal.dot(u);

    array<double, 3> coords3d = {v0[0] + (N/D) * u[0], v0[1] + (N/D) * u[1], v0[2] + (N/D) * u[2]};

    return coords3d;

    //Vector3f dist = coords3d-target;
    //float x2d = dist.dot(axisX);
    //float y2d = dist.dot(axisY);

    //return {x2d, y2d, atan2(y2d, x2d)};
}

int Plane3d::Id(){
    return id;
}

Vector2d Plane3d::Rotate(array<double, 3> _vec){
    Vector3d vec(_vec[0], _vec[1], _vec[2]);
    double xCoord = axisX.dot(vec - target);
    double yCoord = axisY.dot(vec - target);

    return Vector2d(xCoord, yCoord);
}

/*Vector3d Plane3d::Target(){
    return target;
}*/

//Vector3f Plane3d::getNormal(){
//    return Vector3f(rotation.row(2)[0], rotation.row(2)[2], rotation.row(2)[2]);
//}


Shape3d::Shape3d(unsigned long int _tetId, vector<array<double, 3>> _vertices, double _weight, int _label){
    tetId = _tetId;
    vertices = _vertices;
    weight = _weight;
    label = _label;
}

unsigned long int Shape3d::TetId(){
    return tetId;
}

vector<array<double, 3>> Shape3d::Vertices(){
    //return Vector3d(vertices[0], vertices[1], vertices[2]);
    //vector<Vector3d> verticesAsVecs;

    //for(int i=0; i<
    return vertices;
}

double Shape3d::Weight(){
    return weight;
}

int Shape3d::Label(){
    return label;
}
