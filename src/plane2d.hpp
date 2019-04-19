#include <Eigen/Dense>
#include <vector>
#include <array>
#include "plane3d.hpp"

using namespace Eigen;
using namespace std;

#ifndef PLANE2D_HPP
#define PLANE2D_HPP

class Plane2d{
    vector<Vector2d> uniquePoints;
    vector<Vector2d> uniquePolar;
    vector<Vector3d> uniqueAngles;
  public:
    Plane2d(vector<Shape3d>& shapes, Plane3d plane3d);
};

class Point2d{
    Vector2d vec;
    //double angle;
    //double angleCos;
    //double angleSin;
    unsigned long int angleId;
  public:
    double angle;
    Point2d(Vector2d _vec);
    bool operator<(const Point2d &other) const {
	return angle < other.angle;
    }
};

#endif
