#pragma once
#include <Eigen/Dense>
#include <vector>
#include <array>
#include "plane3d.hpp"
#include "shape2d.hpp"

using namespace Eigen;
using namespace std;

#ifndef PLANE2D_HPP
#define PLANE2D_HPP

class Shape2d;

class SweepLineInterval{
    Vector2d point;
    array<double, 3> lineComponents;
    array<double, 2> polarComponents;
  public:
    SweepLineInterval(Vector2d _point);
    Vector2d Point();
};


class Plane2d{
    vector<Shape2d> shapes;
    vector<SweepLineInterval> sweepLineIntervals;
    //vector<Vector2d> uniquePoints;
    //vector<Vector2d> uniquePolar;
    //vector<Vector3d> uniqueAngles;
  public:
    Plane2d(vector<Shape3d>& shapes, Plane3d plane3d);
    vector<Shape2d> Shapes();
    SweepLineInterval getSweepLineAt(unsigned long int angleId);
};

#endif
