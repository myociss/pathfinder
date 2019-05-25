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
array<double, 2> polarEquation(Vector2d v0, Vector2d v1);
int ANGLE_START=0;
int ANGLE_END=1;

class LineInterval{
    Vector2d point;
    //array<double, 3> lineComponents;
    array<double, 2> polarComponents;
    double angleStart;
    double angleEnd;
    double distLowerBound;
    double distUpperBound;
  public:
    LineInterval(Vector2d _point);
    void SetAngleEnd(Vector2d _point);
    Vector2d Point();
    array<double, 3> FunctionsAt(array<double, 2> edge, int side);
    double ApproxRoot(double distStart, double distEnd, double derivStart, double derivEnd);
    void updateLowerBound(double val);
    void updateUpperBound(double val);
};


class Plane2d{
    vector<Shape2d> shapes;
    vector<LineInterval> lineIntervals;
  public:
    Plane2d(vector<Shape3d>& shapes, Plane3d plane3d);
    vector<Shape2d> Shapes();
    //SweepLineInterval getSweepLineAt(unsigned long int angleId);
};

#endif
