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

class LineInterval{
    Vector2d point;
    //array<double, 3> lineComponents;
    array<double, 2> polarComponents;
    double angleStart;
    double angleEnd;
    double distLowerBound;
    double distUpperBound;
    vector<unsigned long int> shapeIds;
    unsigned long int storedShapeId;
  public:
    LineInterval(Vector2d _point);
    void SetAngleEnd(Vector2d _point);
    Vector2d Point();
    array<double, 3> FunctionsAt(array<double, 2> edge, int side);
    double DistAt(array<double, 2> edge, int side);
    bool containsNormal(array<double, 2> edge);
    double ApproxRoot(double distStart, double distEnd, double derivStart, double derivEnd);
    //void update(double upperBound, double lowerBound, unsigned long int shapeId);
    unsigned long int update(double upperBound, double lowerBound, unsigned long int shapeId);
    void updateLowerBound(double val);
    void updateUpperBound(double val);
    double UpperBound();
    double LowerBound();
};


class Plane2d{
    vector<Shape2d> shapes;
    vector<LineInterval> lineIntervals;
  public:
    Plane2d(vector<Shape3d>& shapes, Plane3d plane3d);
    vector<Shape2d> Shapes();
    void CalcLineIntervalsInit();
    void FindPaths();
    //this is for python and is not used otherwise in this application
    vector<array<double, 2>> LineIntervalBounds();
};

#endif
