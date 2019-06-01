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
    double angleStart;
    double angleEnd;
    double distLowerBound;
    double distUpperBound;
    array<double, 2> intervalMaxDists;
    vector<unsigned long int> shapeIds;
  public:
    LineInterval(Vector2d _point);
    LineInterval(double _angleStart, double _angleEnd);
    void SetAngleEnd(Vector2d _point);
    Vector2d Point();
    array<double, 3> FunctionsAt(array<double, 2> edge, int side);
    double DistAt(array<double, 2> edge, int side);
    bool containsNormal(array<double, 2> edge);
    double ApproxRoot(double distStart, double distEnd, double derivStart, double derivEnd);
    void update(double upperBound, double lowerBound, double distStart, double distEnd, unsigned long int shapeId);
    //void updateLowerBound(double val);
    //void updateUpperBound(double val);
    double UpperBound();
    double LowerBound();
    double MaxWidth();
    vector<unsigned long int> ShapeIds();
};

class Plane2d{
    vector<Shape2d> shapes;
    vector<LineInterval> lineIntervals;
    vector<LineInterval> candidateIntervals;
  public:
    Plane2d(vector<Shape3d>& shapes, Plane3d plane3d);
    vector<Shape2d> Shapes();
    void CalcLineIntervalsInit();
    void FindPaths();
    bool DivideCandidateIntervals();
    vector<array<double, 2>> LineIntervalBounds();
    vector<unsigned long int> IntervalShapeIds(unsigned long int intervalId);
};

#endif
