#pragma once
#include <Eigen/Dense>
#include <vector>
#include <array>
#include "plane3d.hpp"
#include "shape2d.hpp"
#include "line_interval.hpp"

using namespace Eigen;
using namespace std;

#ifndef PLANE2D_HPP
#define PLANE2D_HPP

class Shape2d;
array<double, 2> polarEquation(Vector2d v0, Vector2d v1);

class Plane2d{
    vector<Shape2d> shapes;
    vector<LineInterval> lineIntervals;
    vector<LineInterval> candidateIntervals;
    double minUpperBound;
  public:
    Plane2d(vector<Shape3d>& shapes, Plane3d plane3d);
    vector<Shape2d> Shapes();
    void CalcLineIntervalsInit();
    void FindPaths(double distBound);
    bool DivideCandidateIntervals(double distBound);
    vector<array<double, 2>> LineIntervalBounds();
    vector<unsigned long int> IntervalShapeIds(unsigned long int intervalId);
    double MinUpperBound();
    vector<LineInterval> CandidateIntervals();
};

#endif
