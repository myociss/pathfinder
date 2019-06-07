#pragma once
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <memory>
#include "line_interval.hpp"
#include "plane2d.hpp"

using namespace Eigen;
using namespace std;

#ifndef SHAPE2D_HPP
#define SHAPE2D_HPP

class LineInterval;

class Point2d {
    unsigned long int shapeId;
    int shapeVectorPos;
    Vector2d vec;
    unsigned long int angleId;
  public:
    double angle;
    Point2d(Vector2d _vec, unsigned long int _shapeId, int _shapeVectorPos);
    bool operator<(const Point2d &other) const {
	return angle < other.angle;
    }
    Vector2d Vec();
    double Angle();
    unsigned long int AngleId();
    void setAngleId(unsigned long int _angleId);
    int ShapeVectorPos();
    unsigned long int ShapeId();
};

class Shape2d {
    unsigned long int id;
    int numVertices;
    double weight;
    vector<Point2d> vertices;
    int endVertex;
    //vector<unsigned long int> prevShapeIds;
  public:
    Shape2d(unsigned long int _id, int _numVertices, double _weight);
    bool Complete();
    void addPoint(Point2d point);
    void arrange(vector<LineInterval>& lineIntervals);
    void setVerticesClockwise(int startVertex);
    void setNewVertices(vector<Point2d> tmpVertices);
    void calculateAllIntervals(vector<LineInterval>& lineIntervals);
    //there should probably be different subclasses for target shapes and nontarget shapes at some point
    void calculateOneInterval(LineInterval& li);
    void calculateAllIntervalsTarget(vector<LineInterval>& lineIntervals);
    void computeBounds(array<double, 3> entryFStart, array<double, 3> entryFEnd, array<double, 3> terminalFStart, array<double, 3> terminalFEnd, LineInterval& li);
    double maxDist();
    double Weight();
    unsigned long int Id();
    int EndVertex();
    vector<Vector2d> Vertices();
    vector<Vector2d> VerticesArranged();
    array<double, 2> EntryEdge(LineInterval& li);
    array<double, 2> TerminalEdge(LineInterval& li);
    //vector<unsigned long int> PrevShapeIds();
};


#endif
