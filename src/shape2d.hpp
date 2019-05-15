#include <Eigen/Dense>
#include <vector>
#include <array>
#include <memory>

using namespace Eigen;
using namespace std;

#ifndef SHAPE2D_HPP
#define SHAPE2D_HPP

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
    void setAngleId(unsigned long int _angleId);
    int ShapeVectorPos();
    unsigned long int ShapeId();
};

class Shape2d {
    vector<Point2d> vertices;
  public:
    Shape2d(int numVertices);
    void addPoint(Point2d point);
    vector<Vector2d> Vertices();
};


#endif
