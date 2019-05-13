#include <Eigen/Dense>
#include <vector>
#include <array>

using namespace Eigen;
using namespace std;

#ifndef SHAPE2D_HPP
#define SHAPE2D_HPP

class Point2d {
    Vector2d vec;
    unsigned long int angleId;
  public:
    double angle;
    Point2d(Vector2d _vec);
    bool operator<(const Point2d &other) const {
	return angle < other.angle;
    }
    Vector2d Vec();
};

class Shape2d {
    vector<reference_wrapper<Point2d>> vertices;
  public:
    Shape2d(vector<reference_wrapper<Point2d>>& _vertices);
    vector<Vector2d> Vertices();
};


#endif
