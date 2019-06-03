#include <Eigen/Dense>
#include <vector>
#include <array>

using namespace Eigen;
using namespace std;

#ifndef LINE_INTERVAL_HPP
#define LINE_INTERVAL_HPP

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
    double UpperBound();
    double LowerBound();
    double MaxWidth();
    array<double, 3> Divide();
    vector<unsigned long int> ShapeIds();
    array<Vector2d, 2> EndPoints();
};

#endif