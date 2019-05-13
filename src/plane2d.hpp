#include <Eigen/Dense>
#include <vector>
#include <array>
#include "plane3d.hpp"
#include "shape2d.hpp"

using namespace Eigen;
using namespace std;

#ifndef PLANE2D_HPP
#define PLANE2D_HPP

class Plane2d{
    vector<Shape2d> shapes;
    //vector<Vector2d> uniquePoints;
    //vector<Vector2d> uniquePolar;
    //vector<Vector3d> uniqueAngles;
  public:
    Plane2d(vector<Shape3d>& shapes, Plane3d plane3d);
};

#endif
