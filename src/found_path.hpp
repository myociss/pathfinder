#include <array>
#include <vector>
#include "plane3d.hpp"
#include "mesh_components.hpp"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#ifndef FOUND_PATH_HPP
#define FOUND_PATH_HPP

class FoundPath{
    unsigned long int planeId;
    Vector3d pt0;
    Vector3d pt1;
    double lowerBound;
    double upperBound;
  public:
    FoundPath(unsigned long int _planeId, Vector3d _pt0, Vector3d _pt1, double _lowerBound, double _upperBound);
    unsigned long int PlaneId();
    double UpperBound();
    double LowerBound();
    array<Vector3d, 2> Points();
};

#endif

