#include <array>
#include <vector>
#include "found_path.hpp"
#include "mesh_components.hpp"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


FoundPath::FoundPath(unsigned long int _planeId, Vector3d _pt0, Vector3d _pt1, double _lowerBound, double _upperBound){
    planeId=_planeId;
    pt0=_pt0;
    pt1=_pt1;
    lowerBound=_lowerBound;
    upperBound=_upperBound;
}

unsigned long int FoundPath::PlaneId(){
    return planeId;
}

double FoundPath::UpperBound(){
    return upperBound;
}

double FoundPath::LowerBound(){
    return lowerBound;
}

array<Vector3d, 2> FoundPath::Points(){
    return {pt0, pt1};
}

