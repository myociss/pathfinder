#include "shape2d.hpp"
#include <math.h>
//#include <sort.h>

using namespace Eigen;
using namespace std;

Shape2d::Shape2d(vector<reference_wrapper<Point2d>>& _vertices){
    vertices = _vertices;
}

vector<Vector2d> Shape2d::Vertices(){
    vector<Vector2d> vecs;
    for(int i=0; i<vertices.size(); i++){
	vecs.push_back(vertices[i].get().Vec());
    }
    return vecs;
}

Point2d::Point2d(Vector2d _vec){
    vec = _vec;
    angle = atan2(vec[1], vec[0]);
    angleId = -1;
}

Vector2d Point2d::Vec(){
    return vec;
}
