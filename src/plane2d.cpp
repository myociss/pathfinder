#include "plane2d.hpp"
#include <math.h>
//#include <sort.h>

using namespace Eigen;
using namespace std;

Plane2d::Plane2d(vector<Shape3d>& shapes, Plane3d plane3d){
    unsigned long int count = 0;
    
    for(unsigned long int i=0; i<shapes.size(); ++i){
	count += shapes[i].Vertices().size();
    }

    vector<Point2d> points;
    points.reserve(count);

    for(unsigned long int i=0; i<shapes.size(); ++i){
	vector<array<double, 3>> vertices = shapes[i].Vertices();
	for(int j=0; j<vertices.size(); ++j){
	    Vector2d rotatedPoint = plane3d.Rotate(vertices[j]);
	    Point2d point(rotatedPoint);
	    points.push_back(point);
	}
    }

    sort(points.begin(), points.end());

    //assignAngleIds(points)
}

//void assignAngleIds()


Point2d::Point2d(Vector2d _vec){
    vec = _vec;
    angle = atan2(vec[1], vec[0]);
    angleId = -1;
}
