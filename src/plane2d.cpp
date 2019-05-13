#include "plane2d.hpp"
#include "shape2d.hpp"
#include <math.h>
//#include <sort.h>

using namespace Eigen;
using namespace std;

Plane2d::Plane2d(vector<Shape3d>& _shapes, Plane3d plane3d){
    unsigned long int count = 0;
    
    for(unsigned long int i=0; i<_shapes.size(); ++i){
	count += _shapes[i].Vertices().size();
    }

    vector<Point2d> points;
    points.reserve(count);

    shapes.reserve(_shapes.size());

    for(unsigned long int i=0; i<_shapes.size(); ++i){
	vector<array<double, 3>> vertices = _shapes[i].Vertices();
	vector<reference_wrapper<Point2d>> shape2dPoints;
	for(int j=0; j<vertices.size(); ++j){
	    Vector2d rotatedPoint = plane3d.Rotate(vertices[j]);
	    Point2d point(rotatedPoint);
	    points.push_back(point);
	    shape2dPoints.push_back(point);
	    //shape2dPoints.push_back(point);
	}
	Shape2d shape2d(shape2dPoints);
	shapes.push_back(shape2d);
    }

    sort(points.begin(), points.end());

    //assignAngleIds(points)
}

//void assignAngleIds()


