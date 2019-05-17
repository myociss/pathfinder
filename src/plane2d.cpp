#include "plane2d.hpp"
#include "shape2d.hpp"
#include <math.h>
#include <iostream>
#include <memory>
//#include <sort.h>

using namespace Eigen;
using namespace std;

Plane2d::Plane2d(vector<Shape3d>& _shapes, Plane3d plane3d){
    unsigned long int count = 0;
    shapes.reserve(_shapes.size());
    
    for(unsigned long int i=0; i<_shapes.size(); ++i){
	int numVertices = _shapes[i].Vertices().size();
	count += numVertices;
	Shape2d shape(numVertices);
	shapes.push_back(shape);
    }

    vector<Point2d> points;
    points.reserve(count);
    count=0;

    for(unsigned long int i=0; i<_shapes.size(); ++i){
	vector<array<double, 3>> vertices = _shapes[i].Vertices();
	for(int j=0; j<vertices.size(); ++j){
	    Vector2d rotatedPoint = plane3d.Rotate(vertices[j]);
	    Point2d point(rotatedPoint, i, j);
	    points.push_back(point);
	    //cout << point.Vec() << endl;
	    ++count;
	}
    }
    //cout << points.size() << endl;

    sort(points.begin(), points.end());
    unsigned long int angleCount = 0;
    double anglePrev = points[0].Angle();

    for(unsigned long int i=0; i<points.size(); ++i){
	double angle = points[i].Angle();
	if(angle!=anglePrev || i==0){
	    SweepLineInterval sli(points[i].Vec());
	    sweepLineIntervals.push_back(sli);
	    ++angleCount;
	}
	points[i].setAngleId(angleCount-1);
	anglePrev=angle;
	unsigned long int shapeId=points[i].ShapeId();
	shapes[shapeId].addPoint(points[i]);
	if(shapes[shapeId].Complete() && shapeId!=0){
	    shapes[shapeId].arrange(sweepLineIntervals);
	}
	//cout << angle << endl;
	//cout << points[i].ShapeId() << endl;
	//cout << points[i].ShapeVectorPos() << endl;
    }
}

//void Plane2d::GetPaths(){
    
//}

SweepLineInterval Plane2d::getSweepLineAt(unsigned long int angleId){
    return sweepLineIntervals[angleId];
}

vector<Shape2d> Plane2d::Shapes(){
    return shapes;
}

SweepLineInterval::SweepLineInterval(Vector2d _point){
    point = _point;
    lineComponents[0] = -point[1];
    lineComponents[1] = point[0];
    lineComponents[2] = 0.0;
}

Vector2d SweepLineInterval::Point(){
    return point;
}

//void assignAngleIds()


