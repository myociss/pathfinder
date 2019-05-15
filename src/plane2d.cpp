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
	shapes[points[i].ShapeId()].addPoint(points[i]);
	//cout << angle << endl;
	//cout << points[i].ShapeId() << endl;
	//cout << points[i].ShapeVectorPos() << endl;
    }

    /*vector<shared_ptr<Point2d>> points;
    points.reserve(count);

    shapes.reserve(_shapes.size());
    count = 0;

    for(unsigned long int i=0; i<_shapes.size(); ++i){
	vector<array<double, 3>> vertices = _shapes[i].Vertices();
	//vector<reference_wrapper<Point2d>> shape2dPoints;
	for(int j=0; j<vertices.size(); ++j){
	    Vector2d rotatedPoint = plane3d.Rotate(vertices[j]);
	    auto point_sp = make_shared<Point2d>(rotatedPoint)
	    //shared_ptr<Point2d> point(rotatedPoint);
	    points.push_back(point_sp);
	    //shape2dPoints.push_back(point);
	    //shape2dPoints.push_back(point);
	}
	vector<shared_ptr<Point2d>> shape2dPoints(points.begin()+count, points.end());
	Shape2d shape2d(shape2dPoints);
	shapes.push_back(shape2d);
	cout << "new shape points :/" << endl;
	vector<Vector2d> new_shape_vertices = shapes.back().Vertices();
	for(int k=0; k < new_shape_vertices.size(); k++){
	    cout << new_shape_vertices[k] << endl;
	}
	count+=vertices.size();
    }*/

    //sort(points.begin(), points.end());

    //assignAngleIds(points)
}

vector<Shape2d> Plane2d::Shapes(){
    return shapes;
}

SweepLineInterval::SweepLineInterval(Vector2d _point){
    point = _point;
}

//void assignAngleIds()


