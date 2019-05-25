#include "plane2d.hpp"
#include "shape2d.hpp"
#include <math.h>
#include <iostream>
#include <memory>
//#include <sort.h>

using namespace Eigen;
using namespace std;

array<double, 2> polarEquation(Vector2d v0, Vector2d v1){
    array<double, 3> line={v0[1]-v1[1], v1[0]-v0[0],-(v0[0]*v1[1]-v1[0]-v0[1])};

    double normalAngle=atan2(line[1], line[0]);
    double normalDist=line[2] / sqrt(line[0] * line[0] + line[1] * line[1]);
    array<double, 2> polar={normalDist, normalAngle};
    return polar;
}

Plane2d::Plane2d(vector<Shape3d>& _shapes, Plane3d plane3d){
    unsigned long int count = 0;
    shapes.reserve(_shapes.size());
    
    for(unsigned long int i=0; i<_shapes.size(); ++i){
	int numVertices = _shapes[i].Vertices().size();
	count += numVertices;
	Shape2d shape(numVertices, _shapes[i].Weight());
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
	    LineInterval li(points[i].Vec());
	    LineIntervals.push_back(li);
	    ++angleCount;

	    if(angleCount>1){
		LineIntervals[angleCount-2].SetAngleEnd(LineIntervals[angleCount-1].Point());
	    }
	}
	points[i].setAngleId(angleCount-1);
	anglePrev=angle;
	unsigned long int shapeId=points[i].ShapeId();
	shapes[shapeId].addPoint(points[i]);
	if(shapes[shapeId].Complete() && shapeId!=0){
	    shapes[shapeId].arrange(sweepLineIntervals);
	}
    }

    LineIntervals[angleCount-1].SetAngleEnd(LineIntervals[0].Point());
}


//SweepLineInterval Plane2d::getSweepLineAt(unsigned long int angleId){
//    return sweepLineIntervals[angleId];
//}

vector<Shape2d> Plane2d::Shapes(){
    return shapes;
}

LineInterval::LineInterval(Vector2d _point){
    point = _point;
    Vector2d zero(0.0, 0.0);
    polarComponents = polarEquation(point, zero);
    angleStart = atan2(point[1], point[0]);
    distLowerBound = 0.0;
    distUpperBound = 0.0;
}

void LineInterval::SetAngleEnd(Vector2d _point){
    angleEnd = atan2(_point[1], _point[0]);
}

void LineInterval::updateLowerBound(double val){
    distLowerBound+=val;
}

void LineInterval::updateUpperBound(double val){
    distUpperBound+=val;
}

Vector2d LineInterval::Point(){
    return point;
}

//void assignAngleIds()


