#include "plane2d.hpp"
#include "shape2d.hpp"
#include <math.h>
#include <iostream>
#include <memory>
//#include <sort.h>

using namespace Eigen;
using namespace std;

array<double, 2> polarEquation(Vector2d v0, Vector2d v1){
    array<double, 3> line={v0[1]-v1[1], v1[0]-v0[0],-(v0[0]*v1[1]-v1[0]*v0[1])};

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
	    lineIntervals.push_back(li);
	    ++angleCount;

	    if(angleCount>1){
		lineIntervals[angleCount-2].SetAngleEnd(lineIntervals[angleCount-1].Point());
	    }
	}
	points[i].setAngleId(angleCount-1);
	anglePrev=angle;
	unsigned long int shapeId=points[i].ShapeId();
	shapes[shapeId].addPoint(points[i]);
	if(shapes[shapeId].Complete() && shapeId!=0){
	    shapes[shapeId].arrange(lineIntervals);
	}
    }

    lineIntervals[angleCount-1].SetAngleEnd(lineIntervals[0].Point());
}


void Plane2d::FindPaths(){
    shapes[0].setVerticesClockwise(0);
    shapes[0].calculatePathsTarget(lineIntervals);
    for(unsigned long int i=1; i<shapes.size(); i++){
	shapes[i].calculatePaths(lineIntervals);
    }

    /*for(unsigned long int i=0; i<lineIntervals.size(); i++){
	cout << "-------------" << endl;
	cout << lineIntervals[i].UpperBound() << endl;
	cout << lineIntervals[i].LowerBound() << endl;
    }*/
}

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

double LineInterval::DistAt(array<double, 2> edge, int side){
    double angle=(side==0 ? angleStart : angleEnd);
    return edge[0]/cos(angle-edge[1]);
}

bool LineInterval::containsNormal(array<double, 2> edge){
    double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);   
    return edge[1] >= angleStart && edge[1] <= intervalAngleEnd;
}

array<double, 3> LineInterval::FunctionsAt(array<double, 2> edge, int side){
    double angle=(side==0 ? angleStart : angleEnd);
    double dist=edge[0]/cos(angle-edge[1]);
    double distPrime=tan(angle-edge[1])*dist;
   
    double angleSin=sin(angle-edge[1]);
    double angleCos=cos(angle-edge[1]);

    double distPrime2=(1+angleSin*angleSin)/(angleCos*angleCos*angleCos);
    return {dist, distPrime, distPrime2};
}

double LineInterval::ApproxRoot(double distStart, double distEnd, double derivStart, double derivEnd){
    double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);   

    double bStart=distStart - (angleStart * derivStart);
    double bEnd=distEnd - (intervalAngleEnd * derivEnd);

    Matrix2d A;
    A << derivStart, -1,   derivEnd, -1;
    Vector2d b;
    b << -bStart, -bEnd;

    return A.colPivHouseholderQr().solve(b)[1];
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

double LineInterval::LowerBound(){
    return distLowerBound;
}

double LineInterval::UpperBound(){
    return distUpperBound;
}

Vector2d LineInterval::Point(){
    return point;
}

//void assignAngleIds()


