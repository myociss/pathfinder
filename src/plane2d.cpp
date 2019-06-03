#include "plane2d.hpp"
#include "shape2d.hpp"
#include "line_interval.hpp"
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
	Shape2d shape(i, numVertices, _shapes[i].Weight());
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


void Plane2d::FindPaths(double distBound){
    CalcLineIntervalsInit();

    /*
    //cout << searchAgain.size() << endl;
    bool distBoundSatisfied=DivideCandidateIntervals(distBound);

    while(!distBoundSatisfied){

	double minUpperBound=candidateIntervals[0].UpperBound();
	for(unsigned long int i=0; i<candidateIntervals.size(); ++i){
	    double upperBound=candidateIntervals[i].UpperBound();
	    if(upperBound<minUpperBound){
		minUpperBound=upperBound;
	    }
	}
	vector<LineInterval> newCandidateIntervals;
	for(unsigned long int i=0; i<candidateIntervals.size(); ++i){
	    if(candidateIntervals[i].LowerBound()<minUpperBound){
		newCandidateIntervals.push_back(lineIntervals[i]);
	    }
	}
	candidateIntervals=newCandidateIntervals;
	
	distBoundSatisfied=DivideCandidateIntervals(distBound);
    }*/
    
    /*vector<array<Vector2d, 3>> foundPaths;
    foundPaths.reserve(candidateIntervals.size());
    for(unsigned long int i=0; i<candidateIntervals.size(); ++i){
	LineInterval li=candidateIntervals[i];
	Vector2d bounds(li.LowerBound(), li.UpperBound());
	array<Vector2d, 2> endPoints=li.EndPoints();
	foundPaths.push_back({bounds, endPoints[0], endPoints[1]});
    }
    return foundPaths;*/
}

/*
-get maximum width of all candidate intervals at outer faces
-while maximum width is too large:
  -get new vector of candidate intervals
  -prune vector
  -get new maximum width of new candidate vector at outer faces

to get new vector of candidate intervals:
-call a function on all line intervals that returns that line interval if its width is small enough, or two line intervals if division is required
*/

/*
bool Plane2d::DivideCandidateIntervals(double distBound){
    vector<LineInterval> newCandidateIntervals;
    for(unsigned long int i=0; i<candidateIntervals.size(); ++i){
	if(candidateIntervals[i].MaxWidth() <= distBound){
	    newCandidateIntervals.push_back(candidateIntervals[i]);
	} else{
	    array<double, 3> newAngles=candidateIntervals[i].Divide();
	    LineInterval li0 = LineInterval(newAngles[0], newAngles[1]);
	    LineInterval li1 = LineInterval(newAngles[1], newAngles[2]);
	    vector<unsigned long int> shapeIds=candidateIntervals[i].ShapeIds();
	    shapes[0].calculateIntervalTarget(li0);
	    shapes[0].calculateIntervalTarget(li1);
	    for(unsigned long int i=1; i<shapeIds.size(); ++i){
		shapes[i].calculateInterval(li0);
		shapes[i].calculateInterval(li1);
	    }
	    newCandidateIntervals.push_back(li0);
	    newCandidateIntervals.push_back(li1);
	}
    }
    bool distBoundSatisfied=(candidateIntervals.size()==newCandidateIntervals.size());
    candidateIntervals=newCandidateIntervals;
    return distBoundSatisfied;
}*/

void Plane2d::CalcLineIntervalsInit(){
    shapes[0].setVerticesClockwise(0);
    shapes[0].calculatePathsTarget(lineIntervals);
    for(unsigned long int i=1; i<shapes.size(); ++i){
	shapes[i].calculatePaths(lineIntervals);
    }

    minUpperBound=lineIntervals[0].UpperBound();
    for(unsigned long int i=0; i<lineIntervals.size(); ++i){
	double upperBound=lineIntervals[i].UpperBound();
	if(upperBound<minUpperBound){
	    minUpperBound=upperBound;
	}
    }

    for(unsigned long int i=0; i<lineIntervals.size(); ++i){
	if(lineIntervals[i].LowerBound()<minUpperBound){
	    candidateIntervals.push_back(lineIntervals[i]);
	}
    }
}

double Plane2d::MinUpperBound(){
    return minUpperBound;
}

vector<LineInterval> Plane2d::CandidateIntervals(){
    return candidateIntervals;
}

vector<array<double, 2>> Plane2d::LineIntervalBounds(){
    vector<array<double, 2>> bounds;
    bounds.reserve(lineIntervals.size());
    for(unsigned long int i; i<lineIntervals.size(); ++i){
	bounds.push_back({lineIntervals[i].LowerBound(), lineIntervals[i].UpperBound()});
    }
    return bounds;
}

vector<Shape2d> Plane2d::Shapes(){
    return shapes;
}


vector<unsigned long int> Plane2d::IntervalShapeIds(unsigned long int intervalId){
    return lineIntervals[intervalId].ShapeIds();
}

