#include "plane2d.hpp"
#include "shape2d.hpp"
#include "line_interval.hpp"
#include <math.h>
#include <iostream>
#include <limits>
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
    cout << "step 0" << endl;
    cout << candidateIntervals.size() << endl;
    bool distBoundSatisfied=DivideCandidateIntervals(distBound);
    int i=1;
    while(!distBoundSatisfied){
	cout << i << endl;
	PruneCandidateIntervals();
	cout << candidateIntervals.size() << endl;
	distBoundSatisfied=DivideCandidateIntervals(distBound);
	++i;
	//distBoundSatisfied=true;
    }
    
}

void Plane2d::PruneCandidateIntervals(){
    minUpperBound=numeric_limits<double>::max();
    vector<LineInterval> newCandidateIntervals;
    for(unsigned long int i=0; i<candidateIntervals.size(); ++i){
	double upperBound=candidateIntervals[i].UpperBound();
	if(upperBound<minUpperBound){
	    minUpperBound=upperBound;
	}
    }

    for(unsigned long int i=0; i<candidateIntervals.size(); ++i){
	if(candidateIntervals[i].LowerBound()<minUpperBound){
	    newCandidateIntervals.push_back(candidateIntervals[i]);
	}
    }
    candidateIntervals = newCandidateIntervals;
}

bool Plane2d::DivideCandidateIntervals(double distBound){
    vector<LineInterval> newCandidateIntervals;
    bool distBoundSatisfied = true;
    for(unsigned long int i=0; i<candidateIntervals.size(); ++i){
	if(candidateIntervals[i].MaxWidth() <= distBound){
	    newCandidateIntervals.push_back(candidateIntervals[i]);
	} else{
	    /*cout << "-------------------" << endl;
	    cout << candidateIntervals[i].LowerBound() << endl;
	    cout << candidateIntervals[i].UpperBound() << endl;*/
	    distBoundSatisfied = false;
	    array<double, 3> newAngles=candidateIntervals[i].Divide();
	    LineInterval li0 = LineInterval(newAngles[0], newAngles[1]);
	    LineInterval li1 = LineInterval(newAngles[1], newAngles[2]);
	    //LineInterval li0 = LineInterval(newAngles[0], newAngles[2]);
	    //LineInterval li1 = LineInterval(newAngles[0], newAngles[2]);
	    vector<unsigned long int> shapeIds=candidateIntervals[i].ShapeIds();

	    shapes[0].calculateOneIntervalTarget(li0);
	    shapes[0].calculateOneIntervalTarget(li1);

	    for(unsigned long int j=1; j<shapeIds.size(); ++j){
		shapes[shapeIds[j]].calculateOneInterval(li0);
		shapes[shapeIds[j]].calculateOneInterval(li1);
	    }
	    /*cout << li0.LowerBound() << endl;
	    cout << li0.UpperBound() << endl;
	    cout << li1.LowerBound() << endl;
	    cout << li1.UpperBound() << endl;*/
	    newCandidateIntervals.push_back(li0);
	    newCandidateIntervals.push_back(li1);
	}
    }

    candidateIntervals=newCandidateIntervals;
    return distBoundSatisfied;
}

void Plane2d::CalcLineIntervalsInit(){
    shapes[0].setVerticesClockwise(0);
    shapes[0].calculateAllIntervalsTarget(lineIntervals);
    for(unsigned long int i=1; i<shapes.size(); ++i){
	shapes[i].calculateAllIntervals(lineIntervals);
    }

    minUpperBound=lineIntervals[0].UpperBound();
    //cout << minUpperBound << endl;
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

