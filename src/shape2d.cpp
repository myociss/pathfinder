#include "shape2d.hpp"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

//#include <sort.h>

using namespace Eigen;
using namespace std;

Shape2d::Shape2d(int _numVertices){
    numVertices=_numVertices;
    vertices.reserve(numVertices);
}

bool Shape2d::Complete(){
    return numVertices==vertices.size();
}

void Shape2d::addPoint(Point2d point){
    int vectorPos = point.ShapeVectorPos();
    if(vertices.size()==0 || vectorPos > vertices.back().ShapeVectorPos()){
	vertices.push_back(point);
    } else if(vertices[0].ShapeVectorPos() > vectorPos){
	vertices.insert(vertices.begin(), point);
    } else {
	for(int i=0; i<vertices.size()-1; i++){
	    if(vectorPos>vertices[i].ShapeVectorPos() && vectorPos<vertices[i+1].ShapeVectorPos()){
		vertices.insert(vertices.begin()+i+1, point);
		break;
	    }
	}
    }
}

void Shape2d::arrange(vector<SweepLineInterval>& sweeplineIntervals){

    int startVertex;
    for(int i=0; i<vertices.size(); i++){
	Vector2d prev;
	if(i==0){
	    prev=vertices.back().Vec();
	} else {
	    prev=vertices[i-1].Vec();
	}

	Vector2d next;
	if(i==vertices.size()-1){
	    next=vertices[0].Vec();
	} else{
	    next=vertices[i+1].Vec();
	}

	SweepLineInterval sli=sweeplineIntervals[vertices[i].AngleId()];
	Vector2d sliPoint=sli.Point();
	if(-sliPoint[1]*prev[0] + sliPoint[0]*prev[1]>0 && -sliPoint[1]*next[0] + sliPoint[0]*next[1]>0){
	    startAngleId=vertices[i].AngleId();
	}
    }
    Vector2d p = vertices[0].Vec();
    Vector2d q = vertices[1].Vec();
    Vector2d r = vertices[2].Vec();

    bool orientedClockwise=(r[0]-p[0])*(q[1]-p[1])-(q[0]-p[0])*(r[1]-p[1]) > 0;

    vector<Point2d> tmpVertices;
    tmpVertices.reserve(vertices.size());
    if(orientedClockwise){
	int idx=startVertex;
	for(int i=0; i<vertices.size(); ++i){
	    tmpVertices.push_back(vertices[idx]);
	    ++idx;
	    if(idx==vertices.size()){
		idx=0;
	    }
	}
    } else {
	int idx=startVertex;
	for(int i=0; i<vertices.size(); ++i){
	    tmpVertices.push_back(vertices[idx]);
	    --idx;
	    if(idx<0){
		idx=vertices.size()-1;
	    }
	}
    }

    setNewVertices(tmpVertices);
}

void Shape2d::setNewVertices(vector<Point2d> tmpVertices){
    endVertexId=tmpVertices[0].AngleId();
    double angleMax=tmpVertices[0].Angle();
    for(int i=0; i<numVertices; ++i){
	vertices[i]=tmpVertices[i];
	if(vertices[i].Angle()<0 && vertices[0].Angle()>0){
	    vertices[i].updateAngle();
	}
	if(vertices[i].Angle()>angleMax){
	    endVertexId=vertices[i].AngleId();
	}
    }
}

vector<Vector2d> Shape2d::Vertices(){
    vector<Vector2d> vecs;
    for(int i=0; i<vertices.size(); ++i){
	for(int j=0; j<vertices.size(); ++j){
	    if(vertices[j].ShapeVectorPos()==i){
		vecs.push_back(vertices[j].Vec());
		break;
	    }
	}
    }
    return vecs;
}

vector<Vector2d> Shape2d::VerticesArranged(){
    vector<Vector2d> vecs;
    for(int i=0; i<vertices.size(); ++i){
	vecs.push_back(vertices[i].Vec());
    }
    return vecs;
}

void Shape2d::calculatePaths(vector<SweepLineInterval> sweepLineIntervals){
    unsigned long int intervalStartIdx=vertices[0].AngleId();
    int entryEdgeIdx=0;
    int terminalEdgeIdx=vertices.size()-1;
    unsigned long int startIdx=intervalStartIdx;

    array<double, 3> entryEdgeStartVals;
    array<double, 3> terminalStartEdgeVals;

    array<double, 2> entryEdgePolar=polarEquation(vertices[0].Vec(), vertices[1].Vec());
    array<double, 2> terminalEdgePolar=polarEquation(vertices[0].Vec(), vertices[terminalEdgeIdx].Vec());

    while(startIdx!=endVertexId){
	SweepLineInterval intervalStart=sweepLineIntervals[startIdx];
	int endIdx = startIdx == sweepLineIntervals.size()-1 ? 0 : idx+1;
	SweepLineInterval intervalEnd=sweepLineIntervals[endIdx];

	if(vertices[startEdgeIdx].AngleId()==startIdx){
	    entryEdgeStartVals=intervalStart.FunctionsAt(entryEdgePolar);
	}

	if(vertices[(terminalEdgeIdx+1)%vertices.size()].AngleId()==startIdx){
	    terminalEdgeStartVals=intervalStart.FunctionsAt(terminalEdgePolar);
	}
	
	array<double, 3> entryEdgeEndVals=intervalEnd.FunctionsAt(entryEdgePolar);
	array<double, 3> terminalEdgeEndVals=intervalEnd.FunctionsAt(terminalEdgePolar);

	double startDist=terminalEdgeEndVals[0]-terminalEdgeEndVals[0];
	double startDeriv=terminalEdgeEndVals[1]-terminalEdgeEndVals[1];
	double startDeriv2=terminalEdgeEndVals[2]-terminalEdgeEndVals[2];

	

	if(vertices[terminalEdgeIdx].AngleId()==endIdx){
	    --terminalEdgeIdx;
	    terminalEdgePolar=polarEquation(vertices[terminalEdgeIdx+1].Vec(), vertices[terminalEdgeIdx].Vec());
	}
	if(vertices[startEdgeIdx+1].AngleId()==endIdx){
	    ++startEdgeIdx;
	    entryEdgePolar=polarEquation(vertices[startEdgeIdx-1].Vec(), vertices[startEdgeIdx].Vec());
	}
	

	startIdx=endIdx;
    }
}

Point2d::Point2d(Vector2d _vec, unsigned long int _shapeId, int _shapeVectorPos){
    vec = _vec;
    shapeId = _shapeId;
    shapeVectorPos = _shapeVectorPos;
    angle = atan2(vec[1], vec[0]);
}

double Point2d::Angle(){
    return angle;
}

unsigned long int Point2d::AngleId(){
    return angleId;
}

void Point2d::updateAngle(){
    angle += 2 * M_PI;
}

unsigned long int Point2d::ShapeId(){
    return shapeId;
}

int Point2d::ShapeVectorPos(){
    return shapeVectorPos;
}

void Point2d::setAngleId(unsigned long int _angleId){
    angleId = _angleId;
}

Vector2d Point2d::Vec(){
    return vec;
}
