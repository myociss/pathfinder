#include "shape2d.hpp"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

//#include <sort.h>

using namespace Eigen;
using namespace std;

Shape2d::Shape2d(int _numVertices, double _weight){
    numVertices=_numVertices;
    weight=_weight;
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

void Shape2d::arrange(vector<LineInterval>& lineIntervals){

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

	LineInterval& li=lineIntervals[vertices[i].AngleId()];
	Vector2d liPoint=li.Point();
	if(-liPoint[1]*prev[0] + liPoint[0]*prev[1]>0 && -liPoint[1]*next[0] + liPoint[0]*next[1]>0){
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

void Shape2d::calculatePaths(vector<SweepLineInterval>& lineIntervals){
    unsigned long int startIntervalId=vertices[0].AngleId();

    int entryEdgeIdx=0;
    int terminalEdgeIdx=vertices.size()-1;

    unsigned long int intervalId=startIntervalId;

    array<double, 3> entryEdgeStartVals;
    array<double, 3> terminalStartEdgeVals;

    array<double, 2> entryEdgePolar=polarEquation(vertices[0].Vec(), vertices[1].Vec());
    array<double, 2> terminalEdgePolar=polarEquation(vertices[0].Vec(), vertices[terminalEdgeIdx].Vec());

    while(intervalId!=endVertexId){
	LineInterval& li = lineIntervals[intervalId];
	//new entry edge
	if(vertices[startEdgeIdx].AngleId()==intervalId){
	    entryEdgeStartVals=li.FunctionsAt(entryEdgePolar, ANGLE_START);
	}

	//new exit edge
	if(vertices[(terminalEdgeIdx+1)%vertices.size()].AngleId()==intervalId){
	    terminalEdgeStartVals=li.FunctionsAt(terminalEdgePolar, ANGLE_START);
	}

	
	
	array<double, 3> entryEdgeEndVals=li.FunctionsAt(entryEdgePolar, ANGLE_END);
	array<double, 3> terminalEdgeEndVals=li.FunctionsAt(terminalEdgePolar, ANGLE_END);

	double startDist=terminalEdgeStartVals[0]-entryEdgeStartVals[0];
	double startDeriv=terminalEdgeStartVals[1]-entryEdgeStartVals[1];
	double startDeriv2=terminalEdgeStartVals[2]-entryEdgeStartVals[2];


	double endDist=terminalEdgeEndVals[0]-entryEdgeEndVals[0];
	double endDeriv=terminalEdgeEndVals[1]-entryEdgeEndVals[1];
	double endDeriv2=terminalEdgeEndVals[2]-entryEdgeEndVals[2];

	//if the inflection point has changed
	if(startDeriv2*endDeriv2 < 0){
	    li.updateUpperBound(weight * maxDist());
	} else if(startDeriv*endDeriv < 0){

	} else {
	    li.updateUpperBound(max(startDist, endDist));
	    li.updateLowerBound(min(startDist, endDist));
	}
	

	unsigned long int nextInterval = (intervalId+1)%lineIntervals.size();
	if(vertices[terminalEdgeIdx].AngleId()==nextInterval){
	    --terminalEdgeIdx;
	    terminalEdgePolar=polarEquation(vertices[terminalEdgeIdx+1].Vec(), vertices[terminalEdgeIdx].Vec());
	}
	if(vertices[startEdgeIdx+1].AngleId()==nextInterval){
	    ++startEdgeIdx;
	    entryEdgePolar=polarEquation(vertices[startEdgeIdx-1].Vec(), vertices[startEdgeIdx].Vec());
	}
	

	intervalId=nextInterval;
    }
}

double Shape2d::maxDist(){
    double maxDist=0.0;
    for(int i=0; i<vertices.size(); ++i){
	for(int j=i+1; j<vertices.size();++j){
	    double dist=sqrt(pow(vertices[i][0]-vertices[j][0], 2.0) + pow(vertices[i][1]-vertices[j][1], 2.0));
	    if(maxDist<dist){
		maxDist=dist;
	    }
	}
    }
    return maxDist;
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
