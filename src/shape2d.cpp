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
	    startVertex=i;
	}
    }

    setVerticesClockwise(startVertex);
}

void Shape2d::setVerticesClockwise(int startVertex){
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
    endVertex=0;
    double angleMax=tmpVertices[0].Angle();
    for(int i=0; i<numVertices; ++i){
	vertices[i]=tmpVertices[i];
	if(vertices[i].Angle()<0 && vertices[0].Angle()>0){
	    vertices[i].updateAngle();
	}
	if(vertices[i].Angle()>angleMax){
	    endVertex=i;
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

void Shape2d::calculatePathsTarget(vector<LineInterval>& lineIntervals){
    for(int i=0; i<vertices.size(); i++){
	int next=(i+1) % vertices.size();
	unsigned long int startIntervalId=vertices[i].AngleId();
	unsigned long int intervalId=startIntervalId;

	array<double, 2> edgePolar=polarEquation(vertices[i].Vec(), vertices[next].Vec());
	while(intervalId!=vertices[next].AngleId()){
	    LineInterval& li=lineIntervals[intervalId];
	    double startSide=li.DistAt(edgePolar, 0);
	    double endSide=li.DistAt(edgePolar, 1);

	    if(li.containsNormal(edgePolar)){
		li.updateLowerBound(weight * edgePolar[0]);
		li.updateUpperBound(weight * max(startSide, endSide));
	    } else {
		li.updateLowerBound(weight * min(startSide, endSide));
		li.updateUpperBound(weight * max(startSide, endSide));
	    }

	    intervalId=(intervalId+1)%lineIntervals.size();
	}
    }
}

void Shape2d::calculatePaths(vector<LineInterval>& lineIntervals){
    unsigned long int startIntervalId=vertices[0].AngleId();
    unsigned long int intervalId=startIntervalId;

    int entryEdge=0;
    int terminalEdge=vertices.size()-1;


    array<double, 3> entryFStart;
    array<double, 3> terminalFStart;

    array<double, 2> entryPolar=polarEquation(vertices[0].Vec(), vertices[1].Vec());
    array<double, 2> terminalPolar=polarEquation(vertices[0].Vec(), vertices[terminalEdge].Vec());

    unsigned long int endVertexIntervalId=vertices[endVertex].AngleId();

    while(intervalId!=endVertexIntervalId){
	LineInterval& li = lineIntervals[intervalId];
	//new entry edge
	if(vertices[entryEdge].AngleId()==intervalId){
	    entryFStart=li.FunctionsAt(entryPolar, 0);
	}

	//new exit edge
	if(vertices[(terminalEdge+1)%vertices.size()].AngleId()==intervalId){
	    terminalFStart=li.FunctionsAt(terminalPolar, 0);
	}

	
	
	array<double, 3> entryFEnd=li.FunctionsAt(entryPolar, 1);
	array<double, 3> terminalFEnd=li.FunctionsAt(terminalPolar, 1);

	double startDist=max(terminalFStart[0]-entryFStart[0], 0.0);
	double startDeriv=terminalFStart[1]-entryFStart[1];
	double startDeriv2=terminalFStart[2]-entryFStart[2];


	double endDist=max(terminalFEnd[0]-entryFEnd[0], 0.0);
	double endDeriv=terminalFEnd[1]-entryFEnd[1];
	double endDeriv2=terminalFEnd[2]-entryFEnd[2];

	double maxSide=max(startDist, endDist);
	double minSide=min(startDist, endDist);

	//if the inflection point has changed
	if(startDeriv2*endDeriv2 < 0){
	    li.updateUpperBound(weight * maxDist());
	} /*else if(startDeriv*endDeriv < 0){
	    double root=li.ApproxRoot(startDist, endDist, startDeriv, endDeriv);
	    if(root < 0){
		root=0.0;
	    }

	    if(startDeriv<0){
		li.updateLowerBound(weight * min(root, minSide));
		li.updateUpperBound(weight * maxSide);
	    } else{
		li.updateUpperBound(weight * root);
		li.updateLowerBound(weight * minSide);
	    }
	}*/ else {
	    li.updateUpperBound(weight * maxSide);
	    li.updateLowerBound(weight * minSide);
	}

	terminalFStart=terminalFEnd;
	entryFStart=entryFEnd;
	

	unsigned long int nextInterval = (intervalId+1)%lineIntervals.size();
	if(vertices[terminalEdge].AngleId()==nextInterval){
	    --terminalEdge;
	    terminalPolar=polarEquation(vertices[terminalEdge+1].Vec(), vertices[terminalEdge].Vec());
	}
	if(vertices[entryEdge+1].AngleId()==nextInterval){
	    ++entryEdge;
	    entryPolar=polarEquation(vertices[entryEdge-1].Vec(), vertices[entryEdge].Vec());
	}
	

	intervalId=nextInterval;
    }
}

double Shape2d::maxDist(){
    double maxDist=0.0;
    for(int i=0; i<vertices.size(); ++i){
	for(int j=i+1; j<vertices.size();++j){
	    Vector2d v0=vertices[i].Vec();
	    Vector2d v1=vertices[j].Vec();
	    double dist=sqrt(pow(v0[0]-v1[0], 2.0) + pow(v0[1]-v1[1], 2.0));
	    if(maxDist<dist){
		maxDist=dist;
	    }
	}
    }
    return maxDist;
}

int Shape2d::EndVertex(){
    return endVertex;
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
