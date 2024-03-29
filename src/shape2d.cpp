#include "shape2d.hpp"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

//#include <sort.h>

using namespace Eigen;
using namespace std;

Shape2d::Shape2d(unsigned long int _id, int _numVertices, double _weight){
    id=_id;
    numVertices=_numVertices;
    weight=_weight;
    vertices.reserve(numVertices);
    endVertex=0;
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
	if(-liPoint[1]*prev[0] + liPoint[0]*prev[1]>0 && -liPoint[1]*next[0] + liPoint[0]*next[1]>=0){
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
    double angleMax=tmpVertices[0].Angle();
    for(int i=0; i<numVertices; ++i){
	vertices[i]=tmpVertices[i];
	double vertexAngle=vertices[i].Angle();
	if(vertices[i].Angle()<0 && vertices[0].Angle()>0){
	    vertexAngle += 2 * M_PI;
	}
	if(vertexAngle>angleMax){
	    endVertex=i;
	    angleMax=vertexAngle;
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

void Shape2d::calculateOneIntervalTarget(LineInterval& li){
    for(int i=vertices.size()-1; i>=0; --i){
	int next=i-1;
	if(next<0){
	    next=vertices.size()-1;
	}
	if(li.IntersectsEdge(vertices[i].Angle(), vertices[next].Angle())){
	    array<double, 2> edgePolar=polarEquation(vertices[i].Vec(), vertices[next].Vec());
	    computeBoundsTarget(li, edgePolar);
	}
    }
}

void Shape2d::calculateOneInterval(LineInterval& li){
    array<double, 2> entryPolar=EntryEdge(li);
    array<double, 2> terminalPolar=TerminalEdge(li);

    array<double, 3> entryFStart=li.FunctionsAt(entryPolar, 0);
    array<double, 3> terminalFStart=li.FunctionsAt(terminalPolar, 0);
    array<double, 3> entryFEnd=li.FunctionsAt(entryPolar, 1);
    array<double, 3> terminalFEnd=li.FunctionsAt(terminalPolar, 1);

    computeBounds(entryFStart, entryFEnd, terminalFStart, terminalFEnd, li);
}

void Shape2d::calculateAllIntervalsTarget(vector<LineInterval>& lineIntervals){
    for(int i=vertices.size()-1; i>=0; --i){
	int next=i-1;
	if(next<0){
	    next=vertices.size()-1;
	}

	unsigned long int startIntervalId=vertices[i].AngleId();
	unsigned long int intervalId=startIntervalId;

	//double upperBound=0.0;
	//double lowerBound=0.0;

	array<double, 2> edgePolar=polarEquation(vertices[i].Vec(), vertices[next].Vec());
	while(intervalId!=vertices[next].AngleId()){

	    LineInterval& li=lineIntervals[intervalId];
	    /*double startSide=li.DistAt(edgePolar, 0);
	    double endSide=li.DistAt(edgePolar, 1);

	    if(li.containsNormal(edgePolar)){
		upperBound=weight * max(startSide, endSide);
		lowerBound=weight * edgePolar[0];
	    } else {
		upperBound=weight * max(startSide, endSide);
		lowerBound=weight * min(startSide, endSide);
	    }
	    li.update(upperBound, lowerBound, startSide, endSide, id);*/
	    computeBoundsTarget(li, edgePolar);

	    intervalId=(intervalId+1)%lineIntervals.size();
	}
    }
}

void Shape2d::calculateAllIntervals(vector<LineInterval>& lineIntervals){
    unsigned long int startIntervalId=vertices[0].AngleId();
    unsigned long int intervalId=startIntervalId;
    unsigned long int endVertexIntervalId=vertices[endVertex].AngleId();

    int entryEdge=(vertices[0].AngleId()==vertices[1].AngleId() ? 1 : 0);
    if(entryEdge==1){
	cout << "guess this happens" << endl;
    }
    int terminalEdge=vertices.size()-1;


    array<double, 3> entryFStart;
    array<double, 3> terminalFStart;

    array<double, 2> entryPolar=polarEquation(vertices[entryEdge].Vec(), vertices[entryEdge+1].Vec());
    array<double, 2> terminalPolar=polarEquation(vertices[0].Vec(), vertices[terminalEdge].Vec());

    //cout << endVertexIntervalId << endl;

    while(intervalId!=endVertexIntervalId){
	//cout << intervalId << endl;
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

	computeBounds(entryFStart, entryFEnd, terminalFStart, terminalFEnd, li);

	terminalFStart=terminalFEnd;
	entryFStart=entryFEnd;
	

	unsigned long int nextInterval = (intervalId+1)%lineIntervals.size();
	if(vertices[terminalEdge].AngleId()==nextInterval){
	    --terminalEdge;
	    terminalPolar=polarEquation(vertices[terminalEdge+1].Vec(), vertices[terminalEdge].Vec());
	}
	if(vertices[entryEdge+1].AngleId()==nextInterval){
	    ++entryEdge;
	    entryPolar=polarEquation(vertices[entryEdge].Vec(), vertices[entryEdge+1].Vec());
	}
	

	intervalId=nextInterval;
    }
}

void Shape2d::computeBoundsTarget(LineInterval& li, array<double, 2> edgePolar){
    double startSide=li.DistAt(edgePolar, 0);
    double endSide=li.DistAt(edgePolar, 1);
    double upperBound=0.0;
    double lowerBound=0.0;

    if(li.containsNormal(edgePolar)){
	upperBound=weight * max(startSide, endSide);
	lowerBound=weight * edgePolar[0];
    } else {
	upperBound=weight * max(startSide, endSide);
	lowerBound=weight * min(startSide, endSide);
    }
    li.update(upperBound, lowerBound, startSide, endSide, id);
}

void Shape2d::computeBounds(array<double, 3> entryFStart, array<double, 3> entryFEnd, array<double, 3> terminalFStart, array<double, 3> terminalFEnd, LineInterval& li){

    double startDist=max(terminalFStart[0]-entryFStart[0], 0.0);
    double startDeriv=terminalFStart[1]-entryFStart[1];
    double startDeriv2=terminalFStart[2]-entryFStart[2];


    double endDist=max(terminalFEnd[0]-entryFEnd[0], 0.0);
    double endDeriv=terminalFEnd[1]-entryFEnd[1];
    double endDeriv2=terminalFEnd[2]-entryFEnd[2];

    double maxSide=max(startDist, endDist);
    double minSide=min(startDist, endDist);

    double upperBound = 0.0;
    double lowerBound = 0.0;

    if(startDeriv2*endDeriv2 <= 0 || startDeriv*endDeriv <= 0){
    //if(startDeriv*endDeriv <= 0){
	upperBound=weight * maxDist();
    } else if(startDeriv*endDeriv <= 0){
	double root=li.ApproxRoot(startDist, endDist, startDeriv, endDeriv);
	if(root < 0){
	    root=0.0;
	}

	if(startDeriv<0 || (startDeriv==0 && endDeriv>0)){
	    //cout << "root lower bound" << endl;
	    upperBound=weight*maxSide;
	    lowerBound=weight*root;
	} else{
	    /*cout << "root upper bound" << endl;
	    cout << maxDist() << endl;
	    cout << root << endl;
	    cout << maxSide << endl;*/
	    //upperBound=weight*maxDist();
	    upperBound=weight*min(maxDist(), root);
	    lowerBound=weight*minSide;
	}
    } else {
	upperBound=weight*maxSide;
	lowerBound=weight*minSide;
    }

    li.update(upperBound, lowerBound, terminalFStart[0], terminalFEnd[0], id);
}

array<double, 2> Shape2d::EntryEdge(LineInterval& li){
    array<double, 2> entryEdgePolar;

    for(int i=1; i<=endVertex; ++i){
	if(li.IntersectsEdge(vertices[i-1].Angle(), vertices[i].Angle())){
	    entryEdgePolar=polarEquation(vertices[i-1].Vec(), vertices[i].Vec());
	    break;
	}
    }
    return entryEdgePolar;
}

array<double, 2> Shape2d::TerminalEdge(LineInterval& li){
    array<double, 2> terminalEdgePolar;

    for(int i=endVertex; i<vertices.size(); ++i){
	int next=i+1;
	if(next==vertices.size()){
	    next=0;
	}
	
	if(li.IntersectsEdge(vertices[next].Angle(), vertices[i].Angle())){
	    terminalEdgePolar=polarEquation(vertices[next].Vec(), vertices[i].Vec());
	    break;
	}
    }
    return terminalEdgePolar;
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
