#include "shape2d.hpp"
#include <math.h>
#include <iostream>
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
    unsigned long int startAngleId;
    unsigned long int endAngleId;

    /*cout << "start of arrange: " << endl;
    for(int i=0; i<numVertices; ++i){
	cout << vertices[i].Vec() << endl;
	cout << "----------" << endl;
    }*/

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
	    startVertex=i;
	}
	if(-sliPoint[1]*prev[0] + sliPoint[0]*prev[1]<0 && -sliPoint[1]*next[0] + sliPoint[0]*next[1]<0){
	    endAngleId=vertices[i].AngleId();
	}
    }
    Vector2d p = vertices[0].Vec();
    Vector2d q = vertices[1].Vec();
    Vector2d r = vertices[2].Vec();
    //cout << startVertex << endl;

    bool orientedClockwise=(r[0]-p[0])*(q[1]-p[1])-(q[0]-p[0])*(r[1]-p[1]) > 0;

    vector<Point2d> tmpVertices;
    tmpVertices.reserve(vertices.size());
    if(orientedClockwise){
	int idx=startVertex;
	//count=0;
	for(int i=0; i<vertices.size(); ++i){
	    tmpVertices.push_back(vertices[idx]);
	    ++idx;
	    if(idx==vertices.size()){
		idx=0;
	    }
	}

	/*while(idx!=startVertex){
	    int headIdx=idx-1;
	    //if(headIdx<0){
		//headIdx=vertices.size()-1;
	    //}
	    //Edge edge(vertices[headIdx], vertices[idx]);
	    tmpVertices.push_back(vertices[headIdx]);
	    //edges.push_back(edge);
	    headIdx=idx;
	    ++idx;
	    if(idx==vertices.size()){
		idx=0;
	    }
	    if(headIdx==vertices.size()){
		headIdx=0;
	    }
	}*/
    } else {
	int idx=startVertex;
	//count=0;
	for(int i=0; i<vertices.size(); ++i){
	    tmpVertices.push_back(vertices[idx]);
	    --idx;
	    if(idx<0){
		idx=vertices.size()-1;
	    }
	}
	/*int idx=startVertex-1;
	while(idx!=startVertex){
	    int headIdx=idx+1;
	    //if(headIdx==vertices.size()){
		//headIdx=0;
	    //}
	    //Edge edge(vertices[headIdx], vertices[idx]);
	    //edges.push_back(edge);
	    tmpVertices.push_back(vertices[headIdx]);
	    headIdx=idx;
	    --idx;
	    if(idx<0){
		idx=vertices.size()-1;
	    }
	    //if(headIdx<0){
		//headIdx=vertices.size()-1;
	    //}
	}*/
    }
    /*for(int i=0; i<edges.size(); ++i){
	int next=i+1;
	if(next==edges.size()){
	    next=0;
	}
	edges[i].setNext(edges[next]);
    }*/
    //cout << "end of arrange: " << endl;
    for(int i=0; i<numVertices; ++i){
	vertices[i]=tmpVertices[i];
	//cout << vertices[i].Vec() << endl;
	//cout << "----------" << endl;
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

//vector<Vector2d> Shape2d::VerticesEdgeOrder(){
//    vector<Vector2d> vecs;
//}

/*Edge::Edge(Point2d& _head, Point2d& _tail){
    head = _head;
    tail = _tail;
}

void Edge::setNext(Edge& _next){
    next = _next;
}

void Edge::store(double dist, double distPrime, double distPrime2){
    storedValues[0] = dist;
    storedValues[1] = distPrime;
    storedValues[2] = distPrime2;
}

array<double, 3> Edge::retrieve(){
    return storedValues;
}*/

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
