#define _USE_MATH_DEFINES

#include "mesh.hpp"
#include "plane2d.hpp"
//#include "plane3d.hpp"
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <limits>
#include <functional>
#include <future>
#include <cmath>
#include <thread>

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

Mesh::Mesh(const int numVertices, const int numFaces, const int numCells){
    vertices.reserve(numVertices);
    faces.reserve(numFaces);
    tetrahedrons.reserve(numCells);
}

void Mesh::setVertices(const vector<array<double, 3>> & _vertices){

    for(unsigned long int i=0; i < _vertices.size(); ++i){
	Vertex3d pt(_vertices[i]);
	vertices.push_back(pt);
    }

}


void Mesh::addTetrahedron(const int id, const array<int, 4> vertexIds, 
	const vector<unsigned long int> neighborIds, const double weight, const int label){

    vector<reference_wrapper<Vertex3d>> refs = {vertices[vertexIds[0]], vertices[vertexIds[1]], vertices[vertexIds[2]], vertices[vertexIds[3]]};

    Tetrahedron tet(id, refs, weight, label);

    unsigned long int current_size = tetrahedrons.size();

    for(unsigned long int i=0; i<neighborIds.size(); ++i){
	if(neighborIds[i]<current_size){
	    Tetrahedron& neighbor = tetrahedrons[neighborIds[i]];
	    
	    neighbor.addNeighbor(tet.Id());
	    tet.addNeighbor(neighbor.Id());
	}
    }
    tetrahedrons.push_back(tet);
}

void Mesh::addFace(const array<int, 3> vertexIds, const int tetId){
    vector<reference_wrapper<Vertex3d>> refs = {vertices[vertexIds[0]], vertices[vertexIds[1]], vertices[vertexIds[2]]};

    Face face(refs, tetId);
    faces.push_back(face);
}

bool Mesh::setTarget(array<double, 3> _target){
    bool targetSet=false;
    for(unsigned long int i=0; i < tetrahedrons.size(); ++i){
	if (tetrahedrons[i].contains(_target)){
	    Vector3d t(_target[0], _target[1], _target[2]);
	    target = t;
	    targetTetId=i;
	    targetSet=true;
	    break;
	}
    }
    return targetSet;
}

vector<Shape3d> Mesh::sliceIndv(array<double, 2> rotation){
    vector<int> tetsChecked;
    tetsChecked.reserve(tetrahedrons.size());
    for(unsigned long int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(-1);
    }

    Plane3d plane(0, rotation[0], rotation[1], target);
    return slice(plane, tetsChecked);
}

vector<FoundPath> Mesh::findPaths(vector<Plane3d> planes, double distBound){
    vector<int> tetsChecked;
    tetsChecked.reserve(tetrahedrons.size());
    for(unsigned long int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(-1);
    }

    double minUpperBound=numeric_limits<double>::max();

    vector<vector<LineInterval>> allCandidateIntervals;

    for(int i=0; i<planes.size(); ++i){
	vector<Shape3d> myslice = slice(planes[i], tetsChecked);
	Plane2d plane2d(myslice, planes[i]);
	plane2d.FindPaths(distBound);
	allCandidateIntervals.push_back(plane2d.CandidateIntervals());
	if(plane2d.MinUpperBound()<minUpperBound){
	    minUpperBound=plane2d.MinUpperBound();
	}
    }
    vector<FoundPath> foundPaths;

    for(int i=0; i<planes.size(); ++i){
	vector<LineInterval> candidateIntervals=allCandidateIntervals[i];

	for(int j=0; j<candidateIntervals.size(); ++j){
	    LineInterval candidateInterval=candidateIntervals[j];
	    if(candidateInterval.LowerBound()<minUpperBound){
		array<Vector2d, 2> endPoints=candidateInterval.EndPoints();
		Vector3d pt0=planes[i].Get3dPoint(endPoints[0]);
		Vector3d pt1=planes[i].Get3dPoint(endPoints[1]);

		FoundPath foundPath(planes[i].Id(), pt0, pt1, candidateInterval.LowerBound(), candidateInterval.UpperBound());
		foundPaths.push_back(foundPath);
	    }
	}
    }
    return foundPaths;
}

vector<Shape3d> Mesh::slice(Plane3d plane, vector<int> &tetsChecked){
    vector<Shape3d> slice = computeSliceComponent(plane, tetsChecked, targetTetId);

    for(unsigned long int i=0; i<faces.size(); i++){
	if(faces[i].intersectsPlane(plane) && tetsChecked[faces[i].TetId()]!=plane.Id()){
	    vector<Shape3d> sliceComponent = computeSliceComponent(plane, tetsChecked, faces[i].TetId());
	    slice.insert(slice.end(), sliceComponent.begin(), sliceComponent.end());
	}
    }
    return slice;
}

vector<Shape3d> Mesh::computeSliceComponent(Plane3d plane, vector<int> &tetsChecked, unsigned long int initTet){
    
    vector<unsigned long int> tetStack;
    tetStack.push_back(initTet);

    vector<Shape3d> allPoints;

    while(tetStack.size() > 0){
	Tetrahedron& tet = tetrahedrons[tetStack.back()];
	tetStack.pop_back();
	
	if(tetsChecked[tet.Id()]!=plane.Id()){

	    vector<array<double, 3>> intersectionPoints = tet.intersectsPlane(plane);

	    if(intersectionPoints.size()>2){
		Shape3d shape(tet.Id(), intersectionPoints, tet.Weight(), tet.Label());
		allPoints.push_back(shape);
		vector<unsigned long int> neighbors = tet.Neighbors();

		for(int i=0; i<neighbors.size();++i){
		    tetStack.push_back(neighbors[i]);
		}
	    }

	    tetsChecked[tet.Id()]=plane.Id();
	}
    }

    return allPoints;
}


vector<FoundPath> Mesh::shortestPaths(const int epsilon, const int numThreads, double distBound){

    future<vector<FoundPath>> futures[numThreads];

    vector<Plane3d> planes;
    cout << numThreads << endl;

    for(int i=0; i<epsilon; i++){
	for(int j=0; j<epsilon; j++){
	    Plane3d plane((i*epsilon)+j, i*M_PI/epsilon, j*M_PI/epsilon, target);
	    planes.push_back(plane);
	}
    }

    int subarraySize = ((epsilon * epsilon) + numThreads - 1) / numThreads;

    for(int i=0; i<numThreads; i++){
	int start=i*subarraySize;
	int end=start+subarraySize;
	if(end>(epsilon*epsilon)){
	    end=epsilon*epsilon;
	}

	vector<Plane3d> planeVector;

	for(int idx=start; idx<end; idx++){
	    planeVector.push_back(planes[idx]);
	}

	futures[i]=async(&Mesh::findPaths, this, planeVector, distBound);
    }

    vector<FoundPath> allFoundPaths;
    double minUpperBound=numeric_limits<double>::max();
    for(unsigned long int i=0; i<numThreads; ++i){
	cout << i << endl;
	vector<FoundPath> foundPaths=futures[i].get();
	for(unsigned long int j=0; j<foundPaths.size(); ++j){
	    if(foundPaths[j].UpperBound()<minUpperBound){
		minUpperBound=foundPaths[j].UpperBound();
	    }
	    allFoundPaths.push_back(foundPaths[j]);
	}
    }
    
    vector<FoundPath> ret;
    for(unsigned long int i=0; i<allFoundPaths.size(); ++i){
	if(allFoundPaths[i].LowerBound()<minUpperBound){
	    ret.push_back(allFoundPaths[i]);
	}
    }
    cout << ret.size() << endl;
    return ret;
}


unsigned long int Mesh::getTargetTetId(){
    return targetTetId;
}

FoundPath::FoundPath(unsigned long int _planeId, Vector3d _pt0, Vector3d _pt1, double _lowerBound, double _upperBound){
    planeId=_planeId;
    pt0=_pt0;
    pt1=_pt1;
    lowerBound=_lowerBound;
    upperBound=_upperBound;
}

unsigned long int FoundPath::PlaneId(){
    return planeId;
}

double FoundPath::UpperBound(){
    return upperBound;
}

double FoundPath::LowerBound(){
    return lowerBound;
}

array<Vector3d, 2> FoundPath::Points(){
    return {pt0, pt1};
}

