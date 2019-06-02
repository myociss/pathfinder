#define _USE_MATH_DEFINES

#include "mesh.hpp"
#include "plane2d.hpp"
//#include "plane3d.hpp"
#include <iostream>
#include <map>
#include <string>
#include <vector>
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

vector<vector<array<Vector2d, 3>>> Mesh::findPaths(vector<Plane3d> planes, double distBound){
    vector<int> tetsChecked;
    tetsChecked.reserve(tetrahedrons.size());
    for(unsigned long int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(-1);
    }
    vector<vector<array<Vector2d, 3>>> ret;
    for(int i=0; i<planes.size(); i++){
	vector<Shape3d> myslice = slice(planes[i], tetsChecked);
	Plane2d plane2d(myslice, planes[i]);
	ret.push_back(plane2d.FindPaths(distBound));
    }
    return ret;
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


void Mesh::shortestPaths(const int epsilon, const int numThreads, double distBound){
    //thread threads[numThreads];
    future<vector<vector<array<Vector2d, 3>>>> futures[numThreads];

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
	//promise<vector<array<Vector2d, 3>>> p;
	//future<vector<array<Vector2d, 3>>> future=p.get_future();
	//threads[i]=thread(&Mesh::findPaths, this, planeVector, distBound, move(p));
	futures[i]=async(&Mesh::findPaths, this, planeVector, distBound);
    }

    for(int i=0; i<numThreads; i++){
	cout << i << endl;
	futures[i].get();
    }
}


unsigned long int Mesh::getTargetTetId(){
    return targetTetId;
}
