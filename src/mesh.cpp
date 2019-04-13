#define _USE_MATH_DEFINES

#include "mesh.hpp"
//#include "plane3d.hpp"
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <functional>
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

void Mesh::setVertices(const vector<array<float, 3>> & _vertices){

    for(unsigned long int i=0; i < _vertices.size(); ++i){
	Vertex3d pt(_vertices[i]);
	vertices.push_back(pt);
    }

}


void Mesh::addTetrahedron(const int id, const array<int, 4> vertexIds, 
	const vector<unsigned long int> neighborIds, const float weight){

    vector<reference_wrapper<Vertex3d>> refs = {vertices[vertexIds[0]], vertices[vertexIds[1]], vertices[vertexIds[2]], vertices[vertexIds[3]]};

    Tetrahedron tet(id, refs, weight);

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

bool Mesh::setTarget(array<float, 3> _target){
    for(unsigned long int i=0; i < tetrahedrons.size(); ++i){
	if (tetrahedrons[i].contains(_target)){
	    Vector3f t(_target[0], _target[1], _target[2]);
	    target = t;
	    targetTetId=i;
	    return true;
	}
    }
    return false;
}

vector<vector<array<float, 3>>> Mesh::sliceIndv(array<float, 2> rotation, bool test){
    vector<int> tetsChecked;
    tetsChecked.reserve(tetrahedrons.size());
    for(unsigned long int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(-1);
    }

    Plane3d plane(0, rotation[0], rotation[1], target);
    return slice(plane, tetsChecked, test);
}

void Mesh::findPaths(vector<Plane3d> planes){
    vector<int> tetsChecked;
    tetsChecked.reserve(tetrahedrons.size());
    for(unsigned long int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(-1);
    }
    for(int i=0; i<planes.size(); i++){
	vector<vector<array<float, 3>>> myslice = slice(planes[i], tetsChecked, false);
	cout << myslice.size() << endl;
    }
	
}

vector<vector<array<float, 3>>> Mesh::slice(Plane3d plane, vector<int> &tetsChecked, bool test){
    vector<vector<array<float, 3>>> slice = computeSliceComponent(plane, tetsChecked, targetTetId, test);

    for(unsigned long int i=0; i<faces.size(); i++){
	if(faces[i].intersectsPlane(plane) && tetsChecked[faces[i].TetId()]!=plane.Id()){
	    vector<vector<array<float, 3>>> sliceComponent = computeSliceComponent(plane, tetsChecked, faces[i].TetId(), test);
	    slice.insert(slice.end(), sliceComponent.begin(), sliceComponent.end());
	}
    }
    return slice;
}

vector<vector<array<float, 3>>> Mesh::computeSliceComponent(Plane3d plane, vector<int> &tetsChecked, unsigned long int initTet, bool test){
    
    vector<unsigned long int> tetStack;
    tetStack.push_back(initTet);

    vector<vector<array<float, 3>>> allPoints;

    if(test){
	sliceIds.clear();
    }

    while(tetStack.size() > 0){
	Tetrahedron& tet = tetrahedrons[tetStack.back()];
	tetStack.pop_back();
	
	if(tetsChecked[tet.Id()]!=plane.Id()){

	    vector<array<float, 3>> intersectionPoints = tet.intersectsPlane(plane);

	    if(intersectionPoints.size()>2){
		if(test){
		    sliceIds.push_back(tet.Id());
		}
		allPoints.push_back(intersectionPoints);
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


void Mesh::shortestPaths(const int epsilon, const int numThreads){
    thread t[numThreads];
    vector<Plane3d> planes;

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
	t[i]=thread(&Mesh::findPaths, this, planeVector);
    }

    for(int i=0; i<numThreads; i++){
	cout << i << endl;
	t[i].join();
    }
}

vector<int> Mesh::getSliceIds(){
    return sliceIds;
}

unsigned long int Mesh::getTargetTetId(){
    return targetTetId;
}
