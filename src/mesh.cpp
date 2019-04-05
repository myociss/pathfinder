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
//using namespace plane3d;

Mesh::Mesh(const int numVertices, const int numFaces, const int numCells){
    vertices.reserve(numVertices);
    //faces.reserve(numFaces);
    tetrahedrons.reserve(numCells);
}

void Mesh::setVertices(const vector<array<float, 3>> & _vertices){

    for(unsigned long int i=0; i < _vertices.size(); i++){
	//Vertex3d* pt = new Vertex3d(_vertices[i]);
	Vertex3d pt(_vertices[i]);
	
	vertices.push_back(pt);
	cout << pt.Vec() << endl;
    }

}


void Mesh::addTetrahedron(const int id, const array<int, 4> vertexIds, 
	const vector<int> neighborIds, const float weight){

    vector<reference_wrapper<Vertex3d>> refs = {vertices[vertexIds[0]], vertices[vertexIds[1]], vertices[vertexIds[2]], vertices[vertexIds[3]]};

    Tetrahedron tet(id, refs, weight);
    tetrahedrons.push_back(tet);

    unsigned long int current_size = tetrahedrons.size();

    for(unsigned long int i=0; i<neighborIds.size(); i++){
	if(neighborIds[i]<current_size){
	    Tetrahedron neighbor = tetrahedrons[neighborIds[i]];
	    reference_wrapper<Tetrahedron> neighborRef = neighbor;
	    reference_wrapper<Tetrahedron> tetRef = tet;
	    
	    neighbor.addNeighbor(tetRef);
	    tet.addNeighbor(neighborRef);
	}
    }
}

void Mesh::addFace(const array<int, 3> vertexIds, const int tetId){
    vector<reference_wrapper<Vertex3d>> refs = {vertices[vertexIds[0]], vertices[vertexIds[1]], vertices[vertexIds[2]]};

    Face face(refs, tetId);
    faces.push_back(face);
}

bool Mesh::setTarget(array<float, 3> _target){

    for(unsigned long int i=0; i < tetrahedrons.size(); i++){
	if (tetrahedrons[i].contains(_target)){
	    Vector3f t(_target[0], _target[1], _target[2]);
	    target = t;
	    targetTetId=i;//tetrahedrons[i];
	    return true;
	}
    }
    return false;
}


/*
void Mesh::findPaths(const int epsilon, const int numThreads){
    thread t[numThreads];
    vector<array<float, 2>> planes;
    planes.reserve(epsilon*epsilon);

    for(int i=0; i<epsilon; i++){
	for(int j=0; j<epsilon; j++){
	    //planes[epsilon*i + j]={i*M_PI/epsilon, j*M_PI/epsilon};
	    planes.push_back({i*M_PI/epsilon, j*M_PI/epsilon});
	}
    }

    int subarraySize = ((epsilon * epsilon) + numThreads - 1) / numThreads;

    for(int i=0; i<numThreads; i++){
	int start=i*subarraySize;
	int end=start+subarraySize;
	if(end>(epsilon*epsilon)){
	    end=epsilon*epsilon;
	}

	vector<array<float, 2>> planeVector;

	for(int idx=start; idx<end; idx++){
	    planeVector.push_back(planes[idx]);
	}
	t[i]=thread(&Mesh::computeManySlices, this, planeVector);
    }

    for(int i=0; i<numThreads; i++){
	cout << i << endl;
	t[i].join();
    }
}

void Mesh::computeManySlices(vector<array<float, 2>> planeVector){
    cout << planeVector.size() << endl;
    for(unsigned long int i=0; i<planeVector.size(); i++){
	vector<vector<array<float, 3>>> ret= slice(planeVector[i]);
	cout << ret.size() << endl;
    }
}

vector<vector<array<float, 3>>> Mesh::slice(array<float, 2> rotations){
    Plane3d plane(rotations[0], rotations[1], target);
    //this should maybe be an argument to this function
    vector<Tetrahedron *> tetStack = findIntersectingOuterTets(plane);
    
    vector<bool> tetsChecked;
    vector<vector<array<float, 3>>> allPoints;
    for(unsigned long int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(false);
    }

    sliceIds.clear();

    while(tetStack.size() > 0){
	Tetrahedron* tet = tetStack.back();
	tetStack.pop_back();
	
	if(!tetsChecked[tet->getId()]){

	    vector<array<float, 3>> intersectionPoints = tet->intersectsPlane(plane);

	    if(intersectionPoints.size()>2){
		sliceIds.push_back(tet->getId());
		allPoints.push_back(intersectionPoints);
		vector<Tetrahedron *> neighbors = tet->getNeighbors();

		for(int i=0; i<neighbors.size();i++){
		    tetStack.push_back(neighbors[i]);
		}
	    }

	    tetsChecked[tet->getId()]=true;
	}
    }

    return allPoints;
}

vector<int> Mesh::getSliceIds(){
    return sliceIds;
}

vector<Tetrahedron *> Mesh::findIntersectingOuterTets(Plane3d plane){
    vector<Tetrahedron *> intersectingTets;

    for(unsigned long int i=0; i<faces.size(); i++){
	//this isn't working because faces are never added
	vector<Vertex3d *> vertices = faces[i]->Vertices();
	Vector3f v0 = vertices[0]->Vec();
	Vector3f v1 = vertices[1]->Vec();
	Vector3f v2 = vertices[2]->Vec();

	//std::cout << v0 << std::endl;
	//std::cout << v1 << std::endl;
	//std::cout << v2 << std::endl;

	if(plane.intersects(v0, v1) || plane.intersects(v1,v2) || plane.intersects(v0,v2)){
	    intersectingTets.push_back(faces[i]->getTetrahedron());
	}
    }
    intersectingTets.push_back(targetTet);
    return intersectingTets;
}
*/


unsigned long int Mesh::getTargetTetId(){
    return targetTetId;
}
