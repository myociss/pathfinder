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

    for(unsigned long int i=0; i < _vertices.size(); ++i){
	//Vertex3d* pt = new Vertex3d(_vertices[i]);
	Vertex3d pt(_vertices[i]);
	
	vertices.push_back(pt);
	//cout << pt.Vec() << endl;
    }

}


void Mesh::addTetrahedron(const int id, const array<int, 4> vertexIds, 
	const vector<unsigned long int> neighborIds, const float weight){

    vector<reference_wrapper<Vertex3d>> refs = {vertices[vertexIds[0]], vertices[vertexIds[1]], vertices[vertexIds[2]], vertices[vertexIds[3]]};

    Tetrahedron tet(id, refs, weight);

    unsigned long int current_size = tetrahedrons.size();
    //cout << neighborIds.size() << endl;

    for(unsigned long int i=0; i<neighborIds.size(); ++i){
	if(neighborIds[i]<current_size){
	   // cout << "HERE" << endl;
	    Tetrahedron& neighbor = tetrahedrons[neighborIds[i]];
	    //reference_wrapper<Tetrahedron> neighborRef = ref(neighbor);
	    //reference_wrapper<Tetrahedron> tetRef = ref(tet);
	    
	    neighbor.addNeighbor(tet.Id());
	    tet.addNeighbor(neighbor.Id());
	    //cout << tet.Neighbors().size() << endl;
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

vector<vector<array<float, 3>>> Mesh::slice(array<float, 2> rotations, bool test){
    vector<bool> tetsChecked;
    tetsChecked.reserve(tetrahedrons.size());
    for(unsigned long int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(false);
    }

    Plane3d plane(rotations[0], rotations[1], target);

    return computeSlice(plane, tetsChecked, test);
}


//vector<vector<array<float, 3>>> Mesh::slice(array<float, 2> rotations, vector<bool> &tetsChecked, bool test=false){

vector<vector<array<float, 3>>> Mesh::computeSlice(Plane3d plane, vector<bool> &tetsChecked, bool test){

    //Plane3d plane(rotations[0], rotations[1], target);
    //this should maybe be an argument to this function
    vector<unsigned long int> tetStack = findIntersectingOuterTets(plane);

    vector<vector<array<float, 3>>> allPoints;
    
    /*vector<bool> tetsChecked;
    for(unsigned long int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(false);
    }*/

    if(test){
	sliceIds.clear();
    }

    while(tetStack.size() > 0){
	Tetrahedron& tet = tetrahedrons[tetStack.back()];
	tetStack.pop_back();
	
	if(!tetsChecked[tet.Id()]){

	    vector<array<float, 3>> intersectionPoints = tet.intersectsPlane(plane);

	    if(intersectionPoints.size()>2){
		if(test){
		    sliceIds.push_back(tet.Id());
		}
		allPoints.push_back(intersectionPoints);
		//cout << "THIS HAPPENS0" << endl;
		vector<unsigned long int> neighbors = tet.Neighbors();
		cout << neighbors.size() << endl;

		for(int i=0; i<neighbors.size();++i){
		    //cout << "THIS HAPPENS" << endl;
		    tetStack.push_back(neighbors[i]);
		}
	    }

	    tetsChecked[tet.Id()]=true;
	}
    }

    return allPoints;
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
*/

vector<int> Mesh::getSliceIds(){
    return sliceIds;
}

vector<unsigned long int> Mesh::findIntersectingOuterTets(Plane3d plane){
    vector<unsigned long int> intersectingTets;

    for(unsigned long int i=0; i<faces.size(); ++i){
	vector<reference_wrapper<Vertex3d>> vertices = faces[i].Vertices();
	Vector3f v0 = vertices[0].get().Vec();
	Vector3f v1 = vertices[1].get().Vec();
	Vector3f v2 = vertices[2].get().Vec();

	//std::cout << v0 << std::endl;
	//std::cout << v1 << std::endl;
	//std::cout << v2 << std::endl;

	if(plane.intersects(v0, v1) || plane.intersects(v1,v2) || plane.intersects(v0,v2)){
	    //intersectingTets.push_back(faces[i]->getTetrahedron());
	    
	    //reference_wrapper<Tetrahedron> tet = tetrahedrons[faces[i].TetId()];
	    intersectingTets.push_back(faces[i].TetId());
	}
    }
    //reference_wrapper<Tetrahedron> target = tetrahedrons[targetTetId];
    intersectingTets.push_back(targetTetId);
    return intersectingTets;
}

unsigned long int Mesh::getTargetTetId(){
    return targetTetId;
}
