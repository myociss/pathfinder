#define _USE_MATH_DEFINES

#include "mesh.hpp"
#include "plane3d.hpp"
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <thread>

#include <Eigen/Dense>

using namespace Eigen;
//using namespace plane3d;

Mesh::Mesh(const int numVertices, const int numFaces, const int numCells){
    vertices.reserve(numVertices);
    faces.reserve(numFaces);
    tetrahedrons.reserve(numCells);
}

void Mesh::findPaths(const int epsilon, const int numThreads){
    std::thread t[numThreads];
    std::vector<std::array<float, 2>> planes;
    planes.reserve(epsilon*epsilon);

    for(int i=0; i<epsilon; i++){
	for(int j=0; j<epsilon; j++){
	    //planes[epsilon*i + j]={i*M_PI/epsilon, j*M_PI/epsilon};
	    planes.push_back({i*M_PI/epsilon, j*M_PI/epsilon});
	}
    }

    int subarraySize = ((epsilon * epsilon) + numThreads - 1) / numThreads;
    
    /*for(int i=0; i<(epsilon*epsilon); i+=subarraySize){
	int end=(epsilon*epsilon);
	if(end>(epsilon*epsilon)){
	    end=(epsilon*epsilon);
	}

	std::vector<std::array<float, 2>> planeVector;

	for(int idx=i; idx<end; idx++){
	    planeVector.push_back(planes[idx]);
	}
	t[i]=std::thread(&Mesh::computeManySlices, this, planeVector);
    }*/
    for(int i=0; i<numThreads; i++){
	int start=i*subarraySize;
	int end=i+subarraySize;
	if(end>(epsilon*epsilon)){
	    end=epsilon*epsilon;
	}

	std::vector<std::array<float, 2>> planeVector;

	for(int idx=start; idx<end; idx++){
	    planeVector.push_back(planes[idx]);
	}
	t[i]=std::thread(&Mesh::computeManySlices, this, planeVector);
    }

    for(int i=0; i<numThreads; i++){
	std::cout << i << std::endl;
	t[i].join();
    }
}

void Mesh::computeManySlices(std::vector<std::array<float, 2>> planeVector){
    for(int i=0; i<planeVector.size(); i++){
	std::vector<std::vector<std::array<float, 3>>> ret= slice(planeVector[i]);
	std::cout << ret.size() << std::endl;
    }
}

bool Mesh::setTarget(std::array<float, 3> _target){

    for(int i=0; i < tetrahedrons.size(); i++){
	if (tetrahedrons[i]->contains(_target)){
	    Vector3f t(_target[0], _target[1], _target[2]);
	    target = t;
	    targetTet=tetrahedrons[i];
	    return true;
	}
    }
    return false;
}

void Mesh::setVertices(const std::vector<std::array<float, 3>> & _vertices){

    for(int i=0; i < _vertices.size(); i++){
	Vertex3d* pt = new Vertex3d(_vertices[i]);
	
	vertices.push_back(pt);

    }

}

void Mesh::addFace(const std::array<int, 3> vertexIds, const int tetId){
    std::array<Vertex3d *, 3> face_vertices;
    for(int i=0; i<3; i++){
	face_vertices[i] = vertices[vertexIds[i]];
    }
    Face* face = new Face(face_vertices, tetrahedrons[tetId]);
    faces.push_back(face);
}
    

void Mesh::addTetrahedron(const int id, const std::array<int, 4> vertexIds, 
	const std::vector<int> neighborIds, const float weight){

    std::array<Vertex3d *, 4> tet_vertices;
    for(int i=0; i<4; i++){
	tet_vertices[i] = vertices[vertexIds[i]];
    }

    Tetrahedron* tet = new Tetrahedron(id, tet_vertices, weight);
    tetrahedrons.push_back(tet);

    int current_size = tetrahedrons.size();

    for(int i=0; i<neighborIds.size(); i++){
	if(neighborIds[i]<current_size){
	    Tetrahedron* neighbor = tetrahedrons[neighborIds[i]];
	    neighbor->addNeighbor(tet);
	    tet->addNeighbor(neighbor);
	}
    }
}

std::vector<std::vector<std::array<float, 3>>> Mesh::slice(std::array<float, 2> rotations){
    Plane3d plane(rotations[0], rotations[1], target);
    //this should maybe be an argument to this function
    std::vector<Tetrahedron *> tetStack = findIntersectingOuterTets(plane);
    
    std::vector<bool> tetsChecked;
    std::vector<std::vector<std::array<float, 3>>> allPoints;
    for(int i=0; i<tetrahedrons.size(); i++){
	tetsChecked.push_back(false);
    }

    sliceIds.clear();

    while(tetStack.size() > 0){
	Tetrahedron* tet = tetStack.back();
	tetStack.pop_back();
	
	if(!tetsChecked[tet->getId()]){

	    std::vector<std::array<float, 3>> intersectionPoints = tet->intersectsPlane(plane);

	    if(intersectionPoints.size()>2){
		sliceIds.push_back(tet->getId());
		allPoints.push_back(intersectionPoints);
		std::vector<Tetrahedron *> neighbors = tet->getNeighbors();

		for(int i=0; i<neighbors.size();i++){
		    tetStack.push_back(neighbors[i]);
		}
	    }

	    tetsChecked[tet->getId()]=true;
	}
    }

    return allPoints;
}

std::vector<int> Mesh::getSliceIds(){
    return sliceIds;
}

std::vector<Tetrahedron *> Mesh::findIntersectingOuterTets(Plane3d plane){
    std::vector<Tetrahedron *> intersectingTets;

    for(int i=0; i<faces.size(); i++){
	//this isn't working because faces are never added
	std::vector<Vertex3d *> vertices = faces[i]->Vertices();
	Vector3f v0 = vertices[0]->Vec();
	Vector3f v1 = vertices[1]->Vec();
	Vector3f v2 = vertices[2]->Vec();

	std::cout << v0 << std::endl;
	std::cout << v1 << std::endl;
	std::cout << v2 << std::endl;

	if(plane.intersects(v0, v1) || plane.intersects(v1,v2) || plane.intersects(v0,v2)){
	    intersectingTets.push_back(faces[i]->getTetrahedron());
	}
    }
    intersectingTets.push_back(targetTet);
    return intersectingTets;
}

int Mesh::getTargetTetId(){
    return targetTet->getId();
}

