#include "mesh.hpp"
#include "plane3d.hpp"
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <thread>

#include <Eigen/Dense>

using namespace Eigen;
//using namespace plane3d;

Mesh::Mesh(const int numVertices, const int numFaces, const int numCells, const int _numThreads){
    vertices.reserve(numVertices);
    faces.reserve(numFaces);
    tetrahedrons.reserve(numCells);
    numThreads = _numThreads;

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

    Tetrahedron* tet = new Tetrahedron(id, tet_vertices, weight, numThreads);
    tetrahedrons.push_back(tet);

    int current_size = tetrahedrons.size();

    for(auto const& value: neighborIds){
	if (value < current_size) {
	    Tetrahedron * neighbor = tetrahedrons[value];
	    neighbor->addNeighbor(tet);
	    tet->addNeighbor(neighbor);
	}
    }
}

//this and the concurrent generation for the algorithm will be very similar

std::vector<std::vector<std::array<float, 3>>> Mesh::slice(float alpha, float theta){
    Plane3d plane(alpha, theta, target);
    std::vector<bool> tetsChecked;

    for(int i=0; i<tetrahedrons.size(); i++){
	//tetsChecked[i]=false;
	tetsChecked.push_back(false);
    }

    std::vector<Tetrahedron *> tetStack = findIntersectingOuterTets(plane);
    std::vector<std::vector<std::array<float, 3>>> allPoints;

    while(tetStack.size() > 0){
	Tetrahedron* tet = tetStack.back();
	tetStack.pop_back();
	
	if(!tetsChecked[tet->getId()]){

	    std::vector<std::array<float, 3>> intersectionPoints = tet->intersectsPlane(plane);

	    if(intersectionPoints.size()>2){
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

std::vector<Tetrahedron *> Mesh::findIntersectingOuterTets(Plane3d plane){
    std::vector<Tetrahedron *> intersectingTets;

    for(int i=0; i<faces.size(); i++){
	std::vector<Vertex3d *> vertices = faces[i]->Vertices();
	Vector3f v0 = vertices[0]->Vec();
	Vector3f v1 = vertices[1]->Vec();
	Vector3f v2 = vertices[2]->Vec();

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

