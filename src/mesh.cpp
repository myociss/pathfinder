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

//std::vector<std::array<float, 3>> Mesh::getIntersectionWith(float alpha, float theta){
//    Plane3d plane3d(alpha, theta);

    //for(int i=0; i < faces.size(); i++){
	//if face intersects plane
	
//}

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

	std::vector<std::array<float, 3>> intersectionPoints;
	
	if(!tetsChecked[tet->getId()]){
	    std::vector<Vertex3d *> tetVertices = tet->Vertices();

	    for(int i=0; i<4; i++){
		for(int j=i+1; j<4; j++){
		    Vector3f v0 = tetVertices[i]->Vec();
		    Vector3f v1 = tetVertices[j]->Vec();
		    if(plane.intersects(v0, v1)){
			intersectionPoints.push_back(plane.findIntersection(v0,v1));
		    }
		}
	    }
	    //need to switch indices 2 and 3 if length==4
	    //this part should probably be member of tet class

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
    //Plane3d plane(alpha, theta, target);

    for(int i=0; i<faces.size(); i++){
	//if(intersectsVertices(plane, faces[i]->Vertices())){
	//    intersectingTets.push_back(faces[i]->getTetrahedron());
	//}
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

bool intersectsVertices(Plane3d plane, std::vector<Vertex3d *> vertices){

    /*std::array<float, 3> dotProducts;
    for(int i=0; i < 3; i++){
	std::array<float, 3> pt = vertices[i]->Vec();
	Vector3f v0;
	v0 << pt[0], pt[1], pt[2];
	Vector3f diffVec = v0 - plane.getTarget();
	 
	dotProducts[i] = diffVec.dot(plane.getNormal());
    }*/

    /*for(int i=0; i < vertices.size(); i++){
	int next = i+1;
	if(next==vertices.size()){
	    next = 0;
	}
	//if(dotProducts[i] * dotProducts[next] < 0){
	if(plane.intersects(vertices[i]->Vec(), vertices[next]->Vec())){ 
	    return true;
	}
    }*/
    for(int i=0; i<vertices.size(); i++){
	for(int j=i+1; j<vertices.size(); j++){
	    if(plane.intersects(vertices[i]->Vec(), vertices[j]->Vec())){ 
		return true;
	    }
	}
    }
    return false;
}

int Mesh::getTargetTetId(){
    return targetTet->getId();
}

