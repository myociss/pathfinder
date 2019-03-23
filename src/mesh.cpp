#include "mesh.hpp"
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <thread>

#include <Eigen/Dense>

using namespace Eigen;

Mesh::Mesh(const int numVertices, const int numFaces, const int numCells, const int _numThreads){
    vertices.reserve(numVertices);
    faces.reserve(numFaces);
    tetrahedrons.reserve(numCells);
    numThreads = _numThreads;

}

bool Mesh::setTarget(std::array<float, 3> _target){

    for(int i=0; i < tetrahedrons.size(); i++){
	if (tetrahedrons[i]->contains(_target)){
	    target=_target;
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



Tetrahedron::Tetrahedron(const int _id, const std::array<Vertex3d *, 4> _vertices, const float 	_weight, const int numThreads){
    id = _id;
    vertices = _vertices;
    weight = _weight;

    std::array<float, 3> faceCentroid = {(vertices[0]->Vec()[0] + vertices[1]->Vec()[0] + vertices[2]->Vec()[0]) / 3, (vertices[0]->Vec()[1] + vertices[1]->Vec()[1] + vertices[2]->Vec()[1]) / 3, (vertices[0]->Vec()[2] + vertices[1]->Vec()[2] + vertices[2]->Vec()[2]) / 3};

    std::array<float, 3> pt3 = vertices[3]->Vec();

    sphereCenter = { (3 * faceCentroid[0] + pt3[0])/4, (3 * faceCentroid[1] + pt3[1])/4, (3 * faceCentroid[2] + pt3[2])/4};

    float r = sqrt( std::pow(sphereCenter[0]-vertices[0]->Vec()[0], 2) + std::pow(sphereCenter[1]-vertices[0]->Vec()[1], 2) + std::pow(sphereCenter[2]-vertices[0]->Vec()[2], 2));

    for(int i=1; i<4; i++){
	float tmp_r = sqrt( std::pow(sphereCenter[0]-vertices[i]->Vec()[0], 2) + std::pow(sphereCenter[1]-vertices[i]->Vec()[1], 2) + std::pow(sphereCenter[2]-vertices[i]->Vec()[2], 2));
	if (tmp_r < r){
	    r = tmp_r;
	}
    }

    sphereRadius = r;
   
}

bool Tetrahedron::contains(const std::array<float, 3> pt_target){
    if (sqrt( std::pow(pt_target[0] - sphereCenter[0], 2) + std::pow(pt_target[1] - sphereCenter[1], 2) + std::pow(pt_target[2] - sphereCenter[2], 2)) < sphereRadius){

	std::array<float, 3> pt0 = {(vertices[0])->Vec()};
	std::array<float, 3> pt1 = {(vertices[1])->Vec()};
	std::array<float, 3> pt2 = {(vertices[2])->Vec()};
	std::array<float, 3> pt3 = {(vertices[3])->Vec()};

	Matrix4f m0;
	Matrix4f m1;
	Matrix4f m2;
	Matrix4f m3;
	Matrix4f m4;

	m0 << pt0[0], pt0[1], pt0[2], 1,
	      pt1[0], pt1[1], pt1[2], 1,
	      pt2[0], pt2[1], pt2[2], 1,
	      pt3[0], pt3[1], pt3[2], 1;

	float d0 = m0.determinant();

	m1 << pt_target[0], pt_target[1], pt_target[2], 1,
	      pt1[0], pt1[1], pt1[2], 1,
	      pt2[0], pt2[1], pt2[2], 1,
	      pt3[0], pt3[1], pt3[2], 1;

	float d1 = m1.determinant();

	m2 << pt0[0], pt0[1], pt0[2], 1,
	      pt_target[0], pt_target[1], pt_target[2], 1,
	      pt2[0], pt2[1], pt2[2], 1,
	      pt3[0], pt3[1], pt3[2], 1;

	float d2 = m2.determinant();

	m3 << pt0[0], pt0[1], pt0[2], 1,
	      pt1[0], pt1[1], pt1[2], 1,
	      pt_target[0], pt_target[1], pt_target[2], 1,
	      pt3[0], pt3[1], pt3[2], 1;

	float d3 = m3.determinant();

	m4 << pt0[0], pt0[1], pt0[2], 1,
	      pt1[0], pt1[1], pt1[2], 1,
	      pt2[0], pt2[1], pt2[2], 1,
	      pt_target[0], pt_target[1], pt_target[2], 1;

	float d4 = m4.determinant();

	return (d0<0 && d1<0 && d2<0 && d3<0 && d4<0) || (d0>0 && d1>0 && d2>0 && d3>0 && d4>0);
    }
    return false;
}

void Tetrahedron::addNeighbor(Tetrahedron * neighbor){
    neighbors.push_back(neighbor);
}

int Mesh::getTargetTetId(){
    return targetTet->getId();
}

int Tetrahedron::getId(){
    return id;
}

Vertex3d::Vertex3d(const std::array<float, 3> _vec) {
    vec = _vec;
}


std::array<float, 3> Vertex3d::Vec(){

    return vec;
}

