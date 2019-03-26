#include "mesh_components.hpp"
#include "plane3d.hpp"
#include <vector>
#include <array>
#include <Eigen/Dense>

using namespace Eigen;

Tetrahedron::Tetrahedron(const int _id, const std::array<Vertex3d *, 4> _vertices, const float _weight, const int numThreads){
    vertices.reserve(4);
    id = _id;
    for(int i=0; i<4;i++){
	vertices[i] = _vertices[i];
    }
    weight = _weight;

    std::array<float, 3> faceCentroid = {(vertices[0]->Vec()[0] + vertices[1]->Vec()[0] + vertices[2]->Vec()[0]) / 3, (vertices[0]->Vec()[1] + vertices[1]->Vec()[1] + vertices[2]->Vec()[1]) / 3, (vertices[0]->Vec()[2] + vertices[1]->Vec()[2] + vertices[2]->Vec()[2]) / 3};

    Vector3f pt3 = vertices[3]->Vec();

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

	Vector3f pt0 = {(vertices[0])->Vec()};
	Vector3f pt1 = {(vertices[1])->Vec()};
	Vector3f pt2 = {(vertices[2])->Vec()};
	Vector3f pt3 = {(vertices[3])->Vec()};

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

std::vector<std::array<float, 3>> Tetrahedron::intersectsPlane(Plane3d plane){
    std::vector<std::array<float, 3>> intersectionPoints;

    for(int i=0; i<4; i++){
	for(int j=i+1; j<4; j++){
	    Vector3f v0 = vertices[i]->Vec();
	    Vector3f v1 = vertices[j]->Vec();
	    if(plane.containsEdge(v0, v1)){
		std::array<float, 3> v0Pt = {v0[0], v0[1], v0[2]};
		std::array<float, 3> v1Pt = {v1[0], v1[1], v1[2]};
		intersectionPoints.push_back(v0Pt);
		intersectionPoints.push_back(v1Pt);
	    } else{
	        if(plane.intersects(v0, v1)){
		    intersectionPoints.push_back(plane.findIntersection(v0,v1));
		}
	    }
	}
    }

    if(intersectionPoints.size()==4){
	intersectionPoints = {intersectionPoints[0], intersectionPoints[1], intersectionPoints[3], intersectionPoints[2]};
    }

    return intersectionPoints;

}

void Tetrahedron::addNeighbor(Tetrahedron * neighbor){
    neighbors.push_back(neighbor);
}


int Tetrahedron::getId(){
    return id;
}

std::vector<Tetrahedron *> Tetrahedron::getNeighbors(){
    return neighbors;
}

std::vector<Vertex3d *> Tetrahedron::Vertices(){
    return vertices;
}


Face::Face(const std::array<Vertex3d *, 3> _vertices, Tetrahedron * _tetrahedron){
    vertices.reserve(3);
    for(int i=0; i<3; i++){
	vertices[i]=_vertices[i];
    }
    tetrahedron=_tetrahedron;
}

Tetrahedron* Face::getTetrahedron(){
    return tetrahedron;
}

std::vector<Vertex3d *> Face::Vertices(){
    return vertices;
}

Vertex3d::Vertex3d(const std::array<float, 3> _vec) {
    //Vertex3f vec;
    //vec = _vec;
    vec << _vec[0], _vec[1], _vec[2];
}


Vector3f Vertex3d::Vec(){

    return vec;
}

