#include "mesh_components.hpp"
#include "plane3d.hpp"
#include <vector>
#include <functional>
#include <array>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

Tetrahedron::Tetrahedron(const int _id, const vector<reference_wrapper<Vertex3d>> _vertices, const float _weight){
    id = _id;
    vertices = _vertices;
    weight = _weight;
    //cout << vertices[0].get().Vec() << endl;

    Vector3f v0 = vertices[0].get().Vec();
    Vector3f v1 = vertices[1].get().Vec();
    Vector3f v2 = vertices[2].get().Vec();
    Vector3f v3 = vertices[3].get().Vec();

    array<float, 3> faceCentroid = {(v0[0] + v1[0] + v2[0]) / 3, (v0[1] + v1[1] + v2[1]) / 3, (v0[2] + v1[2] + v2[2]) / 3};

    //Vector3f pt3 = vertices[3]->Vec();

    sphereCenter = { (3 * faceCentroid[0] + v3[0])/4, (3 * faceCentroid[1] + v3[1])/4, (3 * faceCentroid[2] + v3[2])/4};

    float r0 = sqrt( pow(sphereCenter[0]-v0[0], 2) + pow(sphereCenter[1]-v0[1], 2) + pow(sphereCenter[2]-v0[2], 2));

    float r1 = sqrt( pow(sphereCenter[0]-v1[0], 2) + pow(sphereCenter[1]-v1[1], 2) + pow(sphereCenter[2]-v1[2], 2));

    float r2 = sqrt( pow(sphereCenter[0]-v2[0], 2) + pow(sphereCenter[1]-v2[1], 2) + pow(sphereCenter[2]-v2[2], 2));

    float r3 = sqrt( pow(sphereCenter[0]-v3[0], 2) + pow(sphereCenter[1]-v3[1], 2) + pow(sphereCenter[2]-v3[2], 2));

    float radii [4] = {r0, r1, r2, r3};

    float r = r0;

    for(int i=1; i<4; i++){
	//float tmp_r = ;
	if (radii[i] < r){
	    r = radii[i];
	}
    }

    sphereRadius = r;
}

void Tetrahedron::addNeighbor(reference_wrapper<Tetrahedron> neighbor){
    neighbors.push_back(neighbor);
}

Face::Face(const vector<reference_wrapper<Vertex3d>> _vertices, unsigned long int _tetId){
    vertices = _vertices;
    tetId=_tetId;
}

bool Tetrahedron::contains(const array<float, 3> pt_target){
    if (sqrt( pow(pt_target[0] - sphereCenter[0], 2) + pow(pt_target[1] - sphereCenter[1], 2) + pow(pt_target[2] - sphereCenter[2], 2)) < sphereRadius){

	Vector3f pt0 = {vertices[0].get().Vec()};
	Vector3f pt1 = {vertices[1].get().Vec()};
	Vector3f pt2 = {vertices[2].get().Vec()};
	Vector3f pt3 = {vertices[3].get().Vec()};

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

/*
vector<array<float, 3>> Tetrahedron::intersectsPlane(Plane3d plane){
    vector<array<float, 3>> intersectionPoints;

    for(int i=0; i<4; i++){
	for(int j=i+1; j<4; j++){
	    Vector3f v0 = vertices[i]->Vec();
	    Vector3f v1 = vertices[j]->Vec();
	    if(plane.containsEdge(v0, v1)){
		array<float, 3> v0Pt = {v0[0], v0[1], v0[2]};
		array<float, 3> v1Pt = {v1[0], v1[1], v1[2]};
		intersectionPoints.push_back(v0Pt);
		intersectionPoints.push_back(v1Pt);
	    } else{
		if(id==2){
		    plane.intersects(v0,v1);
		}
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


int Tetrahedron::getId(){
    return id;
}

vector<reference_wrapper<Tetrahedron>> Tetrahedron::getNeighbors(){
    return neighbors;
}

vector<reference_wrapper<Vertex3d>> Tetrahedron::Vertices(){
    return vertices;
}

Tetrahedron* Face::getTetrahedron(){
    return tetrahedron;
}

vector<Vertex3d *> Face::Vertices(){
    return vertices;
}*/

Vertex3d::Vertex3d(const array<float, 3> _vec) {
    //Vertex3f vec;
    //vec = _vec;
    vec << _vec[0], _vec[1], _vec[2];
}


Vector3f Vertex3d::Vec(){

    return vec;
}

