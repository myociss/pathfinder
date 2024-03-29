#include "mesh_components.hpp"
#include "plane3d.hpp"
#include <vector>
#include <functional>
#include <array>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

Tetrahedron::Tetrahedron(const int _id, const vector<reference_wrapper<Vertex3d>> _vertices, const double _weight, const int _label){
    id = _id;
    vertices = _vertices;
    weight = _weight;
    label = _label;
    //cout << vertices[0].get().Vec() << endl;

    Vector3d v0 = vertices[0].get().Vec();
    Vector3d v1 = vertices[1].get().Vec();
    Vector3d v2 = vertices[2].get().Vec();
    Vector3d v3 = vertices[3].get().Vec();

    //array<double, 3> faceCentroid = {(v0[0] + v1[0] + v2[0]) / 3, (v0[1] + v1[1] + v2[1]) / 3, (v0[2] + v1[2] + v2[2]) / 3};

    //Vector3f pt3 = vertices[3]->Vec();

    sphereCenter = { (v0[0] + v1[0] + v2[0] + v3[0])/4, (v0[1] + v1[1] + v2[1] + v3[1])/4, (v0[2] + v1[2] + v2[2] + v3[2])/4};

    double r0 = sqrt( pow(sphereCenter[0]-v0[0], 2) + pow(sphereCenter[1]-v0[1], 2) + pow(sphereCenter[2]-v0[2], 2));

    double r1 = sqrt( pow(sphereCenter[0]-v1[0], 2) + pow(sphereCenter[1]-v1[1], 2) + pow(sphereCenter[2]-v1[2], 2));

    double r2 = sqrt( pow(sphereCenter[0]-v2[0], 2) + pow(sphereCenter[1]-v2[1], 2) + pow(sphereCenter[2]-v2[2], 2));

    double r3 = sqrt( pow(sphereCenter[0]-v3[0], 2) + pow(sphereCenter[1]-v3[1], 2) + pow(sphereCenter[2]-v3[2], 2));

    double radii [4] = {r0, r1, r2, r3};

    double r = r0;

    for(int i=1; i<4; ++i){
	//float tmp_r = ;
	if (radii[i] > r){
	    r = radii[i];
	}
    }

    sphereRadius = r;
}

void Tetrahedron::addNeighbor(unsigned long int neighborId){
    neighbors.push_back(neighborId);
}

Face::Face(const vector<reference_wrapper<Vertex3d>> _vertices, unsigned long int _tetId){
    vertices = _vertices;
    tetId=_tetId;
}

bool Tetrahedron::contains(const array<double, 3> pt_target){
    if (sqrt( pow(pt_target[0] - sphereCenter[0], 2) + pow(pt_target[1] - sphereCenter[1], 2) + pow(pt_target[2] - sphereCenter[2], 2)) < sphereRadius){
        /*cout << pt_target[0] << endl;
        cout << pt_target[1] << endl;
        cout << pt_target[2] << endl;
        cout << "HERE" << endl;*/

	Vector3d pt0 = {vertices[0].get().Vec()};
	Vector3d pt1 = {vertices[1].get().Vec()};
	Vector3d pt2 = {vertices[2].get().Vec()};
	Vector3d pt3 = {vertices[3].get().Vec()};

	/*cout << "pt0" << endl;
	cout << pt0 << endl;
	cout << "pt1" << endl;
	cout << pt1 << endl;
	cout << "pt2" << endl;
	cout << pt2 << endl;
	cout << "pt3" << endl;
	cout << pt3 << endl;*/

	Matrix4d m0;
	Matrix4d m1;
	Matrix4d m2;
	Matrix4d m3;
	Matrix4d m4;

	m0 << pt0[0], pt0[1], pt0[2], 1,
	      pt1[0], pt1[1], pt1[2], 1,
	      pt2[0], pt2[1], pt2[2], 1,
	      pt3[0], pt3[1], pt3[2], 1;

	double d0 = m0.determinant();

	m1 << pt_target[0], pt_target[1], pt_target[2], 1,
	      pt1[0], pt1[1], pt1[2], 1,
	      pt2[0], pt2[1], pt2[2], 1,
	      pt3[0], pt3[1], pt3[2], 1;

	double d1 = m1.determinant();

	m2 << pt0[0], pt0[1], pt0[2], 1,
	      pt_target[0], pt_target[1], pt_target[2], 1,
	      pt2[0], pt2[1], pt2[2], 1,
	      pt3[0], pt3[1], pt3[2], 1;

	double d2 = m2.determinant();

	m3 << pt0[0], pt0[1], pt0[2], 1,
	      pt1[0], pt1[1], pt1[2], 1,
	      pt_target[0], pt_target[1], pt_target[2], 1,
	      pt3[0], pt3[1], pt3[2], 1;

	double d3 = m3.determinant();

	m4 << pt0[0], pt0[1], pt0[2], 1,
	      pt1[0], pt1[1], pt1[2], 1,
	      pt2[0], pt2[1], pt2[2], 1,
	      pt_target[0], pt_target[1], pt_target[2], 1;

	double d4 = m4.determinant();

	/*cout << d0 << endl;
	cout << d1 << endl;
	cout << d2 << endl;
	cout << d3 << endl;
	cout << d4 << endl;*/

	return (d0<0 && d1<0 && d2<0 && d3<0 && d4<0) || (d0>0 && d1>0 && d2>0 && d3>0 && d4>0);
    }
    return false;
}


vector<array<double, 3>> Tetrahedron::intersectsPlane(Plane3d plane){
    vector<array<double, 3>> intersectionPoints;

    for(int i=0; i<4; ++i){
	Vector3d v0 = vertices[i].get().Vec();
	if(plane.containsPoint(v0)){
	    array<double, 3> v0Pt = {v0[0], v0[1], v0[2]};
	    intersectionPoints.push_back(v0Pt);
	} 
	else {
	    for(int j=i+1; j<4; ++j){
		Vector3d v1 = vertices[j].get().Vec();
		if(plane.intersectsEdge(v0, v1)){
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

bool Face::intersectsPlane(Plane3d plane){
    Vector3d v0 = vertices[0].get().Vec();
    Vector3d v1 = vertices[1].get().Vec();
    Vector3d v2 = vertices[2].get().Vec();

    return (plane.intersectsEdge(v0, v1) || plane.intersectsEdge(v1, v2) || plane.intersectsEdge(v0, v2));
}


unsigned long int Tetrahedron::Id(){
    return id;
}

vector<unsigned long int> Tetrahedron::Neighbors(){
    return neighbors;
}

vector<reference_wrapper<Vertex3d>> Tetrahedron::Vertices(){
    return vertices;
}

double Tetrahedron::Weight(){
    return weight;
}

int Tetrahedron::Label(){
    return label;
}

unsigned long int Face::TetId(){
    return tetId;
}

vector<reference_wrapper<Vertex3d>> Face::Vertices(){
    return vertices;
}

Vertex3d::Vertex3d(array<double, 3> _vec) {
    //vec = _vec;
    vec << _vec[0], _vec[1], _vec[2];
}


Vector3d Vertex3d::Vec(){
    return vec;
}

