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
    //cores = std::thread::hardware_concurrency;
}

//void Mesh::setTarget(std::array<float, 3> _target){
    //check each tetrahedron to see if target is in circumsphere
    //if so, tetrahedron goes on stack
    //also set each tetrahedron's checked list to 0
//}

void Mesh::setVertices(const std::vector<std::array<float, 3>> _vertices){
    //std::cout << _vertices << std::endl;
    for(int i=0; i < _vertices.size(); i++){
	Vertex3d* pt = new Vertex3d(_vertices[i]);
	//std::cout << i << std::endl;
	//std::cout << pt << std::endl;
	vertices.push_back(pt);
    }

    //for(auto const value: _vertices) {
//	Vertex3d* pt = new Vertex3d(_vertices.at(value));
//	vertices.push_back(pt);
//	std::cout << pt << std::endl;
	//std::cout << "here" << std::endl;
	//Vertex3d pt(value);
	//vertices.push_back(&pt);
  //  }
}

void Mesh::addTetrahedron(const std::array<int, 4> vertexIds, 
	const std::vector<int> neighborIds, const float weight){

    std::array<Vertex3d *, 4> tet_vertices;
    for(int i=0; i<3; i++){
	tet_vertices[i] = vertices.at(vertexIds[i]);
	//std::cout << vertexIds[i] << std::endl;
	//std::cout << vertices.at(vertexIds[i]) << std::endl;
    }

    Tetrahedron tet(tet_vertices, weight, numThreads);
    tetrahedrons.push_back(&tet);

    int current_size = tetrahedrons.size();

    for(auto const& value: neighborIds){
	if (value < current_size) {
	    //std::cout << value << std::endl;
	    Tetrahedron * neighbor = tetrahedrons.at(value);
	    neighbor->addNeighbor(&tet);
	    tet.addNeighbor(neighbor);
	}
    }
}

/*void Graph::addFace(const std::array<int, 3> vertexIds, const int tetId){
    std::array<Vector3d *> face_vertices;
    for(int i=0; i<3; i++){
	face_vertices[i] = vertexIds[i];
    }
    Face face(face_vertices, tetrahedrons.at(tetId));
    faces.push_back(&face)
}

Face::Face(const std::array<Vertex3d *, 3> _vertices, const Tetrahedron * tet){
    vertices = _vertices;
    tetrahedron = tet;
}*/


Tetrahedron::Tetrahedron(const std::array<Vertex3d *, 4> _vertices, const float 	_weight, const int numThreads){
    vertices = _vertices;
    weight = _weight;

    //std::cout << vertices[0]->Vec()[0] << std::endl;

    std::array<std::array<float, 3>, 2> edge0 = {vertices[0]->Vec(), vertices[1]->Vec()};
    std::array<std::array<float, 3>, 2> edge1 = {vertices[0]->Vec(), vertices[2]->Vec()}; 
    std::array<std::array<float, 3>, 2> edge2 = {vertices[0]->Vec(), vertices[3]->Vec()};

     
   
    std::array<float, 3> midpt0 = {(edge0[0][0] + edge0[1][0]) / 2, (edge0[0][1] + edge0[1][1]) / 2, (edge0[0][2] + edge0[1][2]) / 2};

    std::array<float, 3> edge0_vec = {edge0[0][0] - midpt0[0], edge0[0][1] - midpt0[1], edge0[0][2] - midpt0[2]};

    //std::cout << edge0_vec << std::endl;

    


    float d0 = midpt0[0] * edge0_vec[0] + midpt0[1] * edge0_vec[1] + midpt0[2] * edge0_vec[2];
    
//std::cout << d0 << std::endl;


    std::array<float, 3> midpt1 = {(edge1[0][0] + edge1[1][0]) / 2, (edge1[0][1] + edge1[1][1]) / 2, (edge1[0][2] + edge1[1][2]) / 2};

    std::array<float, 3> edge1_vec = {edge1[0][0] - midpt1[0], edge1[0][1] - midpt1[1], edge1[0][2] - midpt1[2]};


    float d1 = midpt1[0] * edge1_vec[0] + midpt1[1] * edge1_vec[1] + midpt1[2] * edge1_vec[2];




   std::array<float, 3> midpt2 = {(edge2[0][0] + edge2[1][0]) / 2, (edge2[0][1] + edge2[1][1]) / 2, (edge2[0][2] + edge2[1][2]) / 2};

    std::array<float, 3> edge2_vec = {edge2[0][0] - midpt2[0], edge2[0][1] - midpt2[1], edge2[0][2] - midpt2[2]};



    float d2 = midpt2[0] * edge2_vec[0] + midpt2[1] * edge2_vec[1] + midpt2[2] * edge2_vec[2];

   //std::cout << d0 << std::endl;
   //std::cout << d1 << std::endl;
   //std::cout << d2 << std::endl;


    Matrix3f A;
    Vector3f b;

    A << midpt0[0], midpt0[1], midpt0[2], midpt1[0], midpt1[1], midpt1[2], midpt2[0], midpt2[1], midpt2[2];
   

    b << -d0, -d1, -d2;

    Vector3f circumcenter = A.colPivHouseholderQr().solve(b);

    std::cout << "matrix A:\n" << A << std::endl;
    std::cout << "vector b:\n" << b << std::endl;
    std::cout << "circumcenter:\n" << circumcenter << std::endl;
   
}

void Tetrahedron::addNeighbor(Tetrahedron * neighbor){
    neighbors.push_back(neighbor);
}


Vertex3d::Vertex3d(const std::array<float, 3> _vec) {
    vec = _vec;
    //std::cout << vec[0] << std::endl;
    //std::cout << vec[1] << std::endl;
    //std::cout << vec[2] << std::endl;
}

std::array<float, 3> Vertex3d::Vec(){
    //std::cout << "this happens" << std::endl;
    return vec;
}

