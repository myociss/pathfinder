#include "mesh.hpp"
#include <map>
#include <string>
#include <vector>
#include <thread>

Mesh::Mesh(const int numVertices, const int numFaces, const int numCells){
    vertices.reserve(numVertices);
    faces.reserve(numFaces);
    tetrahedrons.reserve(numCells);
    cores = std::thread::hardware_concurrency;
}

Mesh::setTarget(std::array<float, 3> _target){
    //check each tetrahedron to see if target is in circumsphere
    //if so, tetrahedron goes on stack
    //also set each tetrahedron's checked list to 0
}

void Mesh::setVertices(const std::vector<std::array<float, 3>> & _vertices){
    for(auto const& value: _vertices) {
	Vertex3d pt(value);
	vertices.push_back(&pt);
    }
}

void Mesh::addTetrahedron(const std::array<int, 4> vertexIds, 
	const std::vector<int> neighborIds, const float weight){

    std::array<Vertex3d *, 4> tet_vertices;
    for(int i=0; i<3; i++){
	tet_vertices[i] = vertices[vertexIds[i]];
    }

    Tetrahedron tet(tet_vertices, weight, cores);
    tetrahedrons.push_back(&tet);

    int current_size = tetrahedrons.size();

    for(auto const& value: neighborIds){
	if (value < current_size) {
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


Tetrahedron::Tetrahedron(const std::array<Vertex3d *, 4> _vertices, const float 	_weight, const unsigned int cores){
    vertices = _vertices;
    weight = _weight;

    std::array<std::array<float, 3>, 2> edge0 = {vertices[0]->Vec(), vertices[1]->Vec()};
    std::array<std::array<float, 3>, 2> edge1 = {vertices[0]->Vec(), vertices[2]->Vec()}; 
    std::array<std::array<float, 3>, 2> edge2 = {vertices[0]->Vec(), vertices[3]->Vec()};


    std::array<float, 3> midpt0 = {(edge0[0][0] + edge0[1][0]) / 2, (edge0[0][1] + edge0[1][1]) / 2, (edge0[0][2] + edge0[1][2]) / 2};

    std::array<float, 3> edge0_vec = {edge0[0][0] - midpt0[0], edge0[0][1] - midpt0[1], edge0[0][2] - midpt0[2]};

    float mag_0 = sqrt((edge0_vec[0] * edge0_vec[0]) + (edge0_vec[1] * edge0_vec[1]) + (edge0_vec[2] * edge0_vec[2]));

    std::array<float, 3> n0 = {edge0_vec[0] / mag_0[0], edge0_vec[1] / mag_0[1], edge0_vec[2] / mag_0[2]};

    float d0 = midpt0[0] * n0[0] + midpt0[1] * n0[1] + midpt0[2] * n0[2];




    std::array<float, 3> midpt1 = {(edge1[0][0] + edge1[1][0]) / 2, (edge1[0][1] + edge1[1][1]) / 2, (edge1[0][2] + edge1[1][2]) / 2};

   std::array<float, 3> midpt2 = {(edge2[0][0] + edge2[1][0]) / 2, (edge2[0][1] + edge2[1][1]) / 2, (edge2[0][2] + edge2[1][2]) / 2};
   
    //neighbors.reserve(4);

    /*checkedThreadId.reserve(cores);
    for (int i=0; i < cores; i++) {
	checkedThreadId = -1;
    }*/
}

void Tetrahedron::addNeighbor(Tetrahedron * neighbor){
    neighbors.push_back(neighbor);
}


Vertex3d::Vertex3d(const std::array<float, 3> _vec) {
    vec = _vec;
}

std::array<float, 3> Vertex3d::Vec(){
    return vec;
}

