#include "graph.hpp"
#include <map>
#include <string>
#include <vector>

Graph::Graph(const int numVertices, const int numFaces, const int numCells){
    vertices.reserve(numVertices);
    faces.reserve(numFaces);
    tetrahedrons.reserve(numCells);
}

void Graph::setVertices(const std::vector<std::array<float, 3>> & _vertices){
    for(auto const& value: _vertices) {
	Vertex3d pt(value);
	vertices.push_back(&pt);
    }
}

void Graph::addTetrahedron(const std::array<int, 3> vertexIds, 
	const std::vector<int> neighborIds, const float weight){

    std::array<Vertex3d *, 4> tet_vertices;
    for(int i=0; i<3; i++){
	tet_vertices[i] = vertices[vertexIds[i]];
    }

    Tetrahedron tet(tet_vertices, weight);
    tetrahedrons.push_back(&tet);

    int current_size = tetrahedrons.size();

    for(auto const& value: neighborIds){
	if (value < current_size) {
	    Tetrahedron * neighbor = tetrahedrons.at(value);
	    neighbor->addNeighbor(tet);
	    tet->addNeighbor(neighbor);
	}
    }
}

void Graph::addFace(const std::array<int, 3> vertexIds, const int tetId){
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
}


Tetrahedron::Tetrahedron(const std::array<Vertex3d *, 4> _vertices, const float 	_weight){
    vertices = _vertices;
    weight = _weight;
    neighbors.reserve(4);
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

