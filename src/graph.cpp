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
/*Graph::Graph(const std::vector<std::array<float, 3>> & _vertices,
	const std::vector<std::map<std::string) {
    vertices.reserve(_vertices.size());
    for(auto const& value: _vertices) {
	Vertex3d pt(value);
	vertices.push_back(&pt);
    }
    //tetrahedrons = new Tetrahedron*[tets_len];
}

std::vector<Vertex3d *> Graph::Vertices(){
    return vertices;
}
*/
Vertex3d::Vertex3d(const std::array<float, 3> _vec) {
    vec = _vec;
}

std::array<float, 3> Vertex3d::Vec(){
    return vec;
}

