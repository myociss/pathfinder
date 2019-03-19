#include "graph.hpp"
//#include <vector>


Graph::Graph(const std::vector<std::array<float, 3>> & _vertices) {
    vertices = new Vertex3d*[_vertices.size()];
    for (std::vector<std::array<float, 3>>::size_type i=0; i < _vertices.size(); i++) {
        Vertex3d pt(_vertices[i][0],
	    _vertices[i][1], _vertices[i][2]);
	vertices[i] = &pt;
    }
    //vertices = new Vertex3d*[vertices_len];
    //tetrahedrons = new Tetrahedron*[tets_len];
}

Vertex3d::Vertex3d(float _x, float _y, float _z) {
    x = _x;
    y = _y;
    z = _z;
}
//int Graph::numVertices(){
//    return sizeof(vertices)/sizeof(vertices[0]);
//}
