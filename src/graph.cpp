#include "graph.hpp"


Graph::Graph(int vertices_len, int tets_len) {
    vertices = new Vertex3d*[vertices_len];
    tetrahedrons = new Tetrahedron*[tets_len];
}

int Graph::numVertices(){
    return sizeof(vertices)/sizeof(vertices[0]);
}
