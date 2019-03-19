#include "graph.hpp"


graph::graph(int vertices_len, int tets_len) {
    vertices = vertex3d* [vertices_len];
    tetrahedrons = tetrahedron* [tets_len];
}

graph::numVertices(){
    return sizeof(graph.vertices)
}
