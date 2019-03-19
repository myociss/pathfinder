#include <array>
#include <vector>

class Vertex3d;
class Tetrahedron;

class Graph {
    Vertex3d **vertices;
    Tetrahedron **tetrahedrons;
  public:
    Graph (const std::vector<std::array<float, 3>> & _vertices);
    //int numVertices();
};

class Vertex3d {
    float x, y, z;
    Tetrahedron **tetrahedrons;
  public:
    Vertex3d (float _x, float _y, float _z);
};

class Tetrahedron {
    Vertex3d **vertices;
    Tetrahedron **neighbors;
  public:
    Tetrahedron ();
};
