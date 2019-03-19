class Vertex3d;
class Tetrahedron;

class Graph {
    Vertex3d **vertices;
    Tetrahedron **tetrahedrons;
  public:
    Graph (int, int);
    int numVertices();
};

class Vertex3d {
    float x, y, z;
    Tetrahedron **tetrahedrons;
  public:
    Vertex3d (float, float, float);
};

class Tetrahedron {
    Vertex3d **vertices;
    Tetrahedron **neighbors;
  public:
    Tetrahedron ();
};
