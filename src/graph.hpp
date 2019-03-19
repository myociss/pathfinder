class vertex3d;
class tetrahedron;

class graph {
    vertex3d *vertices;
    tetrahedron *tetrahedrons;
  public:
    graph (int, int);
    int numVertices();
};

class vertex3d {
    float x, y, z;
    tetrahedron *tetrahedrons;
  public:
    vertex3d (float, float, float);
};

class tetrahedron {
    vertex3d *vertices;
    tetrahedron *neighbors;
  public:
    tetrahedron (std::array<int, 4>, std::array<int, 4>);
};
