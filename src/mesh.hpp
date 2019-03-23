#include <array>
#include <vector>

class Vertex3d;
class Face;
class Tetrahedron;

class Mesh {
    std::vector<Vertex3d *> vertices;
    std::vector<Face *> faces;
    std::vector<Tetrahedron *> tetrahedrons;
    int numThreads;
    std::array<float, 3> target;
    Tetrahedron * targetTet;
    //unsigned int cores;
  public:
    Mesh (const int numVertices, const int numFaces, const int numCells, const int _numThreads);
    void setVertices(const std::vector<std::array<float, 3>> & _vertices);
    void addTetrahedron(const int id, const std::array<int, 4> vertexIds, const std::vector<int> neighborIds, const float weight);
    bool setTarget(const std::array<float, 3> _target);
    //this function is used for testing; it is unlikely that it will be useful in python code
    int getTargetTetId();
    //std::vector<Vertex3d *> Vertices();
};

class Vertex3d {
    //float x, y, z;
    std::array<float, 3> vec;
    std::vector<Tetrahedron *> tetrahedrons;
  public:
    Vertex3d (std::array<float, 3>);
    std::array<float, 3> Vec();
};

/*
class Face {
    std::array<Vertex3d *, 3> vertices;
    Tetrahedron * tetrahedron;
  public:
    Face ();
};*/

class Tetrahedron {
    int id;
    std::array<Vertex3d *, 4> vertices;
    std::vector<Tetrahedron *> neighbors;
    std::vector<int> threads;
    std::array<float, 3> sphereCenter;
    float sphereRadius;
    float weight;
  public:
    Tetrahedron (const int id, const std::array<Vertex3d *, 4> _vertices, const float 	  _weight, const int numThreads);
    void addNeighbor(Tetrahedron * neighbor);
    bool contains(const std::array<float, 3>);
    int getId();
};
