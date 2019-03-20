#include <array>
#include <vector>

class Vertex3d;
class Face;
class Tetrahedron;

class Graph {
    std::vector<Vertex3d *> vertices;
    std::vector<Face *> faces;
    std::vector<Tetrahedron *> tetrahedrons;
  public:
    Graph (const int numVertices, const int numFaces, const int numCells);
    void setVertices(const std::vector<std::array<float, 3>> & _vertices);
    void addTetrahedron(const std::array<int, 3> vertexIds, 
	const std::vector<int> neighborIds, const float weight);
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

class Face {
    std::array<Vertex3d *, 3> vertices;
    Tetrahedron * tetrahedron;
  public:
    Face ();
};

class Tetrahedron {
    std::array<Vertex3d *, 3> vertices;
    std::vector<Tetrahedron *> neighbors;
    float weight;
  public:
    Tetrahedron (const std::array<Vertex3d *, 4> _vertices, const float 	  _weight);
    void addNeighbor(Tetrahedron * neighbor);
};
