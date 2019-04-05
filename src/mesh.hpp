#include <array>
#include <vector>
#include "plane3d.hpp"
#include "mesh_components.hpp"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#ifndef MESH_HPP
#define MESH_HPP

class Mesh {
    vector<Vertex3d> vertices;
    vector<Tetrahedron> tetrahedrons;
    vector<Face> faces;
    Vector3f target;
    unsigned long int targetTetId;
    /*//used for testing
    vector<int> sliceIds;*/
  public:
    Mesh (const int numVertices, const int numFaces, const int numCells);
    //~Mesh();
    void setVertices(const vector<array<float, 3>> & _vertices);
    void addTetrahedron(const int id, const array<int, 4> vertexIds, const vector<int> neighborIds, const float weight);
    void addFace(const array<int, 3> vertexIds, const int tetId);
    bool setTarget(const array<float, 3> _target);
    /*vector<Tetrahedron *> findIntersectingOuterTets(Plane3d plane);
    vector<vector<array<float, 3>>> slice(array<float, 2> rotations);
    void findPaths(const int epsilon, const int numThreads);*/
    void computeManySlices(vector<array<float, 2>> planeVector);
    //these functions are exposed to Python for testing purposes; while it is possible that a python application may require them, they are not required for any other purpose in this application
    unsigned long int getTargetTetId();
    //vector<int> getSliceIds();
};
#endif

