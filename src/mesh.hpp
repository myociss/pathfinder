#include <array>
#include <vector>
#include "plane3d.hpp"
#include "found_path.hpp"
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
    Vector3d target;
    unsigned long int targetTetId;
    //used for testing
    vector<int> sliceIds;
  public:
    Mesh (const int numVertices, const int numFaces, const int numCells);
    void setVertices(const vector<array<double, 3>> & _vertices);
    void addTetrahedron(const int id, const array<int, 4> vertexIds, const vector<unsigned long int> neighborIds, const double weight, const int label);
    void addFace(const array<int, 3> vertexIds, const int tetId);
    bool setTarget(const array<double, 3> _target);
    vector<Shape3d> slice(Plane3d plane, vector<int> &tetsChecked);
    vector<FoundPath> shortestPaths(const int epsilon, const int numThreads, double distBound);
    vector<Shape3d> computeSliceComponent(Plane3d plane, vector<int> &tetsChecked, unsigned long int initTet);
    vector<Shape3d> sliceIndv(array<double, 2> rotation);
    vector<FoundPath> findPaths(vector<Plane3d> planes, double distBound);
    //these functions are exposed to Python for testing purposes; while it is possible that a python application may require them, they are not required for any other purpose in this application
    unsigned long int getTargetTetId();
};

#endif

