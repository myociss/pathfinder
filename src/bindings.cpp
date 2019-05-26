#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "plane3d.hpp"
#include "plane2d.hpp"
#include "shape2d.hpp"
#include "mesh.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pathfinder, m){
  py::class_<Mesh>(m, "Mesh")
    .def(py::init<const int,const int,const int>(), 
	py::arg("num_vertices"), 
	py::arg("num_faces"),
	py::arg("num_tetrahedrons"))
    .def("set_vertices", &Mesh::setVertices)
    .def("add_tetrahedron", &Mesh::addTetrahedron,
	py::arg("tetrahedron_id"),
	py::arg("vertex_ids"),
	py::arg("neighbor_ids"),
	py::arg("weight"),
	py::arg("label"))
    .def("add_face", &Mesh::addFace,
	py::arg("vertex_ids"),
	py::arg("tetrahedron_id"))
    .def("set_target", &Mesh::setTarget)
    .def("get_target_idx", &Mesh::getTargetTetId)
    .def("slice", &Mesh::sliceIndv,
	py::arg("rotation"))
    .def("multiple_slices", &Mesh::shortestPaths,
	py::arg("epsilon"),
	py::arg("threads"));

  py::class_<Plane3d>(m, "Plane3d")
    .def(py::init<int, double, double, Eigen::Vector3d>(),
	py::arg("id"),
	py::arg("alpha"),
	py::arg("theta"),
	py::arg("target"));

  py::class_<Shape3d>(m, "Shape3d")
    .def(py::init<unsigned long int, vector<array<double, 3>>, double, int>(),
	py::arg("tet_id"),
	py::arg("points"),
	py::arg("weight"),
	py::arg("label"))
    .def("tet_id", &Shape3d::TetId)
    .def("vertices", &Shape3d::Vertices)
    .def("label", &Shape3d::Label);

  py::class_<Plane2d>(m, "Plane2d")
    .def(py::init<vector<Shape3d>&, Plane3d>())
    .def("shapes", &Plane2d::Shapes);

  py::class_<Shape2d>(m, "Shape2d")
    .def("vertices", &Shape2d::Vertices)
    .def("arranged_vertices", &Shape2d::VerticesArranged)
    .def("hull_supporting_idx", &Shape2d::EndVertex);
}
