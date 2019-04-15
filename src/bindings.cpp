#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "plane3d.hpp"
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
	py::arg("weight"))
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

  py::class_<Shape3d>(m, "Shape3d")
    .def(py::init<unsigned long int, vector<array<double, 3>>, double>(),
	py::arg("tet_id"),
	py::arg("points"),
	py::arg("weight"))
    .def("tet_id", &Shape3d::TetId);
}
