#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "mesh.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pathfinder, m){
  py::class_<Mesh>(m, "Mesh")
    .def(py::init<const int,const int,const int, const int>(), 
	py::arg("num_vertices"), 
	py::arg("num_faces"),
	py::arg("num_tetrahedrons"),
	py::arg("num_threads"))
    .def("set_vertices", &Mesh::setVertices)
    .def("add_tetrahedron", &Mesh::addTetrahedron,
	py::arg("vertex_ids"),
	py::arg("neighbor_ids"),
	py::arg("weight"));

}
