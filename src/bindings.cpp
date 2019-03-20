#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "graph.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pathfinder, m){
  py::class_<Graph>(m, "Graph")
    .def(py::init<const int,const int,const int>(), 
	py::arg("num_vertices"), 
	py::arg("num_faces"),
	py::arg("num_tetrahedrons"))
    .def("set_vertices", &Graph::setVertices)
    .def("add_tetrahedron", &Graph::addTetrahedron,
	py::arg("vertex_ids"),
	py::arg("neighbor_ids"),
	py::arg("weight"));

}
