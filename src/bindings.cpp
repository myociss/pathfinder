#include <pybind11/pybind11.h>
#include "graph.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pathfinder, m){
  py::class_<graph>(m, "graph")
    .def(py::init<int, int>())
    .def("numVertices", &graph::numVertices);
}
