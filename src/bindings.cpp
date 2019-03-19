#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "graph.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pathfinder, m){
  py::class_<Graph>(m, "Graph")
    .def(py::init<const std::vector<std::array<float, 3>> &>());
    //.def("numVertices", &Graph::numVertices);
}
