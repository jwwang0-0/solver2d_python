// bindings/bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>

extern "C" {
  #include "solver2d/solver2d.h"  
  #include "solver2d/world.h" 
  #include "solver2d/solvers.h" 
}

namespace py = pybind11;

PYBIND11_MODULE(solver2d_py, m) {
    m.doc() = "Tiny test binding for solver2d: create -> TGS_Soft step -> destroy";
    std::cout << "[bindings] solver2d_py loaded!" << std::endl;

    // Expose the handle type so pybind11 can pass it around.
    py::class_<s2WorldId>(m, "WorldId")
        // If it's an aggregate with public fields, you can expose them:
        // .def_readwrite("index", &s2WorldId::index)
        // .def_readwrite("revision", &s2WorldId::revision)
        // If you don't know/need the fields, at least make it constructible:
        .def(py::init<>())
        .def("__repr__", [](const s2WorldId& self) {
            return "<solver2d_py.WorldId>";
        });

    // Create an empty world and return it as an integer handle.
    m.def("create_world", []() -> s2WorldId {
        s2WorldDef worldDef = s2DefaultWorldDef();
        worldDef.solverType = s2_solverTGS_Soft;
        s2WorldId id = s2CreateWorld(&worldDef);
        return id;
    });

    // Run one TGS_Soft step with a bare-minimum context.
    m.def("step_tgs_soft", [](s2WorldId id,
                              float dt,
                              int iterations,
                              int pos_iterations,
                              bool warm_start) {
        s2World_Step(id, dt, iterations, pos_iterations, warm_start);
    }, py::arg("world_id"), py::arg("dt") = 1.0f/60.0f, py::arg("iterations") = 20, py::arg("pos_iterations") = 20, py::arg("warm_start") = true);

    // Destroy the world.
    m.def("destroy_world", [](s2WorldId id) {
        s2DestroyWorld(id);        
    });
}