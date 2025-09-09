// bindings/bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>

extern "C" {
  #include "solver2d/solver2d.h"  
  #include "solver2d/world.h" 
  #include "solver2d/solvers.h" 
  #include "solver2d/body.h"
  #include "solver2d/shape.h"
  #include "solver2d/types.h"
  #include "solver2d/geometry.h"
  #include "solver2d/math.h"
  #include "solver2d/hull.h"
}

namespace py = pybind11;

struct Pose { float x, y, angle; };
struct Velocity { float vx, vy, omega; };

static inline void s2_make_box_polygon(float hx, float hy, s2Polygon* poly)
{
    poly->count = 4;
    poly->vertices[0] = { -hx, -hy };
    poly->vertices[1] = { +hx, -hy };
    poly->vertices[2] = { +hx, +hy };
    poly->vertices[3] = { -hx, +hy };
    // CCW around the local origin
}

PYBIND11_MODULE(solver2d_py, m) {
    m.doc() = "Tiny test binding for solver2d: create -> TGS_Soft step -> destroy";
    std::cout << "[bindings] solver2d_py loaded!" << std::endl;

    ////////////Define all types can be used in python//////////////
    py::class_<s2WorldId>(m, "WorldId")
        .def(py::init<>())
        .def("__repr__", [](const s2WorldId& self) {
            return "<solver2d_py.WorldId>";
        });

    py::class_<s2BodyId>(m, "BodyId")
        .def(py::init<>())
        .def("__repr__", [](const s2BodyId&) { return "<solver2d_py.BodyId>"; });

    py::class_<Pose>(m, "Pose")
        .def_readonly("x", &Pose::x)
        .def_readonly("y", &Pose::y)
        .def_readonly("angle", &Pose::angle);
    
    py::class_<Velocity>(m, "Velocity")
        .def_readonly("vx", &Velocity::vx)
        .def_readonly("vy", &Velocity::vy)
        .def_readonly("omega", &Velocity::omega);

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
    }, py::arg("world_id"), py::arg("dt") = 1.0f/60.0f, py::arg("iterations") = 4, py::arg("pos_iterations") = 2, py::arg("warm_start") = true);

    // Destroy the world.
    m.def("destroy_world", [](s2WorldId id) {
        s2DestroyWorld(id);        
    });

    m.def("add_ground", [](s2WorldId world,
                        float friction) {
            // 1) Create body
            float extent = 1.0f;
            s2BodyDef bd = s2_defaultBodyDef;
            s2BodyId groundId = s2CreateBody(world, &bd);
            
            float groundWidth = 66.0f * extent;
            s2ShapeDef shapeDef = s2_defaultShapeDef;
            shapeDef.friction = friction;
            s2Segment segment = {{-0.5f * 2.0f * groundWidth, 0.0f}, {0.5f * 2.0f * groundWidth, 0.0f}};
            s2CreateSegmentShape(groundId, &shapeDef, &segment);
        },
        py::arg("world_id"),
        py::arg("friction") = 0.5f);

    m.def("add_box", [](s2WorldId world,
                        float cx, float cy,        // body position in world
                        float hx, float hy,       // half of x and y dimensions
                        float angle,               //radians
                        float density,            
                        float friction) -> s2BodyId           // optional material
                        
        {
            // 1) Create bodydef, it contain the information of
            // of body is dynamic or not
            // and body location
            s2BodyDef bd = s2_defaultBodyDef;
            bd.type = s2_dynamicBody;
            bd.position = { cx, cy };
            bd.angle = angle;

            // 2) Body shape defination 
            // include information of friction/ 
            s2ShapeDef shapeDef = s2_defaultShapeDef;
            shapeDef.friction = friction;
            shapeDef.density = density;

            // 3) Body real shape definition
            s2Polygon box = s2MakeBox(hx, hy);

            // create body
            s2BodyId bodyID = s2CreateBody(world, &bd);
            s2CreatePolygonShape(bodyID, &shapeDef, &box);

            return bodyID;
        },
        py::arg("world_id"),
        py::arg("cx"), py::arg("cy"), 
        py::arg("hx"), py::arg("hy"),
        py::arg("angle") = 0.0f,
        py::arg("density") = 1.0f,
        py::arg("friction") = 0.5f);

m.def("add_4pt_polygon",[](s2WorldId world,
                        float cx, float cy,
                        const std::vector<std::array<float,2>>& verts,  // local CCW vertices
                        float angle,               //radians
                        float density,
                        float friction) -> s2BodyId
        {
            // 1) Create bodydef, it contain the information of
            // of body is dynamic or not
            // and body location
            s2BodyDef bd = s2_defaultBodyDef;
            bd.type = s2_dynamicBody;
            bd.position = { cx, cy };
            bd.angle = angle;

            // 2) Body shape defination 
            // include information of friction/         
            s2ShapeDef sd = s2_defaultShapeDef;
            sd.density = density;
            sd.friction = friction;

            // 3) Body real shape definition
            // s2Polygon shape = {0};
            const int n = 4;

            // // Safety: respect max polygon vertices if defined
            #ifdef s2_maxPolygonVertices
            if ((int)verts.size() != 4) throw std::runtime_error("Not 4 points");
            #endif

            s2Vec2 points[n];
            for (int i = 0; i < n; ++i) {
                points[i] = { static_cast<float>(verts[i][0]),
                            static_cast<float>(verts[i][1]) };
            }
            s2Hull hull = s2ComputeHull(points, n);
            s2Polygon polygon = s2MakePolygon(&hull);

            // NOTE: verts must be convex + CCW around local origin.
            s2BodyId body = s2CreateBody(world, &bd);
            s2CreatePolygonShape(body, &sd, &polygon);
            return body;
        },
        py::arg("world_id"),
        py::arg("cx"), py::arg("cy"),
        py::arg("verts"),
        py::arg("angle") = 0.0f,
        py::arg("density") = 1.0f,
        py::arg("friction") = 0.5f);

    m.def("get_body_pose",[](s2BodyId bodyID) -> Pose
        {
            // s2World* world = s2GetWorldFromIndex(bodyID.world);
            // s2Body* body = s2GetBody(world, bodyID);
            s2Vec2 pos = s2Body_GetPosition(bodyID);
            float angle = s2Body_GetAngle(bodyID);
            Pose p{ pos.x, pos.y, angle};
            return p;
        }, py::arg("body_id"));
    
    m.def("get_body_velocity", [](s2BodyId bodyID) -> Velocity {
        s2Vec2 v = s2Body_GetLinearVelocity(bodyID);   
        float w  = s2Body_GetAngularVelocity(bodyID);  
        return Velocity{ v.x, v.y, w };
        }, py::arg("body_id"));
}