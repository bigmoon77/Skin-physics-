#pragma once

#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    physics_mesh_build_result build_physics_mesh(
        const raw_tet_mesh& input,
        const physics_mesh_build_settings& settings = {});
}
