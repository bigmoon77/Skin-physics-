#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    struct msh_read_diagnostic
    {
        uint32_t line = 0;
        std::string message;
    };

    struct msh_read_result
    {
        raw_tet_mesh mesh;
        std::vector<msh_read_diagnostic> diagnostics;
        bool success = false;
    };

    msh_read_result read_gmsh_2_2_ascii(const std::filesystem::path& path);
}
