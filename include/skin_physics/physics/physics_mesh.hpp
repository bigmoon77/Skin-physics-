#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::physics
{
    struct tetrahedron
    {
        std::array<uint32_t, 4> vertices = {};
        std::array<float, 9> rest_inverse = {};
        std::array<asset::vec3, 4> rest_shape_gradients = {};
        float rest_volume = 0.0f;
        float quality = 0.0f;
        float minimum_dihedral_degrees = 0.0f;
    };

    struct boundary_triangle
    {
        std::array<uint32_t, 3> vertices = {};
        uint32_t source_tetrahedron = asset::invalid_index;
    };

    struct raw_tet_mesh
    {
        std::vector<asset::vec3> positions;
        std::vector<std::array<uint32_t, 4>> tetrahedra;
    };

    struct physics_mesh
    {
        std::vector<asset::vec3> rest_positions;
        std::vector<tetrahedron> tetrahedra;
        std::vector<boundary_triangle> boundary_triangles;

        // CSR: tetrahedra incident to a vertex.
        std::vector<uint32_t> vertex_tetrahedron_offsets;
        std::vector<uint32_t> vertex_tetrahedron_ids;

        std::vector<float> mass;
        std::vector<float> inverse_mass;
    };

    enum class physics_mesh_error : uint8_t
    {
        none,
        empty_mesh,
        invalid_density,
        invalid_index,
        invalid_topology,
        degenerate_tetrahedron,
        non_manifold_volume,
        capacity_exceeded,
        non_finite_value,
        invalid_quality
    };

    struct physics_mesh_diagnostic
    {
        physics_mesh_error code = physics_mesh_error::none;
        uint32_t element_index = asset::invalid_index;
        std::string message;
    };

    struct physics_mesh_build_settings
    {
        float density = 1000.0f;
        float minimum_volume = 1.0e-12f;
        float minimum_quality = 1.0e-5f;
        float minimum_dihedral_degrees = 0.0f;
        uint32_t maximum_tetrahedra = 1'000'000;
    };

    struct physics_mesh_build_result
    {
        physics_mesh value;
        std::vector<physics_mesh_diagnostic> diagnostics;
        bool success = false;
    };
}
