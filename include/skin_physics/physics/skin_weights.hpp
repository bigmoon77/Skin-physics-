#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"
#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    enum class skin_weight_error : uint8_t
    {
        none,
        invalid_surface_mesh,
        invalid_skin,
        invalid_physics_mesh,
        no_valid_surface_triangles,
        boundary_match_too_far,
        disconnected_interior_component,
        solver_not_converged,
        invalid_result
    };

    struct skin_weight_diagnostic
    {
        skin_weight_error code = skin_weight_error::none;
        uint32_t element_index = asset::invalid_index;
        std::string message;
    };

    struct skin_weight_transfer_settings
    {
        // Maximum allowed distance from a Physics boundary vertex to the Render Mesh.
        float maximum_surface_distance = 0.01f;
        uint32_t maximum_iterations = 512;
        float relative_residual_tolerance = 1.0e-5f;
    };

    struct skin_weight_transfer_result
    {
        std::vector<skin_weight_diagnostic> diagnostics;
        uint32_t maximum_iterations_used = 0;
        float maximum_relative_residual = 0.0f;
        bool success = false;
    };

    // Transfers Render Mesh skin weights to Physics Mesh boundary vertices by
    // closest-triangle barycentric interpolation, then harmonically extends
    // them to interior vertices using a positive graph Laplacian.
    skin_weight_transfer_result transfer_skin_weights(
        const asset::mesh& render_mesh,
        const asset::skin& render_skin,
        physics_mesh& target_mesh,
        const skin_weight_transfer_settings& settings = {});
}
