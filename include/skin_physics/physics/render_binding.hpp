#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"
#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    enum class render_binding_kind : uint8_t
    {
        tetrahedron,
        boundary_triangle
    };

    struct render_vertex_binding
    {
        render_binding_kind kind = render_binding_kind::tetrahedron;
        std::array<uint32_t, 4> physics_vertices = {};
        std::array<float, 4> barycentric = {};
        asset::vec3 rest_offset_local;
        float rest_error = 0.0f;
    };

    enum class render_binding_error : uint8_t
    {
        none,
        invalid_render_mesh,
        invalid_physics_mesh,
        invalid_settings,
        no_surface_embedding,
        excessive_rest_error,
        invalid_binding,
        degenerate_deformed_triangle
    };

    struct render_binding_diagnostic
    {
        render_binding_error code = render_binding_error::none;
        uint32_t render_vertex = asset::invalid_index;
        std::string message;
    };

    struct render_binding_settings
    {
        float maximum_surface_distance = 0.25f;
        float maximum_rest_error_relative_to_bbox_diagonal = 1.0e-3f;
        float barycentric_tolerance = 1.0e-5f;
    };

    struct render_binding_build_result
    {
        std::vector<render_vertex_binding> bindings;
        std::vector<render_binding_diagnostic> diagnostics;
        float maximum_rest_error = 0.0f;
        float maximum_relative_rest_error = 0.0f;
        bool success = false;
    };

    struct render_feedback_result
    {
        asset::mesh value;
        std::vector<render_binding_diagnostic> diagnostics;
        bool success = false;
    };

    using physics_vertex_attribute_weights = std::array<float,
        static_cast<std::size_t>(asset::physics_attribute_channel::count)>;

    struct physics_attribute_transfer_settings
    {
        float maximum_surface_distance = 0.25f;
        float ambiguity_tolerance = 1.0e-4f;
        double harmonic_relative_tolerance = 1.0e-6;
        uint32_t harmonic_maximum_iterations = 20000u;
    };

    struct physics_attribute_transfer_result
    {
        std::vector<physics_vertex_attribute_weights> vertex_weights;
        float maximum_surface_distance = 0.0f;
        double maximum_harmonic_relative_residual = 0.0;
        std::string diagnostic;
        bool success = false;
    };

    physics_attribute_transfer_result transfer_physics_vertex_attribute_weights(
        const asset::mesh& render_mesh,
        const physics_mesh& physics_value,
        const physics_attribute_transfer_settings& settings = {});

    render_binding_build_result build_render_vertex_bindings(
        const asset::mesh& render_mesh,
        const physics_mesh& physics_value,
        const render_binding_settings& settings = {});

    // Copies the source mesh and changes only positions plus present normal/tangent streams.
    // UVs, colors, material assignments, indices and other vertex attributes remain unchanged.
    render_feedback_result apply_render_feedback(
        const asset::mesh& source_mesh,
        const physics_mesh& physics_value,
        const std::vector<render_vertex_binding>& bindings);
}
