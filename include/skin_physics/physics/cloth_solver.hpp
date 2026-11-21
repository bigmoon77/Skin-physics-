#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::physics
{
    struct raw_cloth_mesh
    {
        std::vector<asset::vec3> rest_positions;
        std::vector<std::array<uint32_t, 3>> triangles;
    };

    struct cloth_stretch_constraint
    {
        std::array<uint32_t, 2> vertices = {};
        float rest_length = 0.0f;
    };

    struct cloth_shear_constraint
    {
        std::array<uint32_t, 3> vertices = {};
        float rest_dot = 0.0f;
        float normalization_length = 0.0f;
    };

    struct cloth_bend_constraint
    {
        std::array<uint32_t, 4> vertices = {};
        float rest_angle = 0.0f;
    };

    struct cloth_stretch_color_range
    {
        uint32_t first_constraint_index = 0u;
        uint32_t constraint_count = 0u;
    };

    enum class cloth_vertex_constraint_type : uint32_t
    {
        stretch = 0u,
        shear = 1u,
        bend = 2u
    };

    struct cloth_vertex_constraint_ref
    {
        uint32_t type = 0u;
        uint32_t constraint_index = 0u;
        uint32_t local_vertex_index = 0u;
        uint32_t reserved = 0u;
    };

    struct cloth_mesh
    {
        std::vector<asset::vec3> rest_positions;
        std::vector<std::array<uint32_t, 3>> triangles;
        std::vector<cloth_stretch_constraint> stretch_constraints;
        std::vector<cloth_stretch_color_range> stretch_color_ranges;
        // Flattened by color; each color is a vertex-disjoint write batch.
        std::vector<uint32_t> stretch_color_constraint_indices;
        std::vector<cloth_vertex_constraint_ref> vertex_constraint_refs;
        std::vector<uint32_t> vertex_constraint_offsets;
        std::vector<cloth_shear_constraint> shear_constraints;
        std::vector<cloth_bend_constraint> bend_constraints;
    };

    enum class cloth_error : uint8_t
    {
        none,
        empty_mesh,
        invalid_index,
        degenerate_triangle,
        non_manifold_edge,
        invalid_settings,
        mismatched_state,
        non_finite_state,
        degenerate_constraint
    };

    struct cloth_diagnostic
    {
        cloth_error code = cloth_error::none;
        uint32_t element_index = asset::invalid_index;
        std::string message;
    };

    struct cloth_build_result
    {
        cloth_mesh value;
        std::vector<cloth_diagnostic> diagnostics;
        bool success = false;
    };

    struct cloth_solver_settings
    {
        float stretch_compliance = 0.0f;
        float shear_compliance = 0.0f;
        float bend_compliance = 0.0f;
        float fixed_delta_time = 1.0f / 60.0f;
        float relaxation = 1.0f;
        uint32_t iterations = 8;
        bool solve_stretch = true;
        bool solve_shear = true;
        bool solve_bend = true;
    };

    // Explicit Shader Model 5 StructuredBuffer records. These records are
    // separate from the CPU topology structs so GPU layout changes remain
    // deliberate and compile-time checked.
    struct cloth_gpu_stretch_constraint_record
    {
        std::array<uint32_t, 2> vertices = {};
        float rest_length = 0.0f;
        float reserved = 0.0f;
    };

    struct cloth_gpu_constraint_record
    {
        std::array<uint32_t, 4> indices = {};
        uint32_t type = 0u;
        float rest_value = 0.0f;
        float normalization = 0.0f;
        float compliance = 0.0f;
    };

    static_assert(std::is_standard_layout_v<cloth_gpu_stretch_constraint_record>);
    static_assert(std::is_trivially_copyable_v<cloth_gpu_stretch_constraint_record>);
    static_assert(sizeof(cloth_gpu_stretch_constraint_record) == 16u,
        "Cloth Stretch GPU constraint ABI must remain 16 bytes.");
    static_assert(offsetof(cloth_gpu_stretch_constraint_record, rest_length) == 8u);
    static_assert(std::is_standard_layout_v<cloth_gpu_constraint_record>);
    static_assert(std::is_trivially_copyable_v<cloth_gpu_constraint_record>);
    static_assert(sizeof(cloth_gpu_constraint_record) == 32u,
        "Cloth Jacobi GPU constraint ABI must remain 32 bytes.");
    static_assert(offsetof(cloth_gpu_constraint_record, type) == 16u);
    static_assert(offsetof(cloth_gpu_constraint_record, compliance) == 28u);
    static_assert(sizeof(cloth_vertex_constraint_ref) == 16u,
        "Cloth Vertex-to-Constraint CSR reference ABI must remain 16 bytes.");

    struct cloth_gpu_topology_packet
    {
        uint32_t vertex_count = 0u;
        std::vector<cloth_gpu_stretch_constraint_record> stretch_constraints;
        std::vector<cloth_stretch_color_range> stretch_color_ranges;
        std::vector<uint32_t> stretch_color_constraint_indices;
        std::vector<cloth_gpu_constraint_record> shear_constraints;
        std::vector<cloth_gpu_constraint_record> bend_constraints;
        std::vector<uint32_t> vertex_constraint_offsets;
        std::vector<cloth_vertex_constraint_ref> vertex_constraint_refs;
        // Lambda is Segment-local. These arrays are initialized to zero and
        // must be reset at each new Segment, then retained across its iterations.
        std::vector<float> stretch_lambdas;
        std::vector<float> shear_lambdas;
        std::vector<float> bend_lambdas;
    };

    enum class cloth_gpu_topology_error : uint8_t
    {
        none,
        invalid_mesh,
        invalid_settings,
        invalid_coloring,
        invalid_csr,
        capacity_exceeded
    };

    struct cloth_gpu_topology_build_result
    {
        cloth_gpu_topology_packet value;
        cloth_gpu_topology_error error = cloth_gpu_topology_error::none;
        std::string message;
        bool success = false;
    };

    // Builds a transactional, GPU-layout-checked topology packet from validated
    // CPU Cloth data. Dynamic Positions / inverse masses and Segment-local
    // Lambda resets remain Runtime-owned resources.
    cloth_gpu_topology_build_result build_cloth_gpu_topology_packet(
        const cloth_mesh& mesh,
        const cloth_solver_settings& settings = {});

    struct cloth_bend_evaluation
    {
        float constraint = 0.0f;
        std::array<asset::vec3, 4> gradients = {};
        bool valid = false;
    };

    struct cloth_step_result
    {
        std::vector<asset::vec3> positions;
        std::vector<cloth_diagnostic> diagnostics;
        uint32_t iterations_completed = 0;
        bool success = false;
    };

    struct cloth_stretch_color_step_result
    {
        std::vector<asset::vec3> positions;
        std::vector<float> lambda_values;
        std::vector<cloth_diagnostic> diagnostics;
        bool success = false;
    };

    cloth_build_result build_cloth_mesh(const raw_cloth_mesh& input);

    // Validates deterministic Stretch graph-color batches before GPU upload.
    // Every constraint must occur exactly once and no two constraints in one
    // color may write the same vertex.
    bool validate_cloth_stretch_coloring(
        const cloth_mesh& mesh,
        std::string& diagnostic);

    // Validates the deterministic Vertex-to-Constraint CSR incidence table.
    bool validate_cloth_vertex_constraint_csr(
        const cloth_mesh& mesh,
        std::string& diagnostic);

    // CPU reference for one in-place, vertex-disjoint Stretch color dispatch.
    // Lambda values are Segment-local and must be preserved across iterations.
    cloth_stretch_color_step_result solve_cloth_stretch_color(
        const cloth_mesh& mesh,
        uint32_t color_index,
        const std::vector<asset::vec3>& current_positions,
        const std::vector<float>& inverse_mass,
        const std::vector<float>& lambda_values,
        float delta_time,
        float compliance,
        float relaxation);

    cloth_bend_evaluation evaluate_cloth_bend(
        const std::array<asset::vec3, 4>& positions,
        float rest_angle);

    cloth_step_result solve_cloth(
        const cloth_mesh& mesh,
        const std::vector<asset::vec3>& current_positions,
        const std::vector<float>& inverse_mass,
        const cloth_solver_settings& settings = {});
}
