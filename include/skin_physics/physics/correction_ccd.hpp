#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/physics/collision.hpp"

namespace skin_physics::physics
{
    enum class correction_ccd_status : uint8_t
    {
        no_hit,
        hit,
        path_crossing,
        unresolved,
        invalid_input
    };

    struct correction_vertex_motion
    {
        asset::vec3 before;
        asset::vec3 after;
    };

    struct correction_ccd_settings
    {
        float contact_thickness = 0.01f;
        float distance_tolerance = 1.0e-5f;
        float epsilon_length = 1.0e-7f;
        uint32_t maximum_interval_evaluations = 32;
    };

    enum class correction_contact_type : uint8_t
    {
        vertex_triangle,
        edge_edge
    };

    struct correction_candidate_pair_key
    {
        correction_contact_type contact_type = correction_contact_type::vertex_triangle;
        uint64_t stable_id_a = 0;
        uint32_t mesh_a = 0;
        uint32_t primitive_a = 0;
        uint64_t stable_id_b = 0;
        uint32_t mesh_b = 0;
        uint32_t primitive_b = 0;

        bool operator==(const correction_candidate_pair_key& other) const;
        bool operator<(const correction_candidate_pair_key& other) const;
    };

    enum class correction_primitive_role : uint8_t
    {
        vertex,
        triangle,
        edge
    };

    struct correction_broadphase_primitive
    {
        uint64_t stable_id = 0;
        uint32_t mesh_id = 0;
        uint32_t primitive_id = 0;
        correction_primitive_role role = correction_primitive_role::vertex;
        collision_aabb envelope_bounds;
    };

    struct correction_broadphase_result
    {
        std::vector<correction_candidate_pair_key> candidates;
        std::vector<collision_diagnostic> diagnostics;
        std::size_t required_cell_records = 0;
        std::size_t broadphase_candidate_count = 0;
        std::size_t required_candidates = 0;
        bool overflow = false;
        bool success = false;
    };

    struct correction_candidate_union_result
    {
        std::vector<correction_candidate_pair_key> candidates;
        std::size_t required_count = 0;
        bool overflow = false;
        bool success = false;
    };

    struct correction_ccd_result
    {
        correction_ccd_status status = correction_ccd_status::invalid_input;
        bool hit_witness_valid = false;
        float witness_s = 0.0f;
        float witness_distance_lower = 0.0f;
        float witness_distance_upper = 0.0f;
        asset::vec3 witness_point_a;
        asset::vec3 witness_point_b;
        asset::vec3 witness_normal;
        std::array<float, 3> witness_barycentric = {};
        float witness_parameter_a = 0.0f;
        float witness_parameter_b = 0.0f;
        asset::vec3 reference_normal;
        int8_t allowed_side_sign = 0;
        uint32_t interval_evaluations = 0;
        std::string diagnostic;
    };

    struct vertex_triangle_correction_motion
    {
        correction_vertex_motion point;
        std::array<correction_vertex_motion, 3> triangle;
        asset::vec3 segment_start_point;
        std::array<asset::vec3, 3> segment_start_triangle;
        // Set true only after Contact Key, Topology and Patch Generation match.
        bool has_valid_persistent_contact_cache = false;
        asset::vec3 persistent_contact_normal;
    };

    struct edge_edge_correction_motion
    {
        std::array<correction_vertex_motion, 2> edge_a;
        std::array<correction_vertex_motion, 2> edge_b;
        std::array<asset::vec3, 2> segment_start_edge_a;
        std::array<asset::vec3, 2> segment_start_edge_b;
        // Set true only after Contact Key, Topology and Patch Generation match.
        bool has_valid_persistent_contact_cache = false;
        asset::vec3 persistent_contact_normal;
    };

    correction_candidate_pair_key make_canonical_correction_candidate_key(
        correction_contact_type contact_type,
        uint64_t stable_id_a,
        uint32_t mesh_a,
        uint32_t primitive_a,
        uint64_t stable_id_b,
        uint32_t mesh_b,
        uint32_t primitive_b);

    correction_candidate_union_result merge_correction_candidate_union(
        const std::vector<correction_candidate_pair_key>& existing_candidates,
        const std::vector<correction_candidate_pair_key>& incoming_candidates,
        std::size_t capacity);

    // Input envelope bounds must already include the per-primitive Contact Thickness expansion.
    // settings.contact_thickness must be zero so the existing Broadphase does not expand twice.
    correction_broadphase_result generate_correction_broadphase_candidates(
        const std::vector<correction_broadphase_primitive>& primitives,
        const std::vector<broadphase_mesh_pair>& mesh_pairs,
        const broadphase_settings& settings = {});

    bool union_correction_envelope_aabb(
        const collision_aabb& first,
        const collision_aabb& second,
        collision_aabb& output_bounds);

    bool build_correction_swept_aabb(
        const std::vector<correction_vertex_motion>& vertex_motions,
        float contact_thickness,
        collision_aabb& output_bounds);

    correction_ccd_result detect_vertex_triangle_correction_motion(
        const vertex_triangle_correction_motion& motion,
        const correction_ccd_settings& settings = {});

    correction_ccd_result detect_edge_edge_correction_motion(
        const edge_edge_correction_motion& motion,
        const correction_ccd_settings& settings = {});
}
