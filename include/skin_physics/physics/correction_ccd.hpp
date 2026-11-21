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
        std::size_t contact_set_capacity = 1u << 20;
        std::size_t vertex_contact_csr_capacity = 1u << 22;
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

    struct correction_primitive_identity
    {
        uint64_t stable_id = 0;
        uint32_t mesh_id = 0;
        uint32_t primitive_id = 0;

        bool operator==(const correction_primitive_identity& other) const;
        bool operator<(const correction_primitive_identity& other) const;
    };

    struct correction_broadphase_primitive
    {
        uint64_t stable_id = 0;
        uint32_t mesh_id = 0;
        uint32_t primitive_id = 0;
        correction_primitive_role role = correction_primitive_role::vertex;
        collision_aabb envelope_bounds;
        std::vector<uint64_t> vertex_stable_ids;
        // Explicit one-ring / adjacency exclusions supplied by the topology owner.
        std::vector<correction_primitive_identity> excluded_primitive_ids;
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

    struct correction_discrete_contact_input
    {
        correction_candidate_pair_key key;
        std::array<uint64_t, 4> vertex_stable_ids = {};
        std::array<asset::vec3, 4> positions = {};
        float contact_thickness = 0.0f;
    };

    struct correction_discrete_contact
    {
        correction_candidate_pair_key key;
        std::array<uint64_t, 4> participant_vertex_stable_ids = {};
        vertex_triangle_contact vertex_triangle;
        edge_edge_contact edge_edge;
        bool active = false;
    };

    struct correction_vertex_contact_csr
    {
        std::vector<uint64_t> vertex_stable_ids;
        std::vector<uint32_t> offsets;
        std::vector<uint32_t> contact_indices;
        uint64_t candidate_generation = 0;
        uint64_t position_writer_epoch_generation = 0;
        bool current = false;
    };

    struct correction_contact_set_refresh_result
    {
        std::size_t candidate_count = 0;
        std::size_t active_contact_count = 0;
        uint64_t candidate_generation = 0;
        uint64_t position_writer_epoch_generation = 0;
        std::size_t required_contact_count = 0;
        std::size_t required_vertex_contact_indices = 0;
        bool generation_mismatch = false;
        bool invalid_input = false;
        bool capacity_overflow = false;
        bool success = false;
        std::string diagnostic;
    };

    struct correction_primitive_epoch_input
    {
        uint64_t stable_id = 0;
        uint32_t mesh_id = 0;
        uint32_t primitive_id = 0;
        correction_primitive_role role = correction_primitive_role::vertex;
        std::vector<uint64_t> vertex_stable_ids;
        std::vector<correction_primitive_identity> excluded_primitive_ids;
        std::vector<correction_vertex_motion> vertex_motions;
        std::vector<asset::vec3> segment_start_positions;
    };

    struct correction_segment_side_latch
    {
        correction_candidate_pair_key key;
        asset::vec3 reference_normal;
        int8_t allowed_side_sign = 0;
        std::vector<uint64_t> topology_vertex_stable_ids;
    };

    struct correction_segment_vertex_snapshot
    {
        uint64_t stable_vertex_id = 0;
        asset::vec3 position;
    };

    struct correction_segment_candidate_state
    {
        uint64_t base_integration_segment_id = 0;
        uint64_t candidate_generation = 0;
        uint64_t position_writer_epoch_generation = 0;
        uint32_t last_position_writer_epoch_id = 0;
        uint64_t contact_set_candidate_generation = 0;
        uint64_t contact_set_position_writer_epoch_generation = 0;
        uint64_t vertex_contact_csr_candidate_generation = 0;
        uint64_t vertex_contact_csr_position_writer_epoch_generation = 0;
        bool contact_set_current = false;
        bool vertex_contact_csr_current = false;
        bool segment_geometry_initialized = false;
        std::vector<correction_segment_vertex_snapshot> segment_start_vertex_positions;
        std::vector<correction_segment_vertex_snapshot> latest_vertex_positions;
        bool segment_broadphase_envelope_initialized = false;
        collision_aabb segment_broadphase_envelope;
        std::vector<correction_candidate_pair_key> candidates;
        std::vector<correction_segment_side_latch> side_latches;
        std::vector<correction_discrete_contact> contact_set;
        correction_vertex_contact_csr vertex_contact_csr;
    };

    enum class correction_segment_commit_gate_status : uint8_t
    {
        ready,
        rejected
    };

    struct correction_segment_commit_gate_result
    {
        correction_segment_commit_gate_status status =
            correction_segment_commit_gate_status::rejected;
        uint64_t candidate_generation = 0;
        uint64_t position_writer_epoch_generation = 0;
        bool commit_allowed = false;
        std::string diagnostic;
    };

    struct correction_segment_candidate_state_result
    {
        correction_segment_candidate_state state;
        std::size_t required_count = 0;
        bool capacity_overflow = false;
        bool success = false;
        std::string diagnostic;
    };

    struct correction_segment_candidate_update_result
    {
        std::size_t required_count = 0;
        bool changed = false;
        bool capacity_overflow = false;
        bool wrong_segment = false;
        bool generation_overflow = false;
        bool invalid_epoch_order = false;
        bool epoch_generation_overflow = false;
        bool success = false;
        std::string diagnostic;
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

        // Segment-local latch reused unchanged by every Position-Writer Epoch.
        bool has_segment_side_latch = false;
        asset::vec3 segment_reference_normal;
        int8_t segment_allowed_side_sign = 0;
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

        // Segment-local latch reused unchanged by every Position-Writer Epoch.
        bool has_segment_side_latch = false;
        asset::vec3 segment_reference_normal;
        int8_t segment_allowed_side_sign = 0;    };

    enum class correction_writer_epoch_status : uint8_t
    {
        accepted,
        blocked,
        rejected
    };

    struct correction_writer_epoch_ccd_record
    {
        correction_candidate_pair_key candidate;
        correction_ccd_result result;
    };

    struct correction_writer_epoch_result
    {
        correction_writer_epoch_status status = correction_writer_epoch_status::blocked;
        bool state_committed = false;
        uint64_t candidate_generation = 0;
        uint64_t position_writer_epoch_generation = 0;
        std::size_t required_cell_records = 0;
        std::size_t required_candidates = 0;
        std::size_t required_contacts = 0;
        std::size_t required_vertex_contact_indices = 0;
        bool capacity_overflow = false;
        std::vector<correction_writer_epoch_ccd_record> correction_ccd_results;
        std::vector<correction_candidate_pair_key> active_contacts;
        std::vector<collision_diagnostic> diagnostics;
        std::string diagnostic;
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

    bool initialize_correction_segment_geometry(
        correction_segment_candidate_state& state,
        uint64_t base_integration_segment_id,
        const std::vector<correction_segment_vertex_snapshot>& segment_start_vertex_positions,
        const collision_aabb& physical_swept_bounds);

    correction_segment_candidate_state_result create_correction_segment_candidate_state(
        uint64_t base_integration_segment_id,
        const std::vector<correction_candidate_pair_key>& physical_swept_candidates,
        std::size_t capacity);

    correction_segment_candidate_update_result merge_correction_position_writer_epoch_candidates(
        correction_segment_candidate_state& state,
        uint64_t base_integration_segment_id,
        uint32_t position_writer_epoch_id,
        const std::vector<correction_candidate_pair_key>& epoch_candidates,
        std::size_t capacity);

    correction_segment_candidate_update_result begin_correction_position_writer_epoch(
        correction_segment_candidate_state& state,
        uint64_t base_integration_segment_id,
        uint32_t position_writer_epoch_id);

    correction_contact_set_refresh_result refresh_correction_segment_contact_set_and_csr(
        correction_segment_candidate_state& state,
        uint64_t candidate_generation,
        uint64_t position_writer_epoch_generation,
        const std::vector<correction_discrete_contact_input>& discrete_inputs,
        float epsilon_length = 1.0e-7f,
        std::size_t contact_set_capacity = 1u << 20,
        std::size_t vertex_contact_csr_capacity = 1u << 22);

    correction_segment_commit_gate_result validate_correction_segment_commit_gate(
        const correction_segment_candidate_state& state);

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

    correction_writer_epoch_result process_correction_position_writer_epoch(
        correction_segment_candidate_state& state,
        uint64_t base_integration_segment_id,
        uint32_t position_writer_epoch_id,
        const std::vector<correction_primitive_epoch_input>& primitives,
        const std::vector<broadphase_mesh_pair>& mesh_pairs,
        const broadphase_settings& broadphase_settings,
        const correction_ccd_settings& ccd_settings);

    correction_ccd_result detect_vertex_triangle_correction_motion(
        const vertex_triangle_correction_motion& motion,
        const correction_ccd_settings& settings = {});

    correction_ccd_result detect_edge_edge_correction_motion(
        const edge_edge_correction_motion& motion,
        const correction_ccd_settings& settings = {});
}
