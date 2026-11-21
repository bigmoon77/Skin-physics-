#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"
#include "skin_physics/physics/animation_track.hpp"

namespace skin_physics::physics
{
    struct vertex_triangle_contact
    {
        asset::vec3 closest_point;
        std::array<float, 3> barycentric = {};
        asset::vec3 normal;
        float distance = 0.0f;
        float penetration_depth = 0.0f;
        bool hit = false;
        bool valid = false;
    };

    struct vertex_triangle_solve_result
    {
        std::array<asset::vec3, 4> positions = {};
        float post_solve_penetration = 0.0f;
        bool momentum_conserved = false;
        bool success = false;
    };

    struct edge_edge_contact
    {
        asset::vec3 closest_point_a;
        asset::vec3 closest_point_b;
        asset::vec3 normal;
        float parameter_a = 0.0f;
        float parameter_b = 0.0f;
        float distance = 0.0f;
        float penetration_depth = 0.0f;
        bool hit = false;
        bool valid = false;
    };

    enum class collision_error : uint8_t
    {
        none,
        invalid_geometry,
        invalid_settings,
        cell_capacity_overflow,
        candidate_capacity_overflow
    };

    struct collision_diagnostic
    {
        collision_error code = collision_error::none;
        uint32_t primitive_index = asset::invalid_index;
        std::string message;
    };

    struct collision_aabb
    {
        asset::vec3 minimum;
        asset::vec3 maximum;
    };

    struct kinematic_collider_geometry_result
    {
        std::vector<asset::vec3> world_vertices;
        std::vector<asset::vec3> world_vertex_velocities;
        collision_aabb swept_bounds;
        collision_error error = collision_error::none;
        std::string diagnostic;
        bool success = false;
    };

    // CPU collision oracle for one immutable Kinematic Collider Span. The
    // swept AABB uses the certified local bounding sphere and the complete
    // endpoint translation segment, so it remains conservative for arbitrary
    // rotation throughout the Span. World vertices are evaluated at u.
    kinematic_collider_geometry_result evaluate_kinematic_collider_geometry(
        const asset::mesh& collision_mesh,
        const kinematic_collider_gpu_record& span,
        double span_local_u);

    struct broadphase_primitive
    {
        uint32_t mesh_id = 0;
        collision_aabb bounds;
    };

    struct broadphase_mesh_pair
    {
        uint32_t mesh_a = 0;
        uint32_t mesh_b = 0;
        bool enabled = false;
    };

    struct broadphase_settings
    {
        float cell_size = 1.0f;
        float contact_thickness = 0.0f;
        std::size_t max_cells_per_primitive = 4096;
        std::size_t cell_record_capacity = 1u << 20;
        std::size_t candidate_capacity = 1u << 20;
        bool self_collision_enabled = false;
    };

    struct collision_candidate
    {
        uint32_t primitive_a = 0;
        uint32_t primitive_b = 0;

        bool operator==(const collision_candidate& other) const
        {
            return primitive_a == other.primitive_a &&
                primitive_b == other.primitive_b;
        }

        bool operator<(const collision_candidate& other) const
        {
            if (primitive_a != other.primitive_a) return primitive_a < other.primitive_a;
            return primitive_b < other.primitive_b;
        }
    };

    struct broadphase_result
    {
        std::vector<collision_candidate> candidates;
        std::vector<collision_diagnostic> diagnostics;
        std::size_t required_cell_records = 0;
        std::size_t required_candidates = 0;
        bool overflow = false;
        bool success = false;
    };

    struct capacity_check_result
    {
        std::size_t required_count = 0;
        std::size_t capacity = 0;
        bool overflow = false;
        bool state_committable = false;
    };

    vertex_triangle_contact find_vertex_triangle_contact(
        const asset::vec3& point,
        const std::array<asset::vec3, 3>& triangle,
        float point_radius);

    vertex_triangle_solve_result solve_vertex_triangle_contact(
        const vertex_triangle_contact& contact,
        const std::array<asset::vec3, 4>& positions,
        const std::array<float, 4>& inverse_masses,
        float point_radius,
        float relaxation = 0.5f);

    edge_edge_contact find_edge_edge_contact(
        const std::array<asset::vec3, 2>& edge_a,
        const std::array<asset::vec3, 2>& edge_b,
        float combined_thickness);

    using broadphase_candidate_filter =
        std::function<bool(uint32_t, uint32_t)>;

    broadphase_result generate_broadphase_candidates(
        const std::vector<broadphase_primitive>& primitives,
        const std::vector<broadphase_mesh_pair>& mesh_pairs,
        const broadphase_settings& settings = {},
        const broadphase_candidate_filter& candidate_filter = {});

    capacity_check_result check_candidate_capacity(
        std::size_t required_count,
        std::size_t capacity);
}
