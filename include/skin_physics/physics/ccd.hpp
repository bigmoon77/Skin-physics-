#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::physics
{
    enum class ccd_status : uint8_t
    {
        no_hit,
        hit,
        unresolved,
        invalid_input
    };

    struct ccd_settings
    {
        float safety = 0.8f;
        float distance_tolerance = 1.0e-5f;
        float minimum_time_advance = 1.0e-8f;
        uint32_t maximum_iterations = 32;
    };

    struct point_triangle_ccd_result
    {
        ccd_status status = ccd_status::invalid_input;
        asset::vec3 contact_center;
        asset::vec3 closest_point;
        asset::vec3 contact_normal;
        std::array<float, 3> barycentric = {};
        float toi_seconds = 0.0f;
        float remaining_interval_seconds = 0.0f;
        uint32_t iterations = 0;
        std::string diagnostic;
        bool hit = false;
    };

    struct toi_impulse_result
    {
        asset::vec3 velocity_point_after;
        asset::vec3 velocity_triangle_after;
        float impulse_magnitude = 0.0f;
        uint32_t application_count = 0;
        bool impact_resolved = false;
        bool success = false;
    };

    enum class initial_repair_status : uint8_t
    {
        success,
        invalid_input,
        over_budget
    };

    struct initial_repair_result
    {
        std::vector<asset::vec3> positions;
        float maximum_requested_correction = 0.0f;
        initial_repair_status status = initial_repair_status::invalid_input;
        bool state_committed = false;
    };

    point_triangle_ccd_result conservative_advance_point_triangle(
        const asset::vec3& point_start,
        const asset::vec3& point_end,
        const std::array<asset::vec3, 3>& triangle,
        float combined_contact_radius,
        float interval_seconds,
        const ccd_settings& settings = {});

    toi_impulse_result resolve_toi_normal_impulse(
        const asset::vec3& velocity_point,
        const asset::vec3& velocity_triangle,
        const asset::vec3& normal,
        float inverse_mass_point,
        float inverse_mass_triangle,
        float restitution,
        bool impact_already_resolved = false);

    initial_repair_result repair_initial_penetration(
        const std::vector<asset::vec3>& initial_positions,
        const std::vector<uint32_t>& participant_vertex_ids,
        const asset::vec3& direction,
        float requested_correction_distance_per_vertex,
        float maximum_correction_distance_per_vertex);
}
