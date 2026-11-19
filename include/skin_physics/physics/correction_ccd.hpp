#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "skin_physics/asset/types.hpp"

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
        bool has_persistent_contact_cache = false;
        asset::vec3 persistent_contact_normal;
    };

    struct edge_edge_correction_motion
    {
        std::array<correction_vertex_motion, 2> edge_a;
        std::array<correction_vertex_motion, 2> edge_b;
        std::array<asset::vec3, 2> segment_start_edge_a;
        std::array<asset::vec3, 2> segment_start_edge_b;
        bool has_persistent_contact_cache = false;
        asset::vec3 persistent_contact_normal;
    };

    correction_ccd_result detect_vertex_triangle_correction_motion(
        const vertex_triangle_correction_motion& motion,
        const correction_ccd_settings& settings = {});

    correction_ccd_result detect_edge_edge_correction_motion(
        const edge_edge_correction_motion& motion,
        const correction_ccd_settings& settings = {});
}
