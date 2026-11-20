#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::physics
{
    struct quaternion
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;
    };

    struct animation_keyframe
    {
        double time = 0.0;
        asset::vec3 translation;
        quaternion rotation;
        asset::vec3 scale = { 1.0f, 1.0f, 1.0f };
    };

    struct animation_track
    {
        std::vector<animation_keyframe> keyframes;
    };

    struct animation_sample_options
    {
        bool teleport = false;
        bool zero_velocity_on_teleport = true;
        asset::vec3 teleport_translation;
    };

    struct animation_transform_sample
    {
        asset::vec3 translation;
        quaternion rotation;
        asset::vec3 scale = { 1.0f, 1.0f, 1.0f };
        asset::vec3 translation_velocity;
    };

    enum class animation_error : uint8_t
    {
        none,
        empty_track,
        invalid_time,
        invalid_keyframe,
        non_monotonic_time,
        invalid_quaternion
    };

    struct animation_sample_result
    {
        animation_transform_sample value;
        animation_error error = animation_error::none;
        std::string message;
        bool success = false;
    };

    quaternion normalize_quaternion(const quaternion& value);
    quaternion slerp_quaternion(
        const quaternion& start,
        const quaternion& end,
        float interpolation);

    animation_sample_result sample_animation_track(
        const animation_track& track,
        double sample_time,
        const animation_sample_options& options = {});

    enum class animation_channel_span_error : uint8_t
    {
        none,
        invalid_track,
        invalid_time,
        malformed_key,
        non_monotonic_time,
        span_crosses_key_boundary,
        invalid_quaternion,
        capacity_exceeded
    };

    // Immutable CPU-side coefficients for one animation channel over a single
    // Key-bounded Span. coefficients[0..3] are c0..c3 in
    // value(u) = c0 + c1*u + c2*u^2 + c3*u^3, u in [0, 1].
    // For LINEAR rotation, c0 / c1 hold the hemisphere-adjusted q0 / q1.
    struct animation_channel_span
    {
        uint32_t track_index = asset::invalid_index;
        uint32_t node_index = asset::invalid_index;
        asset::animation_path path = asset::animation_path::translation;
        asset::interpolation interpolation_mode = asset::interpolation::step;
        double clip_time_start = 0.0;
        double clip_time_delta = 0.0;
        std::array<asset::vec4, 4> coefficients = {};
    };

    struct animation_channel_span_build_result
    {
        animation_channel_span value;
        animation_channel_span_error error = animation_channel_span_error::none;
        std::string message;
        bool success = false;
    };

    struct animation_channel_span_sample
    {
        asset::vec4 value;
        // Derivative per World Simulation second, not per Clip second.
        asset::vec4 world_time_derivative;
    };

    struct animation_channel_span_sample_result
    {
        animation_channel_span_sample value;
        animation_channel_span_error error = animation_channel_span_error::none;
        std::string message;
        bool success = false;
    };

    // Builds a Span polynomial for a signed Clip-Time interval. A Span that
    // crosses an interior key boundary is rejected and must be split by the
    // caller. A zero clip_time_delta represents a paused Track.
    animation_channel_span_build_result build_animation_channel_span(
        const asset::animation_track& track,
        double clip_time_start,
        double clip_time_delta);

    struct animation_channel_spans_build_result
    {
        std::vector<animation_channel_span> values;
        animation_channel_span_error error = animation_channel_span_error::none;
        std::string message;
        bool success = false;
    };

    // Splits the signed Clip-Time interval at every interior key boundary and
    // builds one immutable coefficient record per ordered Span.
    animation_channel_spans_build_result build_animation_channel_spans(
        const asset::animation_track& track,
        double clip_time_start,
        double clip_time_delta);

    struct scheduled_animation_channel_span
    {
        animation_channel_span channel;
        double world_time_start = 0.0;
        double world_time_duration = 0.0;
    };

    struct animation_channel_span_schedule_result
    {
        std::vector<scheduled_animation_channel_span> values;
        animation_channel_span_error error = animation_channel_span_error::none;
        std::string message;
        bool success = false;
    };

    // Maps key-bounded Clip-Time spans to ordered positive-duration World-Time
    // intervals inside one Logical Tick. A paused Track spans the full Tick.
    animation_channel_span_schedule_result schedule_animation_channel_spans(
        const asset::animation_track& track,
        double clip_time_start,
        double clip_time_delta,
        double world_time_start,
        double world_time_duration);

    struct animation_clip_span
    {
        uint32_t span_index = 0u;
        double clip_time_start = 0.0;
        double clip_time_delta = 0.0;
        // Channels are kept in original Asset Track order and share this
        // exact Clip-Time interval across all Nodes / Paths in the Clip.
        std::vector<animation_channel_span> channels;
    };

    struct animation_clip_spans_build_result
    {
        std::vector<animation_clip_span> values;
        animation_channel_span_error error = animation_channel_span_error::none;
        std::string message;
        bool success = false;
    };

    // Builds a synchronized multi-Track schedule for one unwrapped interval
    // inside the Clip duration. Every channel shares the union of all Track
    // Key boundaries. Loop wrap / blend boundaries must be split upstream.
    animation_clip_spans_build_result build_animation_clip_spans(
        const asset::animation_clip& clip,
        uint32_t node_count,
        double clip_time_start,
        double clip_time_delta);

    struct scheduled_animation_clip_span
    {
        animation_clip_span clip;
        double world_time_start = 0.0;
        double world_time_duration = 0.0;
    };

    struct animation_clip_span_schedule_result
    {
        std::vector<scheduled_animation_clip_span> values;
        animation_channel_span_error error = animation_channel_span_error::none;
        std::string message;
        bool success = false;
    };

    // Maps synchronized Clip Spans to positive World-Time intervals within one
    // Logical Tick. Loop-wrap / Blend boundaries must already be split upstream.
    animation_clip_span_schedule_result schedule_animation_clip_spans(
        const asset::animation_clip& clip,
        uint32_t node_count,
        double clip_time_start,
        double clip_time_delta,
        double world_time_start,
        double world_time_duration);

    // Maps Playback Speed into key-bounded Clip Spans over one Logical Tick.
    // Loop wraps are explicit boundaries; non-looping Playback holds the
    // terminal Clip Pose for the remainder of the Tick. maximum_span_count
    // bounds pathological high-speed Loop requests.
    animation_clip_span_schedule_result schedule_animation_clip_playback_spans(
        const asset::animation_clip& clip,
        uint32_t node_count,
        double clip_time_start,
        double playback_speed,
        bool loop,
        double world_time_start,
        double world_time_duration,
        uint32_t maximum_span_count = 4096u);

    // Evaluates the immutable Span coefficients and analytic derivatives.
    // span_duration_seconds is the corresponding positive World-Time duration.
    animation_channel_span_sample_result evaluate_animation_channel_span(
        const animation_channel_span& span,
        double span_local_u,
        double span_duration_seconds);

    struct animation_blend_weight_curve_desc
    {
        double blend_start_time = 0.0;
        double blend_end_time = 1.0;
        float weight_start = 0.0f;
        float weight_end = 1.0f;
        float tangent_start = 0.0f;
        float tangent_end = 0.0f;
    };

    enum class animation_blend_weight_error : uint8_t
    {
        none,
        invalid_curve,
        invalid_time,
        span_crosses_curve_boundary
    };

    struct animation_blend_weight_span
    {
        double world_time_start = 0.0;
        double world_time_duration = 0.0;
        std::array<double, 4> coefficients = {};
    };

    struct animation_blend_weight_span_build_result
    {
        animation_blend_weight_span value;
        animation_blend_weight_error error = animation_blend_weight_error::none;
        std::string message;
        bool success = false;
    };

    struct animation_blend_weight_spans_build_result
    {
        std::vector<animation_blend_weight_span> values;
        animation_blend_weight_error error = animation_blend_weight_error::none;
        std::string message;
        bool success = false;
    };

    struct animation_blend_weight_sample
    {
        double weight = 0.0;
        double world_time_derivative = 0.0;
    };

    struct animation_blend_weight_sample_result
    {
        animation_blend_weight_sample value;
        animation_blend_weight_error error = animation_blend_weight_error::none;
        std::string message;
        bool success = false;
    };

    enum class animation_blend_clip_span_error : uint8_t
    {
        none,
        invalid_source_clip,
        invalid_target_clip,
        invalid_blend_curve,
        invalid_time,
        capacity_exceeded
    };

    struct animation_blend_clip_span
    {
        uint32_t span_index = 0u;
        double world_time_start = 0.0;
        double world_time_duration = 0.0;
        scheduled_animation_clip_span source;
        scheduled_animation_clip_span target;
        animation_blend_weight_span blend_weight;
    };

    struct animation_blend_clip_span_schedule_result
    {
        std::vector<animation_blend_clip_span> values;
        animation_blend_clip_span_error error =
            animation_blend_clip_span_error::none;
        std::string message;
        bool success = false;
    };

    enum class animation_hierarchy_error : uint8_t
    {
        none,
        invalid_model,
        invalid_clip_span,
        invalid_channel,
        invalid_transform,
        invalid_hierarchy,
        invalid_skin,
        singular_mesh_node,
        invalid_blend
    };

    struct animation_skin_pose
    {
        uint32_t mesh_node_index = asset::invalid_index;
        uint32_t skin_index = asset::invalid_index;
        std::vector<asset::mat4> skin_matrices;
        std::vector<asset::mat4> skin_matrix_derivatives;
    };

    struct animation_node_local_transform
    {
        bool uses_matrix = false;
        asset::mat4 matrix;
        asset::vec3 translation;
        asset::vec4 rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
        asset::vec3 scale = { 1.0f, 1.0f, 1.0f };
        asset::vec3 translation_velocity;
        asset::vec4 rotation_velocity;
        asset::vec3 scale_velocity;
    };

    struct animation_hierarchy_sample
    {
        // All matrices are model-local, column-major, column-vector transforms.
        // Local TRS and its derivatives are retained so the synchronized Source
        // / Target Pose can be blended before hierarchy propagation.
        std::vector<animation_node_local_transform> local_transforms;
        std::vector<asset::mat4> local_matrices;
        std::vector<asset::mat4> local_matrix_derivatives;
        std::vector<asset::mat4> global_matrices;
        std::vector<asset::mat4> global_matrix_derivatives;
        std::vector<animation_skin_pose> skin_poses;
    };

    struct animation_hierarchy_sample_result
    {
        animation_hierarchy_sample value;
        animation_hierarchy_error error =
            animation_hierarchy_error::none;
        std::string message;
        bool success = false;
    };

    // Evaluates every Asset Node from its Rest TRS / static Matrix plus one
    // synchronized Clip Span. Builds the full parent hierarchy and all Skin
    // Matrices with analytic World-Time derivatives.
    animation_hierarchy_sample_result evaluate_animation_clip_hierarchy(
        const asset::model& model,
        const asset::animation_clip& clip,
        const animation_clip_span& span,
        double span_local_u,
        double span_world_duration);

    // Evaluates Source and Target Clip spans at the same World-Time coordinate,
    // blends every Node's local TRS and analytic derivative, then rebuilds the
    // complete Node hierarchy and Skin Matrices from the blended local pose.
    animation_hierarchy_sample_result evaluate_animation_blend_clip_hierarchy(
        const asset::model& model,
        const asset::animation_clip& source_clip,
        const asset::animation_clip& target_clip,
        const animation_blend_clip_span& span,
        double span_local_u);

    animation_blend_clip_span_schedule_result schedule_animation_blend_clip_spans(
        const asset::animation_clip& source_clip,
        uint32_t source_node_count,
        double source_clip_time_start,
        double source_playback_speed,
        bool source_loop,
        const asset::animation_clip& target_clip,
        uint32_t target_node_count,
        double target_clip_time_start,
        double target_playback_speed,
        bool target_loop,
        const animation_blend_weight_curve_desc& blend_curve,
        double world_time_start,
        double world_time_duration,
        uint32_t maximum_span_count = 4096u);

    bool valid_animation_blend_weight_curve(
        const animation_blend_weight_curve_desc& curve);

    animation_blend_weight_span_build_result build_animation_blend_weight_span(
        const animation_blend_weight_curve_desc& curve,
        double world_time_start,
        double world_time_duration);

    animation_blend_weight_spans_build_result build_animation_blend_weight_spans(
        const animation_blend_weight_curve_desc& curve,
        double world_time_start,
        double world_time_duration);

    animation_blend_weight_sample_result evaluate_animation_blend_weight_span(
        const animation_blend_weight_span& span,
        double span_local_u);
}
