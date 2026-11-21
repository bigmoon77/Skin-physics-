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

    struct gpu_float4
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 0.0f;
    };

    struct gpu_matrix4x4
    {
        gpu_float4 column0;
        gpu_float4 column1;
        gpu_float4 column2;
        gpu_float4 column3;
    };

    // Segment-local GPU working record shared by Channel / Blend / Hierarchy
    // compute stages. The immutable Packet stores only Channel Records.
    struct alignas(16) animation_node_local_transform_gpu_record
    {
        gpu_float4 translation;
        gpu_float4 translation_velocity;
        gpu_float4 rotation;
        gpu_float4 rotation_velocity;
        gpu_float4 scale;
        gpu_float4 scale_velocity;
        uint32_t node_index = asset::invalid_index;
        uint32_t parent_index = asset::invalid_index;
        uint32_t node_role_flags = 0u;
        uint32_t status = 0u;
    };

    static_assert(std::is_standard_layout_v<animation_node_local_transform_gpu_record>);
    static_assert(std::is_trivially_copyable_v<animation_node_local_transform_gpu_record>);
    static_assert(sizeof(animation_node_local_transform_gpu_record) == 112u);
    static_assert(alignof(animation_node_local_transform_gpu_record) == 16u);

    struct animation_span_gpu_record
    {
        uint32_t span_id = 0u;
        uint32_t animation_input_version_low = 0u;
        uint32_t animation_input_version_high = 0u;
        uint32_t first_track_record = 0u;
        uint32_t track_record_count = 0u;
        uint32_t first_channel_record = 0u;
        uint32_t channel_record_count = 0u;
        uint32_t first_blend_record = 0u;
        uint32_t blend_record_count = 0u;
        uint32_t first_collider_record = 0u;
        uint32_t collider_record_count = 0u;
        uint32_t flags = 0u;
        float span_duration_seconds = 0.0f;
        float inverse_span_duration = 0.0f;
        float speed_bound = 0.0f;
        float acceleration_bound = 0.0f;
        float chord_deviation_bound = 0.0f;
        float quaternion_normalization_lower_bound = 1.0f;
        float reserved0 = 0.0f;
        float reserved1 = 0.0f;
    };

    struct animation_span_cursor_gpu_record
    {
        uint32_t span_record_index = asset::invalid_index;
        uint32_t animation_input_version_low = 0u;
        uint32_t animation_input_version_high = 0u;
        float u_segment_start = 0.0f;
    };

    struct animation_track_gpu_record
    {
        uint32_t clip_index = asset::invalid_index;
        uint32_t first_channel_record = 0u;
        uint32_t channel_record_count = 0u;
        uint32_t flags = 0u;
        float clip_time_start = 0.0f;
        float clip_time_delta = 0.0f;
        float playback_speed = 0.0f;
        float reserved0 = 0.0f;
    };

    struct animation_channel_gpu_record
    {
        uint32_t node_index = asset::invalid_index;
        // Parent Channel offset is local to this Track's Channel range.
        uint32_t parent_index = asset::invalid_index;
        // First encountered IBM for this Joint Node (compatibility / diagnostics).
        // Skin-specific LBS uses animation_skin_joint_gpu_record instead.
        uint32_t inverse_bind_record_index = asset::invalid_index;
        uint32_t interpolation_flags = 0u;
        uint32_t channel_mask = 0u;
        uint32_t node_role_flags = 0u;
        uint32_t reserved0 = 0u;
        uint32_t reserved1 = 0u;
        gpu_float4 translation_c0;
        gpu_float4 translation_c1;
        gpu_float4 translation_c2;
        gpu_float4 translation_c3;
        gpu_float4 rotation_parameter0;
        gpu_float4 rotation_parameter1;
        gpu_float4 rotation_parameter2;
        gpu_float4 rotation_parameter3;
        gpu_float4 scale_c0;
        gpu_float4 scale_c1;
        gpu_float4 scale_c2;
        gpu_float4 scale_c3;
    };

    struct animation_blend_gpu_record
    {
        uint32_t source_track_index = 0u;
        uint32_t target_track_index = 1u;
        uint32_t weight_curve_flags = 0u;
        uint32_t reserved0 = 0u;
        gpu_float4 weight_coefficients;
        float duration_seconds = 0.0f;
        float inverse_duration = 0.0f;
        float blend_start_offset_seconds = 0.0f;
        float reserved2 = 0.0f;
    };

    struct kinematic_collider_gpu_record
    {
        uint32_t stable_collider_id_low = 0u;
        uint32_t stable_collider_id_high = 0u;
        uint32_t span_sample_pair_id = 0u;
        uint32_t flags = 0u;
        gpu_float4 translation_start;
        gpu_float4 translation_end;
        gpu_float4 rotation_start;
        gpu_float4 rotation_end;
        gpu_float4 angular_axis_and_angle;
        float duration_seconds = 0.0f;
        float inverse_duration = 0.0f;
        float linear_speed_bound = 0.0f;
        float angular_speed_bound = 0.0f;
        float maximum_local_radius = 0.0f;
        float sagitta_bound = 0.0f;
        float reserved0 = 0.0f;
        float reserved1 = 0.0f;
    };

    // Static mapping from each Skinned Mesh Node to its ordered Joint set.
    // Channel offsets are relative to a Track's first_channel_record so the
    // same immutable Skin table works for every Track / Span in the Packet.
    struct animation_skin_gpu_record
    {
        uint32_t mesh_node_channel_offset = asset::invalid_index;
        uint32_t skin_index = asset::invalid_index;
        uint32_t first_joint_record = 0u;
        uint32_t joint_count = 0u;
    };

    struct animation_skin_joint_gpu_record
    {
        uint32_t joint_channel_offset = asset::invalid_index;
        uint32_t inverse_bind_record_index = asset::invalid_index;
        uint32_t joint_node_index = asset::invalid_index;
        uint32_t reserved = 0u;
    };

    static_assert(std::is_standard_layout_v<gpu_float4>);
    static_assert(std::is_trivially_copyable_v<gpu_float4>);
    static_assert(sizeof(gpu_float4) == 16u);
    static_assert(alignof(gpu_float4) == 4u);
    static_assert(offsetof(gpu_float4, x) == 0u);
    static_assert(offsetof(gpu_float4, y) == 4u);
    static_assert(offsetof(gpu_float4, z) == 8u);
    static_assert(offsetof(gpu_float4, w) == 12u);

    static_assert(std::is_standard_layout_v<gpu_matrix4x4>);
    static_assert(std::is_trivially_copyable_v<gpu_matrix4x4>);
    static_assert(sizeof(gpu_matrix4x4) == 64u);
    static_assert(alignof(gpu_matrix4x4) == 4u);

    static_assert(std::is_standard_layout_v<animation_span_gpu_record>);
    static_assert(std::is_trivially_copyable_v<animation_span_gpu_record>);
    static_assert(sizeof(animation_span_gpu_record) == 80u);
    static_assert(alignof(animation_span_gpu_record) == 4u);
    static_assert(offsetof(animation_span_gpu_record, first_track_record) == 12u);
    static_assert(offsetof(animation_span_gpu_record, first_channel_record) == 20u);
    static_assert(offsetof(animation_span_gpu_record, first_blend_record) == 28u);
    static_assert(offsetof(animation_span_gpu_record, first_collider_record) == 36u);
    static_assert(offsetof(animation_span_gpu_record, span_duration_seconds) == 48u);
    static_assert(offsetof(animation_span_gpu_record, quaternion_normalization_lower_bound) == 68u);
    static_assert(offsetof(animation_span_gpu_record, reserved1) == 76u);

    static_assert(std::is_standard_layout_v<animation_span_cursor_gpu_record>);
    static_assert(std::is_trivially_copyable_v<animation_span_cursor_gpu_record>);
    static_assert(sizeof(animation_span_cursor_gpu_record) == 16u);
    static_assert(alignof(animation_span_cursor_gpu_record) == 4u);
    static_assert(offsetof(animation_span_cursor_gpu_record, animation_input_version_low) == 4u);
    static_assert(offsetof(animation_span_cursor_gpu_record, u_segment_start) == 12u);

    static_assert(std::is_standard_layout_v<animation_track_gpu_record>);
    static_assert(std::is_trivially_copyable_v<animation_track_gpu_record>);
    static_assert(sizeof(animation_track_gpu_record) == 32u);
    static_assert(alignof(animation_track_gpu_record) == 4u);
    static_assert(offsetof(animation_track_gpu_record, clip_time_start) == 16u);
    static_assert(offsetof(animation_track_gpu_record, playback_speed) == 24u);

    static_assert(std::is_standard_layout_v<animation_channel_gpu_record>);
    static_assert(std::is_trivially_copyable_v<animation_channel_gpu_record>);
    static_assert(sizeof(animation_channel_gpu_record) == 224u);
    static_assert(alignof(animation_channel_gpu_record) == 4u);
    static_assert(offsetof(animation_channel_gpu_record, translation_c0) == 32u);
    static_assert(offsetof(animation_channel_gpu_record, rotation_parameter0) == 96u);
    static_assert(offsetof(animation_channel_gpu_record, scale_c0) == 160u);
    static_assert(offsetof(animation_channel_gpu_record, scale_c3) == 208u);

    static_assert(std::is_standard_layout_v<animation_blend_gpu_record>);
    static_assert(std::is_trivially_copyable_v<animation_blend_gpu_record>);
    static_assert(sizeof(animation_blend_gpu_record) == 48u);
    static_assert(alignof(animation_blend_gpu_record) == 4u);
    static_assert(offsetof(animation_blend_gpu_record, weight_coefficients) == 16u);
    static_assert(offsetof(animation_blend_gpu_record, duration_seconds) == 32u);
    static_assert(offsetof(animation_blend_gpu_record, blend_start_offset_seconds) == 40u);

    static_assert(std::is_standard_layout_v<animation_skin_gpu_record>);
    static_assert(std::is_trivially_copyable_v<animation_skin_gpu_record>);
    static_assert(sizeof(animation_skin_gpu_record) == 16u);
    static_assert(alignof(animation_skin_gpu_record) == 4u);
    static_assert(offsetof(animation_skin_gpu_record, first_joint_record) == 8u);

    static_assert(std::is_standard_layout_v<animation_skin_joint_gpu_record>);
    static_assert(std::is_trivially_copyable_v<animation_skin_joint_gpu_record>);
    static_assert(sizeof(animation_skin_joint_gpu_record) == 16u);
    static_assert(alignof(animation_skin_joint_gpu_record) == 4u);
    static_assert(offsetof(animation_skin_joint_gpu_record, inverse_bind_record_index) == 4u);
    static_assert(offsetof(animation_skin_joint_gpu_record, joint_node_index) == 8u);

    static_assert(std::is_standard_layout_v<kinematic_collider_gpu_record>);
    static_assert(std::is_trivially_copyable_v<kinematic_collider_gpu_record>);
    static_assert(sizeof(kinematic_collider_gpu_record) == 128u);
    static_assert(alignof(kinematic_collider_gpu_record) == 4u);
    static_assert(offsetof(kinematic_collider_gpu_record, translation_start) == 16u);
    static_assert(offsetof(kinematic_collider_gpu_record, angular_axis_and_angle) == 80u);
    static_assert(offsetof(kinematic_collider_gpu_record, duration_seconds) == 96u);
    static_assert(offsetof(kinematic_collider_gpu_record, reserved1) == 124u);

    constexpr uint32_t animation_node_role_joint = 1u << 0u;
    constexpr uint32_t animation_node_role_mesh = 1u << 1u;
    constexpr uint32_t animation_node_role_model_root = 1u << 2u;
    constexpr uint32_t animation_node_role_required_joint_ancestor = 1u << 3u;
    constexpr uint32_t animation_node_role_static_matrix = 1u << 4u;

    constexpr uint32_t animation_channel_mask_translation = 1u << 0u;
    constexpr uint32_t animation_channel_mask_rotation = 1u << 1u;
    constexpr uint32_t animation_channel_mask_scale = 1u << 2u;

    constexpr uint32_t animation_span_flag_has_blend = 1u << 0u;
    constexpr uint32_t animation_span_flag_bounds_certified = 1u << 3u;
    constexpr uint32_t animation_track_flag_loop = 1u << 0u;
    constexpr uint32_t animation_track_flag_reverse_playback = 1u << 1u;
    constexpr uint32_t animation_blend_flag_enabled = 1u << 0u;

    struct animation_trajectory_span_bounds
    {
        // Must be conservative, outward-rounded bounds calculated by a
        // certified interval-bound pass. The Packet Builder never guesses.
        float speed_bound = 0.0f;
        float acceleration_bound = 0.0f;
        float chord_deviation_bound = 0.0f;
        float quaternion_normalization_lower_bound = 1.0f;
    };

    struct animation_trajectory_track_span_input
    {
        uint32_t clip_index = asset::invalid_index;
        scheduled_animation_clip_span scheduled;
        bool loop = false;
        bool reverse_playback = false;
    };

    struct animation_trajectory_span_input
    {
        uint32_t span_id = 0u;
        double world_time_start = 0.0;
        double world_time_duration = 0.0;
        std::vector<animation_trajectory_track_span_input> tracks;
        bool has_blend = false;
        uint32_t source_track_index = 0u;
        uint32_t target_track_index = 1u;
        animation_blend_weight_curve_desc blend_curve;
        animation_trajectory_span_bounds bounds;
        // False is permitted for skinning-only Packets. CCD / no-miss consumers
        // must reject Spans without the Bounds-Certified flag.
        bool bounds_are_certified = true;
    };

    struct animation_trajectory_packet_build_desc
    {
        uint64_t animation_input_version = 0u;
        uint32_t maximum_span_count = 4096u;
        std::vector<animation_trajectory_span_input> spans;
    };

    struct animation_trajectory_packet
    {
        uint64_t animation_input_version = 0u;
        std::vector<animation_span_gpu_record> span_records;
        std::vector<animation_track_gpu_record> track_records;
        std::vector<animation_channel_gpu_record> channel_records;
        std::vector<animation_blend_gpu_record> blend_records;
        std::vector<gpu_matrix4x4> inverse_bind_matrices;
        // Dense by Asset Node Index. TRS nodes contain Identity; static affine
        // Matrix nodes contain their immutable local matrix and set role bit4.
        std::vector<gpu_matrix4x4> static_node_matrices;
        // Static Skin -> Joint mapping. Channel offsets are relative to each
        // Track's first_channel_record; inverse-bind offsets are absolute.
        std::vector<animation_skin_gpu_record> skin_records;
        std::vector<animation_skin_joint_gpu_record> skin_joint_records;
        std::vector<kinematic_collider_gpu_record> collider_records;
        // CPU-only double World-Time metadata; this is never uploaded to the GPU.
        std::vector<double> span_world_time_starts;
    };

    enum class animation_trajectory_packet_error : uint8_t
    {
        none,
        invalid_version,
        invalid_model,
        invalid_span,
        invalid_track,
        invalid_channel,
        invalid_hierarchy,
        invalid_skin,
        unsupported_inverse_bind_layout,
        invalid_bounds,
        capacity_exceeded
    };

    struct animation_trajectory_packet_build_result
    {
        animation_trajectory_packet value;
        animation_trajectory_packet_error error =
            animation_trajectory_packet_error::none;
        std::string message;
        bool success = false;
    };

    animation_trajectory_packet_build_result build_animation_trajectory_packet(
        const asset::model& model,
        const animation_trajectory_packet_build_desc& desc);

    // Initializes the per-World mutable cursor for a new Segment. The Packet
    // remains immutable; only this separate Cursor record is uploaded to UAV.
    bool initialize_animation_span_cursor(
        const animation_trajectory_packet& packet,
        uint32_t span_record_index,
        double segment_world_time,
        animation_span_cursor_gpu_record& out_cursor,
        std::string& diagnostic);

    // Advances one positive World-Time Integration Segment. The Segment must
    // remain inside the current Span or end exactly at its next boundary.
    // The Packet and input version are immutable; output is transactional.
    bool advance_animation_span_cursor(
        const animation_trajectory_packet& packet,
        const animation_span_cursor_gpu_record& cursor,
        double segment_duration_seconds,
        animation_span_cursor_gpu_record& out_cursor,
        std::string& diagnostic);

}
