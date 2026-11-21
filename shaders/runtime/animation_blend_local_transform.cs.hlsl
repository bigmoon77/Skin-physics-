struct animation_blend_gpu_record
{
    uint source_track_index;
    uint target_track_index;
    uint weight_curve_flags;
    uint reserved0;
    float4 weight_coefficients;
    float duration_seconds;
    float inverse_duration;
    float blend_start_offset_seconds;
    float reserved2;
};

struct animation_node_local_transform_gpu_record
{
    float4 translation;
    float4 translation_velocity;
    float4 rotation;
    float4 rotation_velocity;
    float4 scale;
    float4 scale_velocity;
    uint node_index;
    uint parent_index;
    uint node_role_flags;
    uint status;
};

struct animation_span_gpu_record
{
    uint span_id;
    uint animation_input_version_low;
    uint animation_input_version_high;
    uint first_track_record;
    uint track_record_count;
    uint first_channel_record;
    uint channel_record_count;
    uint first_blend_record;
    uint blend_record_count;
    uint first_collider_record;
    uint collider_record_count;
    uint flags;
    float span_duration_seconds;
    float inverse_span_duration;
    float speed_bound;
    float acceleration_bound;
    float chord_deviation_bound;
    float quaternion_normalization_lower_bound;
    float reserved0;
    float reserved1;
};

struct animation_span_cursor_gpu_record
{
    uint span_record_index;
    uint animation_input_version_low;
    uint animation_input_version_high;
    float u_segment_start;
};

cbuffer AnimationBlendParameters : register(b0)
{
    uint node_count;
    uint expected_span_record_index;
    uint blend_record_index;
    uint reserved;
};

StructuredBuffer<animation_node_local_transform_gpu_record> source_local_transforms : register(t0);
StructuredBuffer<animation_node_local_transform_gpu_record> target_local_transforms : register(t1);
StructuredBuffer<animation_blend_gpu_record> blend_records : register(t2);
StructuredBuffer<animation_span_cursor_gpu_record> animation_cursor : register(t3);
StructuredBuffer<animation_span_gpu_record> animation_spans : register(t4);

RWStructuredBuffer<animation_node_local_transform_gpu_record> blended_local_transforms : register(u0);
RWStructuredBuffer<uint> blend_status : register(u1);

static const uint BLEND_ENABLED = 1u;

bool finite4(float4 value)
{
    return all(isfinite(value));
}

bool normalize_quaternion_with_derivative(
    float4 raw_value,
    float4 raw_derivative,
    out float4 normalized_value,
    out float4 normalized_derivative)
{
    const float squared_length = dot(raw_value, raw_value);
    if (!isfinite(squared_length) || squared_length <= 1.0e-24f)
    {
        normalized_value = float4(0.0f, 0.0f, 0.0f, 1.0f);
        normalized_derivative = 0.0f;
        return false;
    }

    const float length = sqrt(squared_length);
    if (!isfinite(length) || length <= 1.0e-12f)
    {
        normalized_value = float4(0.0f, 0.0f, 0.0f, 1.0f);
        normalized_derivative = 0.0f;
        return false;
    }

    normalized_value = raw_value / length;
    normalized_derivative =
        (raw_derivative -
            normalized_value * dot(normalized_value, raw_derivative)) /
        length;
    return finite4(normalized_value) && finite4(normalized_derivative);
}

void fail_node(uint node_index)
{
    animation_node_local_transform_gpu_record failed =
        (animation_node_local_transform_gpu_record)0;
    failed.status = 1u;
    blended_local_transforms[node_index] = failed;
    uint previous_status = 0u;
    InterlockedOr(blend_status[0], 2u, previous_status);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint node_index = dispatch_thread_id.x;
    if (node_index >= node_count)
    {
        return;
    }

    const animation_span_cursor_gpu_record cursor =
        animation_cursor[0];
    if (cursor.span_record_index != expected_span_record_index)
    {
        fail_node(node_index);
        return;
    }
    const animation_span_gpu_record span =
        animation_spans[cursor.span_record_index];
    const float span_reciprocal_error =
        span.span_duration_seconds * span.inverse_span_duration - 1.0f;
    if (cursor.animation_input_version_low !=
            span.animation_input_version_low ||
        cursor.animation_input_version_high !=
            span.animation_input_version_high ||
        !isfinite(cursor.u_segment_start) ||
        cursor.u_segment_start < 0.0f ||
        cursor.u_segment_start > 1.0f ||
        !isfinite(span.span_duration_seconds) ||
        span.span_duration_seconds <= 0.0f ||
        !isfinite(span.inverse_span_duration) ||
        span.inverse_span_duration <= 0.0f ||
        !isfinite(span_reciprocal_error) ||
        abs(span_reciprocal_error) > 2.0e-5f)
    {
        fail_node(node_index);
        return;
    }
    const float span_duration_seconds = span.span_duration_seconds;
    const float span_local_u = cursor.u_segment_start;

    const animation_node_local_transform_gpu_record source =
        source_local_transforms[node_index];
    const animation_node_local_transform_gpu_record target =
        target_local_transforms[node_index];
    if (blend_record_index < span.first_blend_record ||
        blend_record_index - span.first_blend_record >=
            span.blend_record_count)
    {
        fail_node(node_index);
        return;
    }
    const animation_blend_gpu_record blend =
        blend_records[blend_record_index];

    if ((blend.weight_curve_flags & BLEND_ENABLED) == 0u ||
        (blend.weight_curve_flags & ~BLEND_ENABLED) != 0u ||
        !isfinite(blend.duration_seconds) ||
        blend.duration_seconds <= 0.0f ||
        !isfinite(blend.inverse_duration) ||
        blend.inverse_duration <= 0.0f ||
        !isfinite(blend.blend_start_offset_seconds) ||
        !finite4(blend.weight_coefficients) ||
        source.status != 0u ||
        target.status != 0u ||
        source.node_index != target.node_index ||
        source.parent_index != target.parent_index ||
        source.node_role_flags != target.node_role_flags ||
        source.node_index == 0xffffffffu)
    {
        fail_node(node_index);
        return;
    }

    const float blend_coordinate_unclamped =
        (blend.blend_start_offset_seconds +
            span_local_u * span_duration_seconds) *
        blend.inverse_duration;
    if (!isfinite(blend_coordinate_unclamped))
    {
        fail_node(node_index);
        return;
    }

    const float blend_coordinate =
        clamp(blend_coordinate_unclamped, 0.0f, 1.0f);
    const float coordinate_squared =
        blend_coordinate * blend_coordinate;
    const float weight =
        ((blend.weight_coefficients.w * blend_coordinate +
            blend.weight_coefficients.z) *
            blend_coordinate +
            blend.weight_coefficients.y) *
            blend_coordinate +
        blend.weight_coefficients.x;
    float weight_derivative = 0.0f;
    if (blend_coordinate_unclamped >= 0.0f &&
        blend_coordinate_unclamped <= 1.0f)
    {
        weight_derivative =
            (blend.weight_coefficients.y +
                2.0f * blend.weight_coefficients.z * blend_coordinate +
                3.0f * blend.weight_coefficients.w * coordinate_squared) *
            blend.inverse_duration;
    }

    if (!isfinite(weight) || weight < -1.0e-5f ||
        weight > 1.0f + 1.0e-5f ||
        !isfinite(weight_derivative))
    {
        fail_node(node_index);
        return;
    }
    const float clamped_weight = clamp(weight, 0.0f, 1.0f);

    animation_node_local_transform_gpu_record output =
        (animation_node_local_transform_gpu_record)0;
    output.node_index = source.node_index;
    output.parent_index = source.parent_index;
    output.node_role_flags = source.node_role_flags;

    output.translation =
        lerp(source.translation, target.translation, clamped_weight);
    output.translation_velocity =
        (1.0f - clamped_weight) * source.translation_velocity +
        clamped_weight * target.translation_velocity +
        weight_derivative * (target.translation - source.translation);
    output.scale =
        lerp(source.scale, target.scale, clamped_weight);
    output.scale_velocity =
        (1.0f - clamped_weight) * source.scale_velocity +
        clamped_weight * target.scale_velocity +
        weight_derivative * (target.scale - source.scale);

    float4 normalized_source_rotation;
    float4 normalized_source_derivative;
    float4 normalized_target_rotation;
    float4 normalized_target_derivative;
    if (!normalize_quaternion_with_derivative(
            source.rotation,
            source.rotation_velocity,
            normalized_source_rotation,
            normalized_source_derivative) ||
        !normalize_quaternion_with_derivative(
            target.rotation,
            target.rotation_velocity,
            normalized_target_rotation,
            normalized_target_derivative))
    {
        fail_node(node_index);
        return;
    }

    if (dot(normalized_source_rotation, normalized_target_rotation) < 0.0f)
    {
        normalized_target_rotation = -normalized_target_rotation;
        normalized_target_derivative = -normalized_target_derivative;
    }

    const float4 raw_blended_rotation =
        (1.0f - clamped_weight) * normalized_source_rotation +
        clamped_weight * normalized_target_rotation;
    const float4 raw_blended_derivative =
        (1.0f - clamped_weight) * normalized_source_derivative +
        clamped_weight * normalized_target_derivative +
        weight_derivative *
            (normalized_target_rotation - normalized_source_rotation);

    if (!normalize_quaternion_with_derivative(
            raw_blended_rotation,
            raw_blended_derivative,
            output.rotation,
            output.rotation_velocity) ||
        !finite4(output.translation) ||
        !finite4(output.translation_velocity) ||
        !finite4(output.scale) ||
        !finite4(output.scale_velocity))
    {
        fail_node(node_index);
        return;
    }

    output.status = 0u;
    blended_local_transforms[node_index] = output;
}
