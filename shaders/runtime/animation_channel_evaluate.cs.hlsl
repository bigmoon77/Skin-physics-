struct gpu_float4
{
    float x;
    float y;
    float z;
    float w;
};

struct animation_channel_gpu_record
{
    uint node_index;
    uint parent_index;
    uint inverse_bind_record_index;
    uint interpolation_flags;
    uint channel_mask;
    uint node_role_flags;
    uint reserved0;
    uint reserved1;
    float4 translation_c0;
    float4 translation_c1;
    float4 translation_c2;
    float4 translation_c3;
    float4 rotation_parameter0;
    float4 rotation_parameter1;
    float4 rotation_parameter2;
    float4 rotation_parameter3;
    float4 scale_c0;
    float4 scale_c1;
    float4 scale_c2;
    float4 scale_c3;
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

cbuffer AnimationChannelEvaluationParameters : register(b0)
{
    uint channel_count;
    uint first_channel_record;
    uint expected_span_record_index;
    uint reserved;
};

StructuredBuffer<animation_channel_gpu_record> animation_channels : register(t0);
StructuredBuffer<animation_span_cursor_gpu_record> animation_cursor : register(t1);
StructuredBuffer<animation_span_gpu_record> animation_spans : register(t2);
RWStructuredBuffer<animation_node_local_transform_gpu_record> local_transforms : register(u0);
RWStructuredBuffer<uint> evaluation_status : register(u1);

static const uint CHANNEL_TRANSLATION = 1u;
static const uint CHANNEL_ROTATION = 2u;
static const uint CHANNEL_SCALE = 4u;

bool finite4(float4 value)
{
    return all(isfinite(value));
}

bool normalize_quaternion(
    float4 input_value,
    out float4 normalized_value,
    out float input_length)
{
    const float squared_length = dot(input_value, input_value);
    if (!isfinite(squared_length) || squared_length <= 1.0e-24f)
    {
        normalized_value = float4(0.0f, 0.0f, 0.0f, 1.0f);
        input_length = 0.0f;
        return false;
    }

    input_length = sqrt(squared_length);
    if (!isfinite(input_length) || input_length <= 1.0e-12f)
    {
        normalized_value = float4(0.0f, 0.0f, 0.0f, 1.0f);
        return false;
    }

    normalized_value = input_value / input_length;
    return finite4(normalized_value);
}

void evaluate_vector_polynomial(
    float4 c0,
    float4 c1,
    float4 c2,
    float4 c3,
    float u,
    float inverse_duration,
    out float4 value,
    out float4 world_derivative)
{
    const float u_squared = u * u;
    value = ((c3 * u + c2) * u + c1) * u + c0;
    world_derivative =
        (c1 + 2.0f * c2 * u + 3.0f * c3 * u_squared) *
        inverse_duration;
}

bool evaluate_rotation(
    animation_channel_gpu_record channel,
    float u,
    float inverse_duration,
    out float4 value,
    out float4 world_derivative)
{
    const uint interpolation_mode =
        (channel.interpolation_flags >> 2u) & 3u;
    float4 derivative_u = 0.0f;

    if (interpolation_mode == 1u)
    {
        float4 start_value;
        float4 end_value;
        float ignored_length = 0.0f;
        if (!normalize_quaternion(
                channel.rotation_parameter0, start_value, ignored_length) ||
            !normalize_quaternion(
                channel.rotation_parameter1, end_value, ignored_length))
        {
            return false;
        }

        float cosine = dot(start_value, end_value);
        if (cosine < 0.0f)
        {
            end_value = -end_value;
            cosine = -cosine;
        }
        cosine = clamp(cosine, -1.0f, 1.0f);

        if (cosine > 0.9995f)
        {
            const float4 raw =
                start_value * (1.0f - u) + end_value * u;
            const float4 raw_derivative = end_value - start_value;
            float raw_length = 0.0f;
            if (!normalize_quaternion(raw, value, raw_length))
            {
                return false;
            }
            derivative_u =
                (raw_derivative - value * dot(value, raw_derivative)) /
                raw_length;
        }
        else
        {
            const float angle = acos(cosine);
            const float sine = sin(angle);
            if (!isfinite(angle) || !isfinite(sine) ||
                abs(sine) <= 1.0e-12f)
            {
                return false;
            }
            const float4 tangent =
                (end_value - start_value * cosine) / sine;
            const float angle_u = angle * u;
            const float4 raw =
                start_value * cos(angle_u) +
                tangent * sin(angle_u);
            const float4 raw_derivative =
                angle * (-start_value * sin(angle_u) +
                    tangent * cos(angle_u));
            float raw_length = 0.0f;
            if (!normalize_quaternion(raw, value, raw_length))
            {
                return false;
            }
            derivative_u =
                (raw_derivative - value * dot(value, raw_derivative)) /
                raw_length;
        }
    }
    else if (interpolation_mode == 2u)
    {
        const float u_squared = u * u;
        const float4 raw =
            ((channel.rotation_parameter3 * u +
                channel.rotation_parameter2) * u +
                channel.rotation_parameter1) * u +
            channel.rotation_parameter0;
        const float4 raw_derivative =
            channel.rotation_parameter1 +
            2.0f * channel.rotation_parameter2 * u +
            3.0f * channel.rotation_parameter3 * u_squared;
        float raw_length = 0.0f;
        if (!normalize_quaternion(raw, value, raw_length))
        {
            return false;
        }
        derivative_u =
            (raw_derivative - value * dot(value, raw_derivative)) /
            raw_length;
    }
    else
    {
        float ignored_length = 0.0f;
        if (!normalize_quaternion(
                channel.rotation_parameter0, value, ignored_length))
        {
            return false;
        }
    }

    world_derivative = derivative_u * inverse_duration;
    return finite4(value) && finite4(world_derivative);
}

void fail_channel(uint output_index)
{
    animation_node_local_transform_gpu_record failed = (animation_node_local_transform_gpu_record)0;
    failed.status = 1u;
    local_transforms[output_index] = failed;
    uint previous_status = 0u;
    InterlockedOr(evaluation_status[0], 1u, previous_status);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint local_index = dispatch_thread_id.x;
    if (local_index >= channel_count)
    {
        return;
    }

    const animation_span_cursor_gpu_record cursor =
        animation_cursor[0];
    if (cursor.span_record_index != expected_span_record_index)
    {
        fail_channel(local_index);
        return;
    }
    const animation_span_gpu_record span =
        animation_spans[cursor.span_record_index];
    const uint version_low = span.animation_input_version_low;
    const uint version_high = span.animation_input_version_high;
    const float span_reciprocal_error =
        span.span_duration_seconds * span.inverse_span_duration - 1.0f;
    if (cursor.animation_input_version_low != version_low ||
        cursor.animation_input_version_high != version_high ||
        !isfinite(cursor.u_segment_start) ||
        cursor.u_segment_start < 0.0f ||
        cursor.u_segment_start > 1.0f ||
        !isfinite(span.span_duration_seconds) ||
        span.span_duration_seconds <= 0.0f ||
        !isfinite(span.inverse_span_duration) ||
        span.inverse_span_duration <= 0.0f ||
        !isfinite(span_reciprocal_error) ||
        abs(span_reciprocal_error) > 2.0e-5f ||
        first_channel_record < span.first_channel_record ||
        first_channel_record - span.first_channel_record >
            span.channel_record_count ||
        channel_count >
            span.channel_record_count -
                (first_channel_record - span.first_channel_record))
    {
        fail_channel(local_index);
        return;
    }
    const float span_local_u = cursor.u_segment_start;
    const float inverse_span_duration = span.inverse_span_duration;

    const animation_channel_gpu_record channel =
        animation_channels[first_channel_record + local_index];
    animation_node_local_transform_gpu_record output_record =
        (animation_node_local_transform_gpu_record)0;
    output_record.node_index = channel.node_index;
    output_record.parent_index = channel.parent_index;
    output_record.node_role_flags = channel.node_role_flags;

    const uint translation_mode =
        channel.interpolation_flags & 3u;
    const uint rotation_mode =
        (channel.interpolation_flags >> 2u) & 3u;
    const uint scale_mode =
        (channel.interpolation_flags >> 4u) & 3u;
    if (channel.node_index == 0xffffffffu ||
        (channel.parent_index != 0xffffffffu &&
            channel.parent_index >= channel_count) ||
        (channel.channel_mask & ~(CHANNEL_TRANSLATION |
            CHANNEL_ROTATION | CHANNEL_SCALE)) != 0u ||
        (channel.interpolation_flags & ~63u) != 0u ||
        translation_mode > 2u ||
        rotation_mode > 2u ||
        scale_mode > 2u)
    {
        fail_channel(local_index);
        return;
    }

    evaluate_vector_polynomial(
        channel.translation_c0,
        channel.translation_c1,
        channel.translation_c2,
        channel.translation_c3,
        span_local_u,
        inverse_span_duration,
        output_record.translation,
        output_record.translation_velocity);
    evaluate_vector_polynomial(
        channel.scale_c0,
        channel.scale_c1,
        channel.scale_c2,
        channel.scale_c3,
        span_local_u,
        inverse_span_duration,
        output_record.scale,
        output_record.scale_velocity);

    if (!evaluate_rotation(
            channel,
            span_local_u,
            inverse_span_duration,
            output_record.rotation,
            output_record.rotation_velocity) ||
        !finite4(output_record.translation) ||
        !finite4(output_record.translation_velocity) ||
        !finite4(output_record.scale) ||
        !finite4(output_record.scale_velocity))
    {
        fail_channel(local_index);
        return;
    }

    output_record.status = 0u;
    local_transforms[local_index] = output_record;
}
