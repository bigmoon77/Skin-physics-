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

struct kinematic_collider_gpu_record
{
    uint stable_collider_id_low;
    uint stable_collider_id_high;
    uint span_sample_pair_id;
    uint flags;
    float4 translation_start;
    float4 translation_end;
    float4 rotation_start;
    float4 rotation_end;
    float4 angular_axis_and_angle;
    float duration_seconds;
    float inverse_duration;
    float linear_speed_bound;
    float angular_speed_bound;
    float maximum_local_radius;
    float sagitta_bound;
    float reserved0;
    float reserved1;
};

struct kinematic_collider_sample_gpu_record
{
    float4 translation;
    float4 rotation;
    float4 linear_velocity;
    float4 angular_velocity;
    float4 contact_point_velocity;
    uint status;
    uint reserved0;
    uint reserved1;
    uint reserved2;
};

cbuffer KinematicColliderEvaluateParameters : register(b0)
{
    uint span_count;
    uint collider_count;
    float tau_seconds;
    uint reserved0;
};

StructuredBuffer<animation_span_gpu_record> animation_spans : register(t0);
StructuredBuffer<animation_span_cursor_gpu_record> animation_cursor : register(t1);
StructuredBuffer<kinematic_collider_gpu_record> collider_records : register(t2);
StructuredBuffer<float4> local_contact_points : register(t3);
RWStructuredBuffer<kinematic_collider_sample_gpu_record> collider_samples : register(u0);

static const uint INVALID_INDEX = 0xffffffffu;
static const uint SPAN_FLAG_HAS_KINEMATIC_COLLIDER = 1u << 1u;
static const uint COLLIDER_FLAGS_VALID = 3u;
static const uint STATUS_INVALID_INPUT = 1u;
static const float QUATERNION_EPSILON = 1.0e-12f;
static const float SPAN_BOUNDARY_EPSILON = 9.5367431640625e-7f;

bool finite3(float3 value)
{
    return all(isfinite(value));
}

bool finite4(float4 value)
{
    return all(isfinite(value));
}

float4 normalize_quaternion(float4 value, out bool valid)
{
    const float squared = dot(value, value);
    valid = isfinite(squared) && squared > QUATERNION_EPSILON;
    if (!valid)
    {
        return float4(0.0f, 0.0f, 0.0f, 1.0f);
    }
    return value * rsqrt(squared);
}

float4 slerp_shortest_arc(float4 start_value, float4 end_value, float u)
{
    bool start_valid = false;
    bool end_valid = false;
    const float4 start = normalize_quaternion(start_value, start_valid);
    float4 end = normalize_quaternion(end_value, end_valid);
    if (!start_valid || !end_valid)
    {
        return float4(0.0f, 0.0f, 0.0f, 0.0f);
    }

    float cosine = dot(start, end);
    if (cosine < 0.0f)
    {
        end = -end;
        cosine = -cosine;
    }
    cosine = clamp(cosine, -1.0f, 1.0f);
    if (cosine > 0.9995f)
    {
        bool result_valid = false;
        return normalize_quaternion(lerp(start, end, u), result_valid);
    }

    const float angle = acos(cosine);
    const float sine = sin(angle);
    if (!isfinite(sine) || abs(sine) <= QUATERNION_EPSILON)
    {
        return start;
    }
    const float start_weight = sin((1.0f - u) * angle) / sine;
    const float end_weight = sin(u * angle) / sine;
    bool result_valid = false;
    return normalize_quaternion(
        start * start_weight + end * end_weight, result_valid);
}

float3 rotate_vector(float4 rotation, float3 value)
{
    const float3 q = rotation.xyz;
    const float3 t = 2.0f * cross(q, value);
    return value + rotation.w * t + cross(q, t);
}

bool valid_collider_record(
    kinematic_collider_gpu_record record_value,
    float4 local_contact_point,
    float u)
{
    const float start_length = dot(
        record_value.rotation_start, record_value.rotation_start);
    const float end_length = dot(
        record_value.rotation_end, record_value.rotation_end);
    const float axis_length = length(record_value.angular_axis_and_angle.xyz);
    const float angle = record_value.angular_axis_and_angle.w;
    const float3 translation_delta =
        record_value.translation_end.xyz - record_value.translation_start.xyz;
    const float expected_linear_speed =
        length(translation_delta) * record_value.inverse_duration;
    const float expected_angular_speed = angle * record_value.inverse_duration;
    const float expected_sagitta =
        record_value.maximum_local_radius * (1.0f - cos(angle * 0.5f));
    const float contact_radius = length(local_contact_point.xyz);

    return (record_value.stable_collider_id_low != 0u ||
            record_value.stable_collider_id_high != 0u) &&
        record_value.span_sample_pair_id != INVALID_INDEX &&
        record_value.flags == COLLIDER_FLAGS_VALID &&
        finite4(record_value.translation_start) &&
        finite4(record_value.translation_end) &&
        finite4(record_value.rotation_start) &&
        finite4(record_value.rotation_end) &&
        finite4(record_value.angular_axis_and_angle) &&
        isfinite(start_length) && abs(start_length - 1.0f) <= 1.0e-4f &&
        isfinite(end_length) && abs(end_length - 1.0f) <= 1.0e-4f &&
        isfinite(axis_length) && isfinite(angle) &&
        angle >= 0.0f && angle <= 3.1416927f &&
        (angle > QUATERNION_EPSILON
            ? abs(axis_length - 1.0f) <= 1.0e-3f
            : axis_length <= 1.0e-3f) &&
        isfinite(record_value.duration_seconds) &&
        record_value.duration_seconds > 0.0f &&
        isfinite(record_value.inverse_duration) &&
        record_value.inverse_duration > 0.0f &&
        abs(record_value.duration_seconds *
            record_value.inverse_duration - 1.0f) <= 1.0e-4f &&
        isfinite(record_value.linear_speed_bound) &&
        record_value.linear_speed_bound + 2.0e-5f *
            max(1.0f, expected_linear_speed) >= expected_linear_speed &&
        isfinite(record_value.angular_speed_bound) &&
        record_value.angular_speed_bound + 2.0e-5f *
            max(1.0f, expected_angular_speed) >= expected_angular_speed &&
        isfinite(record_value.maximum_local_radius) &&
        record_value.maximum_local_radius >= 0.0f &&
        isfinite(record_value.sagitta_bound) &&
        record_value.sagitta_bound + 2.0e-5f *
            max(1.0f, expected_sagitta) >= expected_sagitta &&
        isfinite(record_value.reserved0) &&
        record_value.reserved0 == 0.0f &&
        isfinite(record_value.reserved1) &&
        record_value.reserved1 == 0.0f &&
        finite4(local_contact_point) &&
        isfinite(contact_radius) &&
        contact_radius <= record_value.maximum_local_radius + 1.0e-5f &&
        isfinite(u) && u >= 0.0f && u <= 1.0f;
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint local_index = dispatch_thread_id.x;
    if (local_index >= collider_count)
    {
        return;
    }

    kinematic_collider_sample_gpu_record output_value =
        (kinematic_collider_sample_gpu_record)0;
    const animation_span_cursor_gpu_record cursor = animation_cursor[0];
    uint span_buffer_count = 0u;
    uint span_stride = 0u;
    uint collider_buffer_count = 0u;
    uint collider_stride = 0u;
    uint contact_buffer_count = 0u;
    uint contact_stride = 0u;
    animation_spans.GetDimensions(span_buffer_count, span_stride);
    collider_records.GetDimensions(collider_buffer_count, collider_stride);
    local_contact_points.GetDimensions(contact_buffer_count, contact_stride);

    if (span_count == 0u ||
        cursor.span_record_index == INVALID_INDEX ||
        cursor.span_record_index >= span_count ||
        cursor.span_record_index >= span_buffer_count ||
        !isfinite(tau_seconds) || tau_seconds < 0.0f ||
        !isfinite(cursor.u_segment_start) ||
        cursor.u_segment_start < 0.0f ||
        cursor.u_segment_start > 1.0f)
    {
        output_value.status = STATUS_INVALID_INPUT;
        collider_samples[local_index] = output_value;
        return;
    }

    const animation_span_gpu_record span =
        animation_spans[cursor.span_record_index];
    const float u = cursor.u_segment_start +
        tau_seconds * span.inverse_span_duration;
    const uint collider_end = span.first_collider_record +
        span.collider_record_count;
    if (span.animation_input_version_low !=
            cursor.animation_input_version_low ||
        span.animation_input_version_high !=
            cursor.animation_input_version_high ||
        (span.flags & SPAN_FLAG_HAS_KINEMATIC_COLLIDER) == 0u ||
        span.collider_record_count != collider_count ||
        span.first_collider_record > collider_buffer_count ||
        span.collider_record_count >
            collider_buffer_count - span.first_collider_record ||
        collider_end < span.first_collider_record ||
        local_index >= contact_buffer_count ||
        !isfinite(span.span_duration_seconds) ||
        span.span_duration_seconds <= 0.0f ||
        !isfinite(span.inverse_span_duration) ||
        span.inverse_span_duration <= 0.0f ||
        abs(span.span_duration_seconds *
            span.inverse_span_duration - 1.0f) > 2.0e-5f ||
        !isfinite(u) || u < 0.0f ||
        u > 1.0f + SPAN_BOUNDARY_EPSILON)
    {
        output_value.status = STATUS_INVALID_INPUT;
        collider_samples[local_index] = output_value;
        return;
    }

    const kinematic_collider_gpu_record collider =
        collider_records[span.first_collider_record + local_index];
    const float4 local_contact_point = local_contact_points[local_index];
    if (!valid_collider_record(collider, local_contact_point, min(u, 1.0f)))
    {
        output_value.status = STATUS_INVALID_INPUT;
        collider_samples[local_index] = output_value;
        return;
    }

    const float sample_u = min(u, 1.0f);
    const float4 rotation = slerp_shortest_arc(
        collider.rotation_start, collider.rotation_end, sample_u);
    const float3 translation = lerp(
        collider.translation_start.xyz, collider.translation_end.xyz, sample_u);
    const float3 linear_velocity =
        (collider.translation_end.xyz - collider.translation_start.xyz) *
        collider.inverse_duration;
    const float3 angular_velocity =
        collider.angular_axis_and_angle.xyz *
        (collider.angular_axis_and_angle.w * collider.inverse_duration);
    const float3 world_contact_radius =
        rotate_vector(rotation, local_contact_point.xyz);
    const float3 contact_point_velocity =
        linear_velocity + cross(angular_velocity, world_contact_radius);

    if (!finite4(rotation) || !finite3(translation) ||
        !finite3(linear_velocity) || !finite3(angular_velocity) ||
        !finite3(contact_point_velocity))
    {
        output_value.status = STATUS_INVALID_INPUT;
        collider_samples[local_index] = output_value;
        return;
    }

    output_value.translation = float4(translation, 1.0f);
    output_value.rotation = rotation;
    output_value.linear_velocity = float4(linear_velocity, 0.0f);
    output_value.angular_velocity = float4(angular_velocity, 0.0f);
    output_value.contact_point_velocity = float4(contact_point_velocity, 0.0f);
    output_value.status = 0u;
    collider_samples[local_index] = output_value;
}
