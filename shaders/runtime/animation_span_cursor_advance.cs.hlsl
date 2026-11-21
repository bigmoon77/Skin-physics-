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

cbuffer AnimationSpanCursorAdvanceParameters : register(b0)
{
    uint span_count;
    float segment_duration_seconds;
    uint reserved0;
    uint reserved1;
};

StructuredBuffer<animation_span_gpu_record> animation_spans : register(t0);
RWStructuredBuffer<animation_span_cursor_gpu_record> animation_cursor : register(u0);
RWStructuredBuffer<uint> cursor_status : register(u1);

static const uint INVALID_INDEX = 0xffffffffu;
static const uint STATUS_ANIMATION_CURSOR = 1u << 5u;
static const float FLOAT_EPSILON = 1.192092896e-7f;
static const float SPAN_BOUNDARY_EPSILON = 8.0f * FLOAT_EPSILON;

void fail_cursor()
{
    uint ignored = 0u;
    InterlockedOr(cursor_status[0], STATUS_ANIMATION_CURSOR, ignored);
}

[numthreads(1, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    animation_span_cursor_gpu_record cursor = animation_cursor[0];

    if (span_count == 0u ||
        cursor.span_record_index == INVALID_INDEX ||
        cursor.span_record_index >= span_count ||
        !isfinite(segment_duration_seconds) ||
        segment_duration_seconds <= 0.0f ||
        !isfinite(cursor.u_segment_start) ||
        cursor.u_segment_start < 0.0f ||
        cursor.u_segment_start > 1.0f)
    {
        fail_cursor();
        return;
    }

    const animation_span_gpu_record span =
        animation_spans[cursor.span_record_index];
    if (span.animation_input_version_low !=
            cursor.animation_input_version_low ||
        span.animation_input_version_high !=
            cursor.animation_input_version_high ||
        !isfinite(span.span_duration_seconds) ||
        span.span_duration_seconds <= 0.0f ||
        !isfinite(span.inverse_span_duration) ||
        span.inverse_span_duration <= 0.0f ||
        abs(span.span_duration_seconds *
            span.inverse_span_duration - 1.0f) > 2.0e-5f)
    {
        fail_cursor();
        return;
    }

    const float delta_u =
        segment_duration_seconds * span.inverse_span_duration;
    const float next_u = cursor.u_segment_start + delta_u;
    if (!isfinite(delta_u) || delta_u <= 0.0f ||
        !isfinite(next_u) ||
        next_u > 1.0f + SPAN_BOUNDARY_EPSILON)
    {
        fail_cursor();
        return;
    }

    if (next_u >= 1.0f - SPAN_BOUNDARY_EPSILON)
    {
        if (cursor.span_record_index + 1u < span_count)
        {
            const uint next_span_index = cursor.span_record_index + 1u;
            const animation_span_gpu_record next_span =
                animation_spans[next_span_index];
            if (next_span.animation_input_version_low !=
                    cursor.animation_input_version_low ||
                next_span.animation_input_version_high !=
                    cursor.animation_input_version_high ||
                !isfinite(next_span.span_duration_seconds) ||
                next_span.span_duration_seconds <= 0.0f ||
                !isfinite(next_span.inverse_span_duration) ||
                next_span.inverse_span_duration <= 0.0f)
            {
                fail_cursor();
                return;
            }

            cursor.span_record_index = next_span_index;
            cursor.u_segment_start = 0.0f;
        }
        else
        {
            cursor.u_segment_start = 1.0f;
        }
    }
    else
    {
        cursor.u_segment_start = next_u;
    }

    animation_cursor[0] = cursor;
}
