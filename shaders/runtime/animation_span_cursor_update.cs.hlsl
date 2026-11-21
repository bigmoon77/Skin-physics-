struct animation_span_cursor_gpu_record
{
    uint span_record_index;
    uint animation_input_version_low;
    uint animation_input_version_high;
    float u_segment_start;
};

cbuffer AnimationSpanCursorUpdateParameters : register(b0)
{
    uint span_record_index;
    uint animation_input_version_low;
    uint animation_input_version_high;
    float u_segment_start;
};

RWStructuredBuffer<animation_span_cursor_gpu_record> animation_span_cursor : register(u0);

[numthreads(1, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (any(dispatch_id != uint3(0u, 0u, 0u)))
    {
        return;
    }

    animation_span_cursor_gpu_record cursor;
    cursor.span_record_index = span_record_index;
    cursor.animation_input_version_low = animation_input_version_low;
    cursor.animation_input_version_high = animation_input_version_high;
    cursor.u_segment_start = u_segment_start;

    if (span_record_index == 0xffffffffu ||
        (animation_input_version_low == 0u &&
            animation_input_version_high == 0u) ||
        !isfinite(u_segment_start) ||
        u_segment_start < 0.0f ||
        u_segment_start > 1.0f)
    {
        cursor.span_record_index = 0xffffffffu;
        cursor.u_segment_start = asfloat(0x7fc00000u);
    }

    animation_span_cursor[0u] = cursor;
}
