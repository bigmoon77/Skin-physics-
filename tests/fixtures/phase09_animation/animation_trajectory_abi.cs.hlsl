struct AnimationSpanRecord
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

struct AnimationSpanCursorRecord
{
    uint span_record_index;
    uint animation_input_version_low;
    uint animation_input_version_high;
    float u_segment_start;
};

struct AnimationTrackRecord
{
    uint clip_index;
    uint first_channel_record;
    uint channel_record_count;
    uint flags;
    float clip_time_start;
    float clip_time_delta;
    float playback_speed;
    float reserved0;
};

struct AnimationChannelRecord
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

struct AnimationBlendRecord
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

struct GpuMatrix4x4
{
    float4 column0;
    float4 column1;
    float4 column2;
    float4 column3;
};

struct KinematicColliderRecord
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

StructuredBuffer<AnimationSpanRecord> span_records : register(t0);
StructuredBuffer<AnimationSpanCursorRecord> cursor_records : register(t1);
StructuredBuffer<AnimationTrackRecord> track_records : register(t2);
StructuredBuffer<AnimationChannelRecord> channel_records : register(t3);
StructuredBuffer<AnimationBlendRecord> blend_records : register(t4);
StructuredBuffer<GpuMatrix4x4> inverse_bind_matrices : register(t5);
StructuredBuffer<GpuMatrix4x4> static_node_matrices : register(t6);
StructuredBuffer<KinematicColliderRecord> collider_records : register(t7);

RWStructuredBuffer<AnimationSpanRecord> span_records_out : register(u0);
RWStructuredBuffer<AnimationSpanCursorRecord> cursor_records_out : register(u1);
RWStructuredBuffer<AnimationTrackRecord> track_records_out : register(u2);
RWStructuredBuffer<AnimationChannelRecord> channel_records_out : register(u3);
RWStructuredBuffer<AnimationBlendRecord> blend_records_out : register(u4);
RWStructuredBuffer<GpuMatrix4x4> inverse_bind_matrices_out : register(u5);
RWStructuredBuffer<GpuMatrix4x4> static_node_matrices_out : register(u6);
RWStructuredBuffer<KinematicColliderRecord> collider_records_out : register(u7);

// Byte-for-byte structured-record ABI probe. C++ provides one record for each
// buffer, and this shader reads and writes every field through the declared
// Shader Model 5 StructuredBuffer stride.
[numthreads(1, 1, 1)]
void copy_animation_trajectory_abi(uint3 dispatch_id : SV_DispatchThreadID)
{
    span_records_out[0] = span_records[0];
    cursor_records_out[0] = cursor_records[0];
    track_records_out[0] = track_records[0];
    channel_records_out[0] = channel_records[0];
    blend_records_out[0] = blend_records[0];
    inverse_bind_matrices_out[0] = inverse_bind_matrices[0];
    static_node_matrices_out[0] = static_node_matrices[0];
    collider_records_out[0] = collider_records[0];
}
