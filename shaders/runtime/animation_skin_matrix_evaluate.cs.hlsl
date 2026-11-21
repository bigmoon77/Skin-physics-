struct gpu_matrix4x4
{
    float4 column0;
    float4 column1;
    float4 column2;
    float4 column3;
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

struct root_transform_gpu_record
{
    float4 translation_start;
    float4 translation_end;
    float4 rotation_start;
    float4 rotation_end;
    float4 angular_axis_and_angle;
    float duration_seconds;
    float inverse_duration;
    uint flags;
    uint reserved;
};

struct animation_skin_gpu_record
{
    uint mesh_node_channel_offset;
    uint skin_index;
    uint first_joint_record;
    uint joint_count;
};

struct animation_skin_joint_gpu_record
{
    uint joint_channel_offset;
    uint inverse_bind_record_index;
    uint joint_node_index;
    uint reserved;
};

cbuffer AnimationSkinMatrixParameters : register(b0)
{
    uint skin_record_index;
    uint skin_record_count;
    uint track_channel_count;
    uint skin_joint_record_count;
    uint inverse_bind_matrix_count;
    uint expected_mesh_node_index;
    uint root_transform_record_index;
    uint root_transform_record_count;
    uint expected_span_record_index;
    uint3 reserved;
};

StructuredBuffer<gpu_matrix4x4> global_matrices : register(t0);
StructuredBuffer<gpu_matrix4x4> global_matrix_derivatives : register(t1);
StructuredBuffer<animation_skin_gpu_record> skin_records : register(t2);
StructuredBuffer<animation_skin_joint_gpu_record> skin_joint_records : register(t3);
StructuredBuffer<gpu_matrix4x4> inverse_bind_matrices : register(t4);
StructuredBuffer<animation_node_local_transform_gpu_record> local_transforms : register(t5);
StructuredBuffer<root_transform_gpu_record> root_transform_records : register(t6);
StructuredBuffer<animation_span_cursor_gpu_record> animation_cursor : register(t7);
StructuredBuffer<animation_span_gpu_record> animation_spans : register(t8);

RWStructuredBuffer<gpu_matrix4x4> skin_matrices : register(u0);
RWStructuredBuffer<gpu_matrix4x4> skin_matrix_derivatives : register(u1);
RWStructuredBuffer<uint> skin_matrix_status : register(u2);

bool finite4(float4 value)
{
    return all(isfinite(value));
}

bool finite_matrix(gpu_matrix4x4 value)
{
    return finite4(value.column0) &&
        finite4(value.column1) &&
        finite4(value.column2) &&
        finite4(value.column3);
}

bool affine_matrix(gpu_matrix4x4 value)
{
    const float tolerance = 1.0e-4f;
    return abs(value.column0.w) <= tolerance &&
        abs(value.column1.w) <= tolerance &&
        abs(value.column2.w) <= tolerance &&
        abs(value.column3.w - 1.0f) <= tolerance;
}

bool affine_derivative_matrix(gpu_matrix4x4 value)
{
    const float tolerance = 1.0e-4f;
    return abs(value.column0.w) <= tolerance &&
        abs(value.column1.w) <= tolerance &&
        abs(value.column2.w) <= tolerance &&
        abs(value.column3.w) <= tolerance;
}

float4 transform_column_major(gpu_matrix4x4 matrix_value, float4 value)
{
    return matrix_value.column0 * value.x +
        matrix_value.column1 * value.y +
        matrix_value.column2 * value.z +
        matrix_value.column3 * value.w;
}

gpu_matrix4x4 multiply_matrix(
    gpu_matrix4x4 left,
    gpu_matrix4x4 right)
{
    gpu_matrix4x4 result;
    result.column0 = transform_column_major(left, right.column0);
    result.column1 = transform_column_major(left, right.column1);
    result.column2 = transform_column_major(left, right.column2);
    result.column3 = transform_column_major(left, right.column3);
    return result;
}

gpu_matrix4x4 add_matrix(
    gpu_matrix4x4 left,
    gpu_matrix4x4 right)
{
    gpu_matrix4x4 result;
    result.column0 = left.column0 + right.column0;
    result.column1 = left.column1 + right.column1;
    result.column2 = left.column2 + right.column2;
    result.column3 = left.column3 + right.column3;
    return result;
}

gpu_matrix4x4 negate_matrix(gpu_matrix4x4 value)
{
    gpu_matrix4x4 result;
    result.column0 = -value.column0;
    result.column1 = -value.column1;
    result.column2 = -value.column2;
    result.column3 = -value.column3;
    return result;
}

bool invert_affine_matrix(
    gpu_matrix4x4 input_matrix,
    out gpu_matrix4x4 inverse_matrix)
{
    inverse_matrix = (gpu_matrix4x4)0;
    if (!finite_matrix(input_matrix) || !affine_matrix(input_matrix))
    {
        return false;
    }

    const float3 a = input_matrix.column0.xyz;
    const float3 b = input_matrix.column1.xyz;
    const float3 c = input_matrix.column2.xyz;
    const float3 translation = input_matrix.column3.xyz;
    const float3 row0 = cross(b, c);
    const float3 row1 = cross(c, a);
    const float3 row2 = cross(a, b);
    const float determinant = dot(a, row0);
    const float scale = max(length(a), max(length(b), length(c)));
    const float determinant_tolerance =
        1.0e-12f * max(1.0f, scale * scale * scale);
    if (!isfinite(determinant) || !isfinite(scale) ||
        scale <= 0.0f ||
        abs(determinant) <= determinant_tolerance)
    {
        return false;
    }

    const float inverse_determinant = rcp(determinant);
    const float3 inverse_row0 = row0 * inverse_determinant;
    const float3 inverse_row1 = row1 * inverse_determinant;
    const float3 inverse_row2 = row2 * inverse_determinant;

    inverse_matrix.column0 = float4(
        inverse_row0.x, inverse_row1.x, inverse_row2.x, 0.0f);
    inverse_matrix.column1 = float4(
        inverse_row0.y, inverse_row1.y, inverse_row2.y, 0.0f);
    inverse_matrix.column2 = float4(
        inverse_row0.z, inverse_row1.z, inverse_row2.z, 0.0f);
    inverse_matrix.column3 = float4(
        -dot(inverse_row0, translation),
        -dot(inverse_row1, translation),
        -dot(inverse_row2, translation),
        1.0f);
    return finite_matrix(inverse_matrix);
}

bool evaluate_root_transform(
    root_transform_gpu_record record_value,
    animation_span_cursor_gpu_record cursor_value,
    animation_span_gpu_record span_value,
    out gpu_matrix4x4 root_transform,
    out gpu_matrix4x4 root_transform_derivative)
{
    root_transform = (gpu_matrix4x4)0;
    root_transform_derivative = (gpu_matrix4x4)0;
    const uint expected_flags = 3u;
    if (record_value.flags != expected_flags ||
        record_value.reserved != 0u ||
        cursor_value.span_record_index != expected_span_record_index ||
        cursor_value.animation_input_version_low !=
            span_value.animation_input_version_low ||
        cursor_value.animation_input_version_high !=
            span_value.animation_input_version_high ||
        !isfinite(cursor_value.u_segment_start) ||
        cursor_value.u_segment_start < 0.0f ||
        cursor_value.u_segment_start > 1.0f ||
        !isfinite(record_value.duration_seconds) ||
        record_value.duration_seconds <= 0.0f ||
        !isfinite(record_value.inverse_duration) ||
        record_value.inverse_duration <= 0.0f ||
        abs(record_value.duration_seconds -
            span_value.span_duration_seconds) >
            1.0e-5f * max(1.0f, span_value.span_duration_seconds) ||
        abs(record_value.duration_seconds *
            record_value.inverse_duration - 1.0f) > 1.0e-4f ||
        !finite4(record_value.translation_start) ||
        !finite4(record_value.translation_end) ||
        !finite4(record_value.rotation_start) ||
        !finite4(record_value.rotation_end) ||
        !finite4(record_value.angular_axis_and_angle) ||
        abs(record_value.translation_start.w - 1.0f) > 1.0e-5f ||
        abs(record_value.translation_end.w - 1.0f) > 1.0e-5f)
    {
        return false;
    }

    const float start_length = length(record_value.rotation_start);
    const float end_length = length(record_value.rotation_end);
    const float axis_length = length(record_value.angular_axis_and_angle.xyz);
    const float angle = record_value.angular_axis_and_angle.w;
    if (!isfinite(start_length) || !isfinite(end_length) ||
        abs(start_length - 1.0f) > 1.0e-4f ||
        abs(end_length - 1.0f) > 1.0e-4f ||
        !isfinite(axis_length) ||
        !isfinite(angle) || angle < 0.0f || angle > 3.1416927f ||
        (angle > 1.0e-6f
            ? abs(axis_length - 1.0f) > 1.0e-3f
            : axis_length > 1.0e-3f))
    {
        return false;
    }

    float4 q0 = record_value.rotation_start / start_length;
    float4 q1 = record_value.rotation_end / end_length;
    float cosine = dot(q0, q1);
    if (cosine < 0.0f)
    {
        q1 = -q1;
        cosine = -cosine;
    }
    cosine = clamp(cosine, -1.0f, 1.0f);

    const float u = cursor_value.u_segment_start;
    float4 rotation;
    if (cosine > 0.9995f)
    {
        const float4 raw = lerp(q0, q1, u);
        const float raw_length = length(raw);
        if (!isfinite(raw_length) || raw_length <= 1.0e-12f)
        {
            return false;
        }
        rotation = raw / raw_length;
    }
    else
    {
        const float interpolation_angle = acos(cosine);
        const float interpolation_sine = sin(interpolation_angle);
        if (!isfinite(interpolation_sine) ||
            abs(interpolation_sine) <= 1.0e-12f)
        {
            return false;
        }
        const float4 tangent =
            (q1 - cosine * q0) / interpolation_sine;
        rotation =
            q0 * cos(interpolation_angle * u) +
            tangent * sin(interpolation_angle * u);
    }

    const float rotation_length = length(rotation);
    if (!isfinite(rotation_length) || rotation_length <= 1.0e-12f)
    {
        return false;
    }
    rotation /= rotation_length;
    const float x = rotation.x;
    const float y = rotation.y;
    const float z = rotation.z;
    const float w = rotation.w;

    root_transform.column0 = float4(
        1.0f - 2.0f * (y * y + z * z),
        2.0f * (x * y + z * w),
        2.0f * (x * z - y * w),
        0.0f);
    root_transform.column1 = float4(
        2.0f * (x * y - z * w),
        1.0f - 2.0f * (x * x + z * z),
        2.0f * (y * z + x * w),
        0.0f);
    root_transform.column2 = float4(
        2.0f * (x * z + y * w),
        2.0f * (y * z - x * w),
        1.0f - 2.0f * (x * x + y * y),
        0.0f);
    root_transform.column3 = float4(
        lerp(
            record_value.translation_start.xyz,
            record_value.translation_end.xyz,
            u),
        1.0f);

    const float3 angular_velocity =
        record_value.angular_axis_and_angle.xyz *
        (angle * record_value.inverse_duration);
    const float3 linear_velocity =
        (record_value.translation_end.xyz -
            record_value.translation_start.xyz) *
        record_value.inverse_duration;
    root_transform_derivative.column0 = float4(
        cross(angular_velocity, root_transform.column0.xyz), 0.0f);
    root_transform_derivative.column1 = float4(
        cross(angular_velocity, root_transform.column1.xyz), 0.0f);
    root_transform_derivative.column2 = float4(
        cross(angular_velocity, root_transform.column2.xyz), 0.0f);
    root_transform_derivative.column3 = float4(linear_velocity, 0.0f);

    return finite_matrix(root_transform) &&
        affine_matrix(root_transform) &&
        finite_matrix(root_transform_derivative) &&
        affine_derivative_matrix(root_transform_derivative);
}

void fail_joint(uint local_joint_index)
{
    gpu_matrix4x4 zero_matrix = (gpu_matrix4x4)0;
    skin_matrices[local_joint_index] = zero_matrix;
    skin_matrix_derivatives[local_joint_index] = zero_matrix;
    uint previous_status = 0u;
    InterlockedOr(skin_matrix_status[0], 8u, previous_status);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    if (skin_record_index >= skin_record_count)
    {
        uint previous_status = 0u;
        InterlockedOr(skin_matrix_status[0], 8u, previous_status);
        return;
    }

    const animation_skin_gpu_record skin =
        skin_records[skin_record_index];
    const uint local_joint_index = dispatch_thread_id.x;
    if (local_joint_index >= skin.joint_count)
    {
        return;
    }

    if (skin.joint_count == 0u ||
        skin.mesh_node_channel_offset >= track_channel_count ||
        skin.first_joint_record > skin_joint_record_count ||
        skin.joint_count >
            skin_joint_record_count - skin.first_joint_record)
    {
        uint previous_status = 0u;
        InterlockedOr(skin_matrix_status[0], 8u, previous_status);
        return;
    }

    const uint joint_record_index =
        skin.first_joint_record + local_joint_index;
    const animation_skin_joint_gpu_record joint =
        skin_joint_records[joint_record_index];
    if (joint.joint_channel_offset >= track_channel_count ||
        joint.inverse_bind_record_index >= inverse_bind_matrix_count ||
        joint.joint_node_index == 0xffffffffu)
    {
        fail_joint(local_joint_index);
        return;
    }

    const animation_node_local_transform_gpu_record mesh_node =
        local_transforms[skin.mesh_node_channel_offset];
    const animation_node_local_transform_gpu_record joint_node =
        local_transforms[joint.joint_channel_offset];
    if (mesh_node.status != 0u ||
        joint_node.status != 0u ||
        mesh_node.node_index != expected_mesh_node_index ||
        joint_node.node_index != joint.joint_node_index)
    {
        fail_joint(local_joint_index);
        return;
    }

    const gpu_matrix4x4 mesh_global =
        global_matrices[skin.mesh_node_channel_offset];
    const gpu_matrix4x4 mesh_global_derivative =
        global_matrix_derivatives[skin.mesh_node_channel_offset];
    const gpu_matrix4x4 joint_global =
        global_matrices[joint.joint_channel_offset];
    const gpu_matrix4x4 joint_global_derivative =
        global_matrix_derivatives[joint.joint_channel_offset];
    const gpu_matrix4x4 inverse_bind =
        inverse_bind_matrices[joint.inverse_bind_record_index];

    gpu_matrix4x4 inverse_mesh;
    if (!invert_affine_matrix(mesh_global, inverse_mesh) ||
        !finite_matrix(mesh_global_derivative) ||
        !affine_derivative_matrix(mesh_global_derivative) ||
        !finite_matrix(joint_global) ||
        !affine_matrix(joint_global) ||
        !finite_matrix(joint_global_derivative) ||
        !affine_derivative_matrix(joint_global_derivative) ||
        !finite_matrix(inverse_bind) ||
        !affine_matrix(inverse_bind))
    {
        fail_joint(local_joint_index);
        return;
    }

    const gpu_matrix4x4 inverse_mesh_derivative =
        negate_matrix(multiply_matrix(
            multiply_matrix(inverse_mesh, mesh_global_derivative),
            inverse_mesh));
    const gpu_matrix4x4 joint_with_bind =
        multiply_matrix(joint_global, inverse_bind);
    const gpu_matrix4x4 inverse_mesh_with_joint =
        multiply_matrix(inverse_mesh, joint_with_bind);
    const gpu_matrix4x4 derivative_first =
        multiply_matrix(inverse_mesh_derivative, joint_with_bind);
    const gpu_matrix4x4 derivative_second =
        multiply_matrix(
            inverse_mesh,
            multiply_matrix(joint_global_derivative, inverse_bind));
    const gpu_matrix4x4 skin_derivative =
        add_matrix(derivative_first, derivative_second);

    if (!finite_matrix(inverse_mesh_derivative) ||
        !finite_matrix(inverse_mesh_with_joint) ||
        !finite_matrix(skin_derivative))
    {
        fail_joint(local_joint_index);
        return;
    }

    if (root_transform_record_index >= root_transform_record_count ||
        root_transform_record_index != expected_span_record_index)
    {
        fail_joint(local_joint_index);
        return;
    }
    const animation_span_cursor_gpu_record cursor_value =
        animation_cursor[0u];
    const animation_span_gpu_record span_value =
        animation_spans[expected_span_record_index];
    const root_transform_gpu_record root_record =
        root_transform_records[root_transform_record_index];
    gpu_matrix4x4 root_transform;
    gpu_matrix4x4 root_transform_derivative;
    if (!evaluate_root_transform(
            root_record, cursor_value, span_value,
            root_transform, root_transform_derivative))
    {
        fail_joint(local_joint_index);
        return;
    }

    const gpu_matrix4x4 world_skin_matrix =
        multiply_matrix(root_transform, inverse_mesh_with_joint);
    const gpu_matrix4x4 world_skin_derivative = add_matrix(
        multiply_matrix(root_transform_derivative, inverse_mesh_with_joint),
        multiply_matrix(root_transform, skin_derivative));
    if (!finite_matrix(world_skin_matrix) ||
        !affine_matrix(world_skin_matrix) ||
        !finite_matrix(world_skin_derivative) ||
        !affine_derivative_matrix(world_skin_derivative))
    {
        fail_joint(local_joint_index);
        return;
    }

    skin_matrices[local_joint_index] = world_skin_matrix;
    skin_matrix_derivatives[local_joint_index] = world_skin_derivative;
}
