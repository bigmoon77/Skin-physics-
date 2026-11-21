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
    uint2 reserved;
    float4 root_transform_column0;
    float4 root_transform_column1;
    float4 root_transform_column2;
    float4 root_transform_column3;
};

StructuredBuffer<gpu_matrix4x4> global_matrices : register(t0);
StructuredBuffer<gpu_matrix4x4> global_matrix_derivatives : register(t1);
StructuredBuffer<animation_skin_gpu_record> skin_records : register(t2);
StructuredBuffer<animation_skin_joint_gpu_record> skin_joint_records : register(t3);
StructuredBuffer<gpu_matrix4x4> inverse_bind_matrices : register(t4);
StructuredBuffer<animation_node_local_transform_gpu_record> local_transforms : register(t5);

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

    gpu_matrix4x4 root_transform;
    root_transform.column0 = root_transform_column0;
    root_transform.column1 = root_transform_column1;
    root_transform.column2 = root_transform_column2;
    root_transform.column3 = root_transform_column3;
    if (!finite_matrix(root_transform) ||
        !affine_matrix(root_transform))
    {
        fail_joint(local_joint_index);
        return;
    }

    const gpu_matrix4x4 world_skin_matrix =
        multiply_matrix(root_transform, inverse_mesh_with_joint);
    const gpu_matrix4x4 world_skin_derivative =
        multiply_matrix(root_transform, skin_derivative);
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
