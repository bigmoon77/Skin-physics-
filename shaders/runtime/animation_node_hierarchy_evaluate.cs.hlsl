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

cbuffer AnimationHierarchyParameters : register(b0)
{
    uint node_count;
    uint static_node_matrix_count;
    uint hierarchy_level;
    uint reserved;
};

StructuredBuffer<animation_node_local_transform_gpu_record> local_transforms : register(t0);
StructuredBuffer<gpu_matrix4x4> static_node_matrices : register(t1);

RWStructuredBuffer<gpu_matrix4x4> global_matrices : register(u0);
RWStructuredBuffer<gpu_matrix4x4> global_matrix_derivatives : register(u1);
RWStructuredBuffer<uint> hierarchy_status : register(u2);

static const uint INVALID_INDEX = 0xffffffffu;
static const uint NODE_ROLE_STATIC_MATRIX = 1u << 4u;

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

void fail_node(uint node_index)
{
    gpu_matrix4x4 zero_matrix = (gpu_matrix4x4)0;
    global_matrices[node_index] = zero_matrix;
    global_matrix_derivatives[node_index] = zero_matrix;
    uint previous_status = 0u;
    InterlockedOr(hierarchy_status[0], 4u, previous_status);
}

bool build_local_trs_matrices(
    animation_node_local_transform_gpu_record local,
    out gpu_matrix4x4 local_matrix,
    out gpu_matrix4x4 local_derivative)
{
    local_matrix = (gpu_matrix4x4)0;
    local_derivative = (gpu_matrix4x4)0;

    if (!finite4(local.translation) ||
        !finite4(local.translation_velocity) ||
        !finite4(local.rotation) ||
        !finite4(local.rotation_velocity) ||
        !finite4(local.scale) ||
        !finite4(local.scale_velocity) ||
        any(local.scale.xyz <= 0.0f))
    {
        return false;
    }

    const float quaternion_squared = dot(local.rotation, local.rotation);
    if (!isfinite(quaternion_squared) || quaternion_squared <= 1.0e-24f)
    {
        return false;
    }
    const float quaternion_length = sqrt(quaternion_squared);
    if (!isfinite(quaternion_length) || quaternion_length <= 1.0e-12f)
    {
        return false;
    }

    const float4 q = local.rotation / quaternion_length;
    const float4 q_dot =
        (local.rotation_velocity -
            q * dot(q, local.rotation_velocity)) / quaternion_length;
    if (!finite4(q) || !finite4(q_dot))
    {
        return false;
    }

    const float x = q.x;
    const float y = q.y;
    const float z = q.z;
    const float w = q.w;
    const float xd = q_dot.x;
    const float yd = q_dot.y;
    const float zd = q_dot.z;
    const float wd = q_dot.w;

    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r10 = 2.0f * (x * y + z * w);
    const float r20 = 2.0f * (x * z - y * w);
    const float r01 = 2.0f * (x * y - z * w);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r21 = 2.0f * (y * z + x * w);
    const float r02 = 2.0f * (x * z + y * w);
    const float r12 = 2.0f * (y * z - x * w);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);

    const float dr00 = -4.0f * (y * yd + z * zd);
    const float dr10 = 2.0f * (xd * y + x * yd + zd * w + z * wd);
    const float dr20 = 2.0f * (xd * z + x * zd - yd * w - y * wd);
    const float dr01 = 2.0f * (xd * y + x * yd - zd * w - z * wd);
    const float dr11 = -4.0f * (x * xd + z * zd);
    const float dr21 = 2.0f * (yd * z + y * zd - xd * w - x * wd);
    const float dr02 = 2.0f * (xd * z + x * zd + yd * w + y * wd);
    const float dr12 = 2.0f * (yd * z + y * zd + xd * w + x * wd);
    const float dr22 = -4.0f * (x * xd + y * yd);

    const float sx = local.scale.x;
    const float sy = local.scale.y;
    const float sz = local.scale.z;
    const float sxd = local.scale_velocity.x;
    const float syd = local.scale_velocity.y;
    const float szd = local.scale_velocity.z;

    local_matrix.column0 = float4(
        r00 * sx, r10 * sx, r20 * sx, 0.0f);
    local_matrix.column1 = float4(
        r01 * sy, r11 * sy, r21 * sy, 0.0f);
    local_matrix.column2 = float4(
        r02 * sz, r12 * sz, r22 * sz, 0.0f);
    local_matrix.column3 = float4(
        local.translation.xyz, 1.0f);

    local_derivative.column0 = float4(
        dr00 * sx + r00 * sxd,
        dr10 * sx + r10 * sxd,
        dr20 * sx + r20 * sxd,
        0.0f);
    local_derivative.column1 = float4(
        dr01 * sy + r01 * syd,
        dr11 * sy + r11 * syd,
        dr21 * sy + r21 * syd,
        0.0f);
    local_derivative.column2 = float4(
        dr02 * sz + r02 * szd,
        dr12 * sz + r12 * szd,
        dr22 * sz + r22 * szd,
        0.0f);
    local_derivative.column3 = float4(
        local.translation_velocity.xyz, 0.0f);

    return finite_matrix(local_matrix) &&
        finite_matrix(local_derivative);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint local_index = dispatch_thread_id.x;
    if (local_index >= node_count)
    {
        return;
    }

    const animation_node_local_transform_gpu_record local =
        local_transforms[local_index];
    if (local.status != 0u ||
        local.node_index == INVALID_INDEX ||
        local.node_index >= static_node_matrix_count)
    {
        fail_node(local_index);
        return;
    }

    uint current_index = local_index;
    uint depth = 0u;
    bool found_root = false;
    [loop]
    for (uint ancestor_step = 0u; ancestor_step < node_count; ++ancestor_step)
    {
        const animation_node_local_transform_gpu_record ancestor =
            local_transforms[current_index];
        if (ancestor.status != 0u)
        {
            fail_node(local_index);
            return;
        }
        if (ancestor.parent_index == INVALID_INDEX)
        {
            found_root = true;
            break;
        }
        if (ancestor.parent_index >= current_index ||
            ancestor.parent_index >= node_count)
        {
            fail_node(local_index);
            return;
        }
        current_index = ancestor.parent_index;
        ++depth;
    }

    if (!found_root)
    {
        fail_node(local_index);
        return;
    }
    if (depth != hierarchy_level)
    {
        return;
    }

    gpu_matrix4x4 local_matrix;
    gpu_matrix4x4 local_derivative;
    if ((local.node_role_flags & NODE_ROLE_STATIC_MATRIX) != 0u)
    {
        local_matrix = static_node_matrices[local.node_index];
        local_derivative = (gpu_matrix4x4)0;
        if (!finite_matrix(local_matrix))
        {
            fail_node(local_index);
            return;
        }
    }
    else if (!build_local_trs_matrices(
            local, local_matrix, local_derivative))
    {
        fail_node(local_index);
        return;
    }

    gpu_matrix4x4 global_matrix = local_matrix;
    gpu_matrix4x4 global_derivative = local_derivative;
    if (local.parent_index != INVALID_INDEX)
    {
        const gpu_matrix4x4 parent_global =
            global_matrices[local.parent_index];
        const gpu_matrix4x4 parent_derivative =
            global_matrix_derivatives[local.parent_index];
        if (!finite_matrix(parent_global) ||
            !finite_matrix(parent_derivative))
        {
            fail_node(local_index);
            return;
        }
        global_matrix = multiply_matrix(parent_global, local_matrix);
        global_derivative = add_matrix(
            multiply_matrix(parent_derivative, local_matrix),
            multiply_matrix(parent_global, local_derivative));
    }

    if (!finite_matrix(global_matrix) ||
        !finite_matrix(global_derivative))
    {
        fail_node(local_index);
        return;
    }

    global_matrices[local_index] = global_matrix;
    global_matrix_derivatives[local_index] = global_derivative;
}
