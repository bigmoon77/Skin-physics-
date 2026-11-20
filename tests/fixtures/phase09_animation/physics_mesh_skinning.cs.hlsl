struct gpu_float4
{
    float x;
    float y;
    float z;
    float w;
};

struct gpu_skin_weight_record
{
    uint4 joints0;
    uint4 joints1;
    float4 weights0;
    float4 weights1;
};

struct gpu_matrix4x4
{
    float4 column0;
    float4 column1;
    float4 column2;
    float4 column3;
};

cbuffer SkinningParameters : register(b0)
{
    uint vertex_count;
    uint joint_count;
    uint2 reserved;
};

StructuredBuffer<gpu_float4> rest_positions : register(t0);
StructuredBuffer<gpu_skin_weight_record> skin_weights : register(t1);
StructuredBuffer<gpu_matrix4x4> skin_matrices : register(t2);
StructuredBuffer<gpu_matrix4x4> skin_matrix_derivatives : register(t3);

RWStructuredBuffer<gpu_float4> target_positions : register(u0);
RWStructuredBuffer<gpu_float4> target_velocities : register(u1);
RWStructuredBuffer<uint> target_status : register(u2);

bool finite4(float4 value)
{
    return all(isfinite(value));
}

float4 transform_column_major(gpu_matrix4x4 matrix_record, float4 value)
{
    return matrix_record.column0 * value.x +
        matrix_record.column1 * value.y +
        matrix_record.column2 * value.z +
        matrix_record.column3 * value.w;
}

void fail_vertex(uint vertex_index)
{
    target_positions[vertex_index] = (gpu_float4)0;
    target_velocities[vertex_index] = (gpu_float4)0;
    uint previous_status = 0u;
    InterlockedOr(target_status[0], 1u, previous_status);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint vertex_index = dispatch_thread_id.x;
    if (vertex_index >= vertex_count)
    {
        return;
    }

    if (joint_count == 0u)
    {
        fail_vertex(vertex_index);
        return;
    }

    const gpu_float4 rest_value = rest_positions[vertex_index];
    const float4 rest_position = float4(
        rest_value.x, rest_value.y, rest_value.z, rest_value.w);
    if (!finite4(rest_position))
    {
        fail_vertex(vertex_index);
        return;
    }

    const gpu_skin_weight_record influences = skin_weights[vertex_index];
    float3 accumulated_position = 0.0f;
    float3 accumulated_velocity = 0.0f;
    float accumulated_weight = 0.0f;

    [unroll]
    for (uint influence_index = 0u; influence_index < 8u; ++influence_index)
    {
        uint joint_index = 0u;
        float weight = 0.0f;
        if (influence_index < 4u)
        {
            joint_index = influences.joints0[influence_index];
            weight = influences.weights0[influence_index];
        }
        else
        {
            joint_index = influences.joints1[influence_index - 4u];
            weight = influences.weights1[influence_index - 4u];
        }

        if (!isfinite(weight) || weight < 0.0f)
        {
            fail_vertex(vertex_index);
            return;
        }
        if (weight == 0.0f)
        {
            continue;
        }
        if (joint_index >= joint_count)
        {
            fail_vertex(vertex_index);
            return;
        }

        const gpu_matrix4x4 skin_matrix = skin_matrices[joint_index];
        const gpu_matrix4x4 skin_derivative =
            skin_matrix_derivatives[joint_index];
        if (!finite4(skin_matrix.column0) ||
            !finite4(skin_matrix.column1) ||
            !finite4(skin_matrix.column2) ||
            !finite4(skin_matrix.column3) ||
            !finite4(skin_derivative.column0) ||
            !finite4(skin_derivative.column1) ||
            !finite4(skin_derivative.column2) ||
            !finite4(skin_derivative.column3))
        {
            fail_vertex(vertex_index);
            return;
        }

        const float4 transformed_position =
            transform_column_major(skin_matrix, rest_position);
        const float4 transformed_velocity =
            transform_column_major(skin_derivative, rest_position);
        if (!finite4(transformed_position) ||
            !finite4(transformed_velocity))
        {
            fail_vertex(vertex_index);
            return;
        }

        accumulated_position += transformed_position.xyz * weight;
        accumulated_velocity += transformed_velocity.xyz * weight;
        accumulated_weight += weight;
    }

    if (!isfinite(accumulated_weight) ||
        abs(accumulated_weight - 1.0f) > 1.0e-3f ||
        !all(isfinite(accumulated_position)) ||
        !all(isfinite(accumulated_velocity)))
    {
        fail_vertex(vertex_index);
        return;
    }

    const float inverse_weight_sum = rcp(accumulated_weight);
    accumulated_position *= inverse_weight_sum;
    accumulated_velocity *= inverse_weight_sum;
    if (!all(isfinite(accumulated_position)) ||
        !all(isfinite(accumulated_velocity)))
    {
        fail_vertex(vertex_index);
        return;
    }

    target_positions[vertex_index].x = accumulated_position.x;
    target_positions[vertex_index].y = accumulated_position.y;
    target_positions[vertex_index].z = accumulated_position.z;
    target_positions[vertex_index].w = 1.0f;
    target_velocities[vertex_index].x = accumulated_velocity.x;
    target_velocities[vertex_index].y = accumulated_velocity.y;
    target_velocities[vertex_index].z = accumulated_velocity.z;
    target_velocities[vertex_index].w = 0.0f;
}
