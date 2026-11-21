struct ClothEpochPrimitive
{
    uint4 vertices;
};

struct ClothEpochAabb
{
    float4 minimum;
    float4 maximum;
    uint4 identity;
};

cbuffer ClothEpochAabbParameters : register(b0)
{
    uint vertex_count;
    uint primitive_count;
    uint primitive_vertex_count;
    float contact_thickness;
};

StructuredBuffer<float4> epoch_before_positions : register(t0);
StructuredBuffer<float4> epoch_after_positions : register(t1);
StructuredBuffer<ClothEpochPrimitive> primitives : register(t2);
RWStructuredBuffer<ClothEpochAabb> swept_aabbs : register(u0);
RWStructuredBuffer<uint> solver_status : register(u1);

static const uint STATUS_INVALID_CLOTH_EPOCH_AABB = 1u << 8u;

void fail_epoch_aabb()
{
    uint ignored = 0u;
    InterlockedOr(solver_status[0], STATUS_INVALID_CLOTH_EPOCH_AABB, ignored);
}

float outward_lower(float value)
{
    if (value == 0.0f)
    {
        return asfloat(0x80000001u);
    }
    const uint bits = asuint(value);
    return asfloat((bits & 0x80000000u) != 0u ? bits + 1u : bits - 1u);
}

float outward_upper(float value)
{
    if (value == 0.0f)
    {
        return asfloat(0x00000001u);
    }
    const uint bits = asuint(value);
    return asfloat((bits & 0x80000000u) != 0u ? bits - 1u : bits + 1u);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint primitive_index = dispatch_thread_id.x;
    if (primitive_index >= primitive_count)
    {
        return;
    }

    uint before_count = 0u;
    uint before_stride = 0u;
    uint after_count = 0u;
    uint after_stride = 0u;
    uint primitive_buffer_count = 0u;
    uint primitive_stride = 0u;
    uint output_count = 0u;
    uint output_stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    epoch_before_positions.GetDimensions(before_count, before_stride);
    epoch_after_positions.GetDimensions(after_count, after_stride);
    primitives.GetDimensions(primitive_buffer_count, primitive_stride);
    swept_aabbs.GetDimensions(output_count, output_stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (status_count != 1u || status_stride != 4u)
    {
        return;
    }
    if (vertex_count == 0u || vertex_count != before_count ||
        vertex_count != after_count || vertex_count != 0u &&
        (before_stride != 16u || after_stride != 16u) ||
        primitive_count == 0u || primitive_count > primitive_buffer_count ||
        primitive_count > output_count || primitive_stride != 16u ||
        output_stride != 48u ||
        (primitive_vertex_count != 2u && primitive_vertex_count != 3u) ||
        !isfinite(contact_thickness) || contact_thickness < 0.0f)
    {
        fail_epoch_aabb();
        return;
    }

    const uint4 indices = primitives[primitive_index].vertices;
    float3 minimum_value = 3.402823466e+38f;
    float3 maximum_value = -3.402823466e+38f;
    [unroll]
    for (uint local_index = 0u; local_index < 3u; ++local_index)
    {
        if (local_index >= primitive_vertex_count)
        {
            continue;
        }
        const uint vertex_index = indices[local_index];
        if (vertex_index >= vertex_count)
        {
            fail_epoch_aabb();
            return;
        }
        const float4 before_position = epoch_before_positions[vertex_index];
        const float4 after_position = epoch_after_positions[vertex_index];
        if (!all(isfinite(before_position)) || !all(isfinite(after_position)))
        {
            fail_epoch_aabb();
            return;
        }

        minimum_value = min(minimum_value,
            min(before_position.xyz, after_position.xyz));
        maximum_value = max(maximum_value,
            max(before_position.xyz, after_position.xyz));
    }

    const float3 expanded_minimum = minimum_value - contact_thickness;
    const float3 expanded_maximum = maximum_value + contact_thickness;
    if (!all(isfinite(expanded_minimum)) || !all(isfinite(expanded_maximum)))
    {
        fail_epoch_aabb();
        return;
    }

    ClothEpochAabb output_value;
    output_value.minimum = float4(
        outward_lower(expanded_minimum.x),
        outward_lower(expanded_minimum.y),
        outward_lower(expanded_minimum.z), 0.0f);
    output_value.maximum = float4(
        outward_upper(expanded_maximum.x),
        outward_upper(expanded_maximum.y),
        outward_upper(expanded_maximum.z), 0.0f);
    output_value.identity = uint4(
        primitive_index, primitive_vertex_count, 0u, 0u);
    if (!all(isfinite(output_value.minimum)) ||
        !all(isfinite(output_value.maximum)))
    {
        fail_epoch_aabb();
        return;
    }
    swept_aabbs[primitive_index] = output_value;
}
