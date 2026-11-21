cbuffer ClothEpochDisplacementParameters : register(b0)
{
    uint vertex_count;
    uint reserved0;
    uint reserved1;
    uint reserved2;
};

StructuredBuffer<float4> epoch_before_positions : register(t0);
StructuredBuffer<float4> epoch_after_positions : register(t1);
RWStructuredBuffer<float4> epoch_displacements : register(u0);
RWStructuredBuffer<uint> solver_status : register(u1);

static const uint STATUS_INVALID_CLOTH_EPOCH_DISPLACEMENT = 1u << 7u;

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint vertex_index = dispatch_thread_id.x;
    if (vertex_index >= vertex_count)
    {
        return;
    }

    uint before_count = 0u;
    uint before_stride = 0u;
    uint after_count = 0u;
    uint after_stride = 0u;
    uint output_count = 0u;
    uint output_stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    epoch_before_positions.GetDimensions(before_count, before_stride);
    epoch_after_positions.GetDimensions(after_count, after_stride);
    epoch_displacements.GetDimensions(output_count, output_stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (status_count != 1u || status_stride != 4u)
    {
        return;
    }
    if (reserved0 != 0u || reserved1 != 0u || reserved2 != 0u ||
        vertex_count == 0u || vertex_count != before_count ||
        vertex_count != after_count || vertex_count != output_count ||
        before_stride != 16u || after_stride != 16u ||
        output_stride != 16u)
    {
        uint ignored = 0u;
        InterlockedOr(
            solver_status[0], STATUS_INVALID_CLOTH_EPOCH_DISPLACEMENT, ignored);
        return;
    }

    const float4 before_position = epoch_before_positions[vertex_index];
    const float4 after_position = epoch_after_positions[vertex_index];
    if (!all(isfinite(before_position)) || !all(isfinite(after_position)))
    {
        uint ignored = 0u;
        InterlockedOr(
            solver_status[0], STATUS_INVALID_CLOTH_EPOCH_DISPLACEMENT, ignored);
        epoch_displacements[vertex_index] = 0.0f;
        return;
    }

    const float3 displacement =
        after_position.xyz - before_position.xyz;
    if (!all(isfinite(displacement)))
    {
        uint ignored = 0u;
        InterlockedOr(
            solver_status[0], STATUS_INVALID_CLOTH_EPOCH_DISPLACEMENT, ignored);
        epoch_displacements[vertex_index] = 0.0f;
        return;
    }

    epoch_displacements[vertex_index] = float4(displacement, 0.0f);
}
