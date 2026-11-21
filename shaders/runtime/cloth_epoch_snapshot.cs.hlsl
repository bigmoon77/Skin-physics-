cbuffer ClothEpochSnapshotParameters : register(b0)
{
    uint vertex_count;
    uint reserved0;
    uint reserved1;
    uint reserved2;
};

StructuredBuffer<float4> source_positions : register(t0);
RWStructuredBuffer<float4> epoch_before_positions : register(u0);
RWStructuredBuffer<uint> solver_status : register(u1);

static const uint STATUS_INVALID_CLOTH_EPOCH_SNAPSHOT = 1u << 6u;

void fail_epoch_snapshot()
{
    uint ignored = 0u;
    InterlockedOr(
        solver_status[0], STATUS_INVALID_CLOTH_EPOCH_SNAPSHOT, ignored);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint vertex_index = dispatch_thread_id.x;
    uint source_count = 0u;
    uint source_stride = 0u;
    uint output_count = 0u;
    uint output_stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    source_positions.GetDimensions(source_count, source_stride);
    epoch_before_positions.GetDimensions(output_count, output_stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (status_count != 1u || status_stride != 4u)
    {
        return;
    }
    if (reserved0 != 0u || reserved1 != 0u || reserved2 != 0u ||
        vertex_count == 0u || vertex_count != source_count ||
        vertex_count != output_count ||
        source_stride != 16u || output_stride != 16u)
    {
        fail_epoch_snapshot();
        return;
    }
    if (vertex_index >= vertex_count)
    {
        return;
    }

    const float4 source_position = source_positions[vertex_index];
    if (!all(isfinite(source_position)))
    {
        fail_epoch_snapshot();
    }
    epoch_before_positions[vertex_index] = source_position;
}
