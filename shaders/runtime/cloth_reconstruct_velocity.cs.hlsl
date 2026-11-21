cbuffer ClothVelocityReconstructParameters : register(b0)
{
    uint vertex_count;
    float delta_time;
    uint reserved0;
    uint reserved1;
};

StructuredBuffer<float4> segment_start_positions : register(t0);
StructuredBuffer<float4> solved_positions : register(t1);
StructuredBuffer<float> inverse_masses : register(t2);
RWStructuredBuffer<float4> reconstructed_velocities : register(u0);
RWStructuredBuffer<uint> solver_status : register(u1);

static const uint STATUS_INVALID_CLOTH_VELOCITY = 1u << 4u;

void fail_velocity()
{
    uint ignored = 0u;
    InterlockedOr(solver_status[0], STATUS_INVALID_CLOTH_VELOCITY, ignored);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint vertex_index = dispatch_thread_id.x;
    if (vertex_index >= vertex_count)
    {
        return;
    }

    uint start_count = 0u;
    uint start_stride = 0u;
    uint solved_count = 0u;
    uint solved_stride = 0u;
    uint mass_count = 0u;
    uint mass_stride = 0u;
    uint velocity_count = 0u;
    uint velocity_stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    segment_start_positions.GetDimensions(start_count, start_stride);
    solved_positions.GetDimensions(solved_count, solved_stride);
    inverse_masses.GetDimensions(mass_count, mass_stride);
    reconstructed_velocities.GetDimensions(velocity_count, velocity_stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (status_count != 1u || status_stride != 4u)
    {
        return;
    }
    if (reserved0 != 0u || reserved1 != 0u ||
        vertex_count == 0u || vertex_count != start_count ||
        vertex_count != solved_count || vertex_count != mass_count ||
        vertex_count != velocity_count || start_stride != 16u ||
        solved_stride != 16u || velocity_stride != 16u ||
        mass_stride != 4u || !isfinite(delta_time) || delta_time <= 0.0f)
    {
        fail_velocity();
        return;
    }

    const float4 start_position = segment_start_positions[vertex_index];
    const float4 solved_position = solved_positions[vertex_index];
    const float inverse_mass = inverse_masses[vertex_index];
    if (!all(isfinite(start_position)) ||
        !all(isfinite(solved_position)) ||
        !isfinite(inverse_mass) || inverse_mass < 0.0f)
    {
        fail_velocity();
        reconstructed_velocities[vertex_index] = 0.0f;
        return;
    }

    if (inverse_mass == 0.0f)
    {
        reconstructed_velocities[vertex_index] = 0.0f;
        return;
    }

    const float3 velocity =
        (solved_position.xyz - start_position.xyz) / delta_time;
    if (!all(isfinite(velocity)))
    {
        fail_velocity();
        reconstructed_velocities[vertex_index] = 0.0f;
        return;
    }

    reconstructed_velocities[vertex_index] =
        float4(velocity, 0.0f);
}
