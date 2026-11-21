cbuffer ClothPredictParameters : register(b0)
{
    uint vertex_count;
    float delta_time;
    float3 gravity;
    uint reserved0;
    uint reserved1;
    uint reserved2;
};

StructuredBuffer<float4> current_positions : register(t0);
StructuredBuffer<float4> current_velocities : register(t1);
StructuredBuffer<float> inverse_masses : register(t2);
RWStructuredBuffer<float4> predicted_positions : register(u0);
RWStructuredBuffer<float4> predicted_velocities : register(u1);
RWStructuredBuffer<uint> solver_status : register(u2);

static const uint STATUS_INVALID_CLOTH_PREDICTION = 1u << 3u;

void fail_prediction()
{
    uint ignored = 0u;
    InterlockedOr(solver_status[0], STATUS_INVALID_CLOTH_PREDICTION, ignored);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint vertex_index = dispatch_thread_id.x;
    if (vertex_index >= vertex_count)
    {
        return;
    }

    uint position_count = 0u;
    uint position_stride = 0u;
    uint velocity_count = 0u;
    uint velocity_stride = 0u;
    uint mass_count = 0u;
    uint mass_stride = 0u;
    uint output_position_count = 0u;
    uint output_position_stride = 0u;
    uint output_velocity_count = 0u;
    uint output_velocity_stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    current_positions.GetDimensions(position_count, position_stride);
    current_velocities.GetDimensions(velocity_count, velocity_stride);
    inverse_masses.GetDimensions(mass_count, mass_stride);
    predicted_positions.GetDimensions(output_position_count, output_position_stride);
    predicted_velocities.GetDimensions(output_velocity_count, output_velocity_stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (status_count != 1u || status_stride != 4u)
    {
        return;
    }
    if (reserved0 != 0u || reserved1 != 0u || reserved2 != 0u ||
        vertex_count == 0u || vertex_count != position_count ||
        vertex_count != velocity_count || vertex_count != mass_count ||
        vertex_count != output_position_count ||
        vertex_count != output_velocity_count ||
        position_stride != 16u || velocity_stride != 16u ||
        output_position_stride != 16u || output_velocity_stride != 16u ||
        mass_stride != 4u || !isfinite(delta_time) || delta_time <= 0.0f ||
        !all(isfinite(gravity)))
    {
        fail_prediction();
        return;
    }

    const float4 position = current_positions[vertex_index];
    const float4 velocity = current_velocities[vertex_index];
    const float inverse_mass = inverse_masses[vertex_index];
    if (!all(isfinite(position)) || !all(isfinite(velocity)) ||
        !isfinite(inverse_mass) || inverse_mass < 0.0f)
    {
        fail_prediction();
        predicted_positions[vertex_index] = position;
        predicted_velocities[vertex_index] = velocity;
        return;
    }

    if (inverse_mass == 0.0f)
    {
        predicted_positions[vertex_index] = position;
        predicted_velocities[vertex_index] = 0.0f;
        return;
    }

    const float3 next_velocity = velocity.xyz + gravity * delta_time;
    const float3 next_position =
        position.xyz + velocity.xyz * delta_time +
        gravity * (delta_time * delta_time);
    if (!all(isfinite(next_velocity)) || !all(isfinite(next_position)))
    {
        fail_prediction();
        predicted_positions[vertex_index] = position;
        predicted_velocities[vertex_index] = velocity;
        return;
    }

    predicted_positions[vertex_index] =
        float4(next_position, position.w);
    predicted_velocities[vertex_index] =
        float4(next_velocity, velocity.w);
}
