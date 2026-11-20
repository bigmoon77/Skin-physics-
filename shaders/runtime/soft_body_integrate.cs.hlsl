struct TickCommitControl
{
    uint committed_sequence_low;
    uint committed_sequence_high;
    uint state_generation_low;
    uint state_generation_high;
    uint active_state_set;
    uint terminal_state;
    uint terminal_failure;
    uint validation_mask;
    uint required_validation_mask;
    uint dispatch_id_low;
    uint dispatch_id_high;
    uint reservation_token_low;
    uint reservation_token_high;
    uint last_gate_state;
    uint diagnostic;
    uint render_slot_id;
    uint render_slot_generation;
    uint reserved0;
    uint reserved1;
    uint reserved2;
};

StructuredBuffer<float4> source_positions : register(t0);
StructuredBuffer<float4> source_velocities : register(t1);
StructuredBuffer<float4> vertex_forces : register(t2);
StructuredBuffer<float> inverse_masses : register(t3);
StructuredBuffer<float> vertex_damping_rates : register(t4);
StructuredBuffer<float4> vertex_impulses : register(t5);
RWStructuredBuffer<float4> destination_positions : register(u0);
RWStructuredBuffer<float4> destination_velocities : register(u1);
RWStructuredBuffer<TickCommitControl> tick_commit_control_buffer : register(u2);

cbuffer IntegrationParameters : register(b0)
{
    uint vertex_count;
    float delta_time;
    float damping_rate;
    uint has_impulse_events;
    float3 gravity;
    float reserved1;
};

static const uint TICK_GATE_READY = 1u;
static const uint TICK_TERMINAL_IN_FLIGHT = 1u;

[numthreads(64, 1, 1)]
void soft_body_integrate(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint index = dispatch_id.x;
    if (index >= vertex_count)
    {
        return;
    }

    TickCommitControl control = tick_commit_control_buffer[0];
    if (control.last_gate_state != TICK_GATE_READY ||
        control.terminal_state != TICK_TERMINAL_IN_FLIGHT ||
        control.terminal_failure != 0u)
    {
        return;
    }

    float4 position_value = source_positions[index];
    float4 velocity_value = source_velocities[index];
    float inverse_mass = inverse_masses[index];
    if (inverse_mass <= 0.0f)
    {
        destination_positions[index] = position_value;
        destination_velocities[index] = velocity_value;
        return;
    }

    float vertex_damping_rate = vertex_damping_rates[index];
    if (!isfinite(vertex_damping_rate) || vertex_damping_rate < 0.0f)
    {
        uint previous_failure = 0u;
        InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
        return;
    }

    float z = vertex_damping_rate * delta_time;
    float decay = exp(-z);
    float damping_integral = delta_time;
    if (z >= 1.0e-3f && vertex_damping_rate > 0.0f)
    {
        damping_integral = (1.0f - decay) / vertex_damping_rate;
    }
    else if (z > 0.0f)
    {
        damping_integral = delta_time *
            (1.0f - 0.5f * z + (z * z) / 6.0f - (z * z * z) / 24.0f);
    }

    float3 impulse = has_impulse_events != 0u
        ? vertex_impulses[index].xyz
        : 0.0f;
    float3 velocity_after_impulse =
        velocity_value.xyz + inverse_mass * impulse;
    float3 acceleration = inverse_mass * vertex_forces[index].xyz + gravity;
    float3 next_velocity =
        decay * velocity_after_impulse + damping_integral * acceleration;
    float3 next_position = position_value.xyz + delta_time * next_velocity;

    if (!all(isfinite(next_velocity)) || !all(isfinite(next_position)))
    {
        uint previous_failure = 0u;
        InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
        return;
    }

    destination_positions[index] = float4(next_position, position_value.w);
    destination_velocities[index] = float4(next_velocity, velocity_value.w);
}

[numthreads(64, 1, 1)]
void soft_body_copy_state(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint index = dispatch_id.x;
    if (index >= vertex_count)
    {
        return;
    }

    TickCommitControl control = tick_commit_control_buffer[0];
    if (control.last_gate_state != TICK_GATE_READY ||
        control.terminal_state != TICK_TERMINAL_IN_FLIGHT ||
        control.terminal_failure != 0u)
    {
        return;
    }

    destination_positions[index] = source_positions[index];
    destination_velocities[index] = source_velocities[index];
}
