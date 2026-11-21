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
StructuredBuffer<float4> animation_target_positions : register(t6);
StructuredBuffer<float4> animation_target_velocities : register(t7);
StructuredBuffer<uint> animation_target_status : register(t8);
StructuredBuffer<float> animation_tracking_weights : register(t9);
RWStructuredBuffer<float4> destination_positions : register(u0);
RWStructuredBuffer<float4> destination_velocities : register(u1);
RWStructuredBuffer<TickCommitControl> tick_commit_control_buffer : register(u2);

void mark_tick_failure(uint diagnostic_code)
{
    uint previous_diagnostic = 0u;
    InterlockedCompareExchange(
        tick_commit_control_buffer[0].diagnostic,
        0u,
        diagnostic_code,
        previous_diagnostic);
    uint previous_failure = 0u;
    InterlockedOr(
        tick_commit_control_buffer[0].terminal_failure,
        1u,
        previous_failure);
}

cbuffer IntegrationParameters : register(b0)
{
    uint vertex_count;
    float delta_time;
    float damping_rate;
    uint has_impulse_events;
    float3 gravity;
    float tracking_stiffness;
    float tracking_damping;
    float tracking_attribute_multiplier;
    uint has_animation_target;
    uint reserved2;
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
        mark_tick_failure(201u);
        return;
    }

    float3 impulse = has_impulse_events != 0u
        ? vertex_impulses[index].xyz
        : 0.0f;
    float3 velocity_after_impulse =
        velocity_value.xyz + inverse_mass * impulse;
    float3 acceleration =
        inverse_mass * vertex_forces[index].xyz + gravity;
    float effective_damping_rate = vertex_damping_rate;

    if (has_animation_target != 0u)
    {
        if (animation_target_status[0] != 0u)
        {
            mark_tick_failure(203u + animation_target_status[0]);
            return;
        }

        float tracking_weight = animation_tracking_weights[index];
        float4 target_position_value = animation_target_positions[index];
        float4 target_velocity_value = animation_target_velocities[index];
        if (!isfinite(tracking_weight) ||
            tracking_weight < 0.0f || tracking_weight > 1.0f ||
            !all(isfinite(target_position_value)) ||
            !all(isfinite(target_velocity_value)) ||
            !isfinite(tracking_stiffness) || tracking_stiffness < 0.0f ||
            !isfinite(tracking_damping) || tracking_damping < 0.0f ||
            !isfinite(tracking_attribute_multiplier) ||
            tracking_attribute_multiplier < 0.0f)
        {
            mark_tick_failure(204u);
            return;
        }

        float effective_weight =
            tracking_weight * tracking_attribute_multiplier;
        float effective_stiffness =
            tracking_stiffness * effective_weight;
        float effective_tracking_damping =
            tracking_damping * effective_weight;
        float3 tracking_spring_force =
            effective_stiffness *
            (target_position_value.xyz - position_value.xyz);
        float3 tracking_target_velocity_force =
            effective_tracking_damping * target_velocity_value.xyz;
        effective_damping_rate +=
            inverse_mass * effective_tracking_damping;
        acceleration += inverse_mass *
            (tracking_spring_force + tracking_target_velocity_force);
        if (!isfinite(effective_damping_rate) ||
            effective_damping_rate < 0.0f ||
            !all(isfinite(acceleration)))
        {
            mark_tick_failure(205u);
            return;
        }
    }

    // Treat the Kelvin-Voigt velocity term as an additional linear damping
    // coefficient. The exponential update remains stable for large c / m.
    float z = effective_damping_rate * delta_time;
    float decay = exp(-z);
    float damping_integral = delta_time;
    if (z >= 1.0e-3f && effective_damping_rate > 0.0f)
    {
        damping_integral = (1.0f - decay) / effective_damping_rate;
    }
    else if (z > 0.0f)
    {
        damping_integral = delta_time *
            (1.0f - 0.5f * z + (z * z) / 6.0f - (z * z * z) / 24.0f);
    }

    float3 next_velocity =
        decay * velocity_after_impulse + damping_integral * acceleration;
    float3 next_position = position_value.xyz + delta_time * next_velocity;

    if (!all(isfinite(next_velocity)) || !all(isfinite(next_position)))
    {
        mark_tick_failure(202u);
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
