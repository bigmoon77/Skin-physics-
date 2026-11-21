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

StructuredBuffer<float4> segment_positions : register(t0);
StructuredBuffer<float> rest_lumped_masses : register(t1);
StructuredBuffer<float> reduction_weights : register(t2);
StructuredBuffer<float> apply_weights : register(t3);
StructuredBuffer<float> apply_weight_sum : register(t4);
StructuredBuffer<float4> previous_accumulator : register(t5);

RWStructuredBuffer<float> raw_distribution_weights : register(u0);
RWStructuredBuffer<TickCommitControl> tick_commit_control_buffer : register(u1);
RWStructuredBuffer<float> partial_sums : register(u2);
RWStructuredBuffer<float> normalized_weight_sum : register(u3);
RWStructuredBuffer<float4> next_accumulator : register(u4);

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

cbuffer EventParameters : register(b0)
{
    uint vertex_count;
    uint include_previous;
    uint2 reserved0;
    float3 action_position;
    float radius;
    float3 direction;
    float magnitude;
};

cbuffer ReductionParameters : register(b1)
{
    uint reduction_vertex_count;
    uint group_count;
    uint partial_sum_offset;
    uint reduction_reserved;
};

static const uint TICK_GATE_READY = 1u;
static const uint TICK_TERMINAL_IN_FLIGHT = 1u;
static const float MINIMUM_WEIGHT_SUM = 1.0e-20f;

bool tick_distribution_is_active(TickCommitControl control)
{
    return control.last_gate_state == TICK_GATE_READY &&
        control.terminal_state == TICK_TERMINAL_IN_FLIGHT &&
        control.terminal_failure == 0u;
}

[numthreads(64, 1, 1)]
void soft_body_force_distribution_weight(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint index = dispatch_id.x;
    if (index >= vertex_count)
    {
        return;
    }

    TickCommitControl control = tick_commit_control_buffer[0];
    if (!tick_distribution_is_active(control))
    {
        return;
    }

    float4 position_value = segment_positions[index];
    float rest_mass = rest_lumped_masses[index];
    if (!all(isfinite(position_value.xyz)) ||
        !isfinite(rest_mass) ||
        rest_mass < 0.0f)
    {
        mark_tick_failure(301u);
        return;
    }

    if (rest_mass == 0.0f)
    {
        raw_distribution_weights[index] = 0.0f;
        return;
    }

    float3 offset = position_value.xyz - action_position;
    float distance_squared = dot(offset, offset);
    float radius_squared = radius * radius;
    if (!isfinite(distance_squared) ||
        !isfinite(radius_squared) ||
        radius_squared <= 0.0f)
    {
        mark_tick_failure(302u);
        return;
    }

    if (distance_squared >= radius_squared)
    {
        raw_distribution_weights[index] = 0.0f;
        return;
    }

    float normalized_distance = sqrt(distance_squared) / radius;
    float one_minus_s = 1.0f - normalized_distance;
    float one_minus_s_squared = one_minus_s * one_minus_s;
    float one_minus_s_fourth = one_minus_s_squared * one_minus_s_squared;
    float phi = one_minus_s_fourth * (4.0f * normalized_distance + 1.0f);
    float q = rest_mass * phi;
    if (!isfinite(q) || q < 0.0f)
    {
        mark_tick_failure(303u);
        return;
    }

    raw_distribution_weights[index] = q;
}

groupshared float group_weight_values[64];

[numthreads(64, 1, 1)]
void soft_body_force_distribution_reduce(
    uint3 group_id : SV_GroupID,
    uint3 group_thread_id : SV_GroupThreadID)
{
    TickCommitControl control = tick_commit_control_buffer[0];
    const bool group_active =
        tick_distribution_is_active(control) &&
        group_id.x < group_count;

    uint vertex_index = group_id.x * 64u + group_thread_id.x;
    float value = 0.0f;
    if (group_active && vertex_index < reduction_vertex_count)
    {
        value = reduction_weights[vertex_index];
        if (!isfinite(value) || value < 0.0f)
        {
            mark_tick_failure(304u);
        value = 0.0f;
        }
    }

    // Every thread in a dispatched group must participate in every group
    // barrier. Runtime activity and tail bounds only gate memory operations;
    // they must never gate the synchronization sequence.
    group_weight_values[group_thread_id.x] = value;
    GroupMemoryBarrierWithGroupSync();

    for (uint stride = 32u; stride > 0u; stride >>= 1u)
    {
        if (group_thread_id.x < stride)
        {
            group_weight_values[group_thread_id.x] +=
                group_weight_values[group_thread_id.x + stride];
        }
        GroupMemoryBarrierWithGroupSync();
    }

    if (group_thread_id.x == 0u && group_active)
    {
        partial_sums[partial_sum_offset + group_id.x] =
            group_weight_values[0];
    }
}

[numthreads(1, 1, 1)]
void soft_body_force_distribution_finalize(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u)
    {
        return;
    }

    TickCommitControl control = tick_commit_control_buffer[0];
    if (!tick_distribution_is_active(control))
    {
        return;
    }

    float sum = 0.0f;
    for (uint group_index = 0u; group_index < group_count; ++group_index)
    {
        float partial = reduction_weights[group_index];
        if (!isfinite(partial) || partial < 0.0f)
        {
            mark_tick_failure(305u);
            normalized_weight_sum[0] = 0.0f;
            return;
        }
        sum += partial;
    }

    if (!isfinite(sum) || sum <= MINIMUM_WEIGHT_SUM)
    {
        control.terminal_failure = 1u;
        control.diagnostic = 7u;
        tick_commit_control_buffer[0] = control;
        normalized_weight_sum[0] = 0.0f;
        return;
    }

    normalized_weight_sum[0] = sum;
}

[numthreads(64, 1, 1)]
void soft_body_force_distribution_apply(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint index = dispatch_id.x;
    if (index >= vertex_count)
    {
        return;
    }

    TickCommitControl control = tick_commit_control_buffer[0];
    if (!tick_distribution_is_active(control))
    {
        return;
    }

    float sum = apply_weight_sum[0];
    float q = apply_weights[index];
    if (!isfinite(sum) || sum <= MINIMUM_WEIGHT_SUM ||
        !isfinite(q) || q < 0.0f)
    {
        mark_tick_failure(306u);
        return;
    }

    float4 previous_value = include_previous != 0u
        ? previous_accumulator[index]
        : float4(0.0f, 0.0f, 0.0f, 0.0f);
    float weight = q / sum;
    float3 next_value =
        previous_value.xyz + weight * direction * magnitude;
    if (!all(isfinite(next_value)))
    {
        mark_tick_failure(307u);
        return;
    }

    next_accumulator[index] = float4(next_value, 0.0f);
}
