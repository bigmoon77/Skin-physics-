struct TickDispatchPlan
{
    uint expected_predecessor_sequence_low;
    uint expected_predecessor_sequence_high;
    uint expected_source_generation_low;
    uint expected_source_generation_high;
    uint expected_source_state_set;
    uint destination_state_set;
    uint tick_sequence_low;
    uint tick_sequence_high;
    uint dispatch_id_low;
    uint dispatch_id_high;
    uint reservation_token_low;
    uint reservation_token_high;
    uint required_validation_mask;
    uint flags;
    uint reserved0;
    uint reserved1;
};

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
    uint reserved;
};

struct TickStatus
{
    uint terminal_state;
    uint terminal_failure;
    uint committed_sequence_low;
    uint committed_sequence_high;
    uint state_generation_low;
    uint state_generation_high;
    uint active_state_set;
    uint diagnostic_code;
    uint dispatch_id_low;
    uint dispatch_id_high;
    uint reservation_token_low;
    uint reservation_token_high;
    uint validation_mask;
    uint required_validation_mask;
    uint work_budget_block_flags;
    uint work_budget_block_origin;
    uint executed_integration_segments;
    uint minimum_required_integration_segments;
    uint reserved0;
    uint reserved1;
};

StructuredBuffer<TickDispatchPlan> tick_dispatch_plan : register(t0);
RWStructuredBuffer<TickCommitControl> tick_commit_control_buffer : register(u1);
RWStructuredBuffer<TickStatus> tick_status_buffer : register(u2);

static const uint TICK_FLAG_EMPTY_WORLD = 1u;

static const uint TICK_GATE_READY = 1u;
static const uint TICK_GATE_BLOCKED_PREDECESSOR = 2u;
static const uint TICK_GATE_INVALID_PLAN = 3u;

static const uint TICK_TERMINAL_IN_FLIGHT = 1u;
static const uint TICK_TERMINAL_VALIDATED = 2u;
static const uint TICK_TERMINAL_BLOCKED = 3u;
static const uint TICK_TERMINAL_REJECTED = 4u;
static const uint TICK_TERMINAL_FAILED = 5u;
static const uint TICK_TERMINAL_COMMITTED = 6u;

bool tick_is_zero_u64(uint low, uint high)
{
    return low == 0u && high == 0u;
}

bool tick_u64_is_next(
    uint previous_low,
    uint previous_high,
    uint next_low,
    uint next_high)
{
    if (previous_low == 0xffffffffu && previous_high == 0xffffffffu)
    {
        return false;
    }

    uint expected_low = previous_low + 1u;
    uint carry = expected_low == 0u ? 1u : 0u;
    uint expected_high = previous_high + carry;
    return next_low == expected_low && next_high == expected_high;
}

bool tick_plan_identity_matches(
    TickDispatchPlan plan,
    TickCommitControl control)
{
    return
        plan.dispatch_id_low == control.dispatch_id_low &&
        plan.dispatch_id_high == control.dispatch_id_high &&
        plan.reservation_token_low == control.reservation_token_low &&
        plan.reservation_token_high == control.reservation_token_high;
}

void publish_tick_status(
    TickDispatchPlan plan,
    TickCommitControl control)
{
    TickStatus value;
    value.terminal_state = control.terminal_state;
    value.terminal_failure = control.terminal_failure;
    value.committed_sequence_low = control.committed_sequence_low;
    value.committed_sequence_high = control.committed_sequence_high;
    value.state_generation_low = control.state_generation_low;
    value.state_generation_high = control.state_generation_high;
    value.active_state_set = control.active_state_set;
    value.diagnostic_code = control.diagnostic;
    value.dispatch_id_low = plan.dispatch_id_low;
    value.dispatch_id_high = plan.dispatch_id_high;
    value.reservation_token_low = plan.reservation_token_low;
    value.reservation_token_high = plan.reservation_token_high;
    value.validation_mask = control.validation_mask;
    value.required_validation_mask = control.required_validation_mask;
    value.work_budget_block_flags = 0u;
    value.work_budget_block_origin = 0u;
    value.executed_integration_segments = 0u;
    value.minimum_required_integration_segments = 0u;
    value.reserved0 = control.reserved & 0x0000ffffu;
    value.reserved1 = (control.reserved >> 16u) & 0x0000ffffu;
    tick_status_buffer[0] = value;
}

[numthreads(1, 1, 1)]
void tick_begin_control(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u)
    {
        return;
    }

    TickDispatchPlan plan = tick_dispatch_plan[0];
    TickCommitControl control = tick_commit_control_buffer[0];

    bool valid_plan =
        plan.expected_source_state_set <= 1u &&
        plan.destination_state_set <= 1u &&
        plan.expected_source_state_set != plan.destination_state_set &&
        !tick_is_zero_u64(plan.dispatch_id_low, plan.dispatch_id_high) &&
        !tick_is_zero_u64(plan.reservation_token_low, plan.reservation_token_high) &&
        plan.required_validation_mask != 0u &&
        tick_u64_is_next(
            plan.expected_predecessor_sequence_low,
            plan.expected_predecessor_sequence_high,
            plan.tick_sequence_low,
            plan.tick_sequence_high);

    if (!valid_plan)
    {
        control.last_gate_state = TICK_GATE_INVALID_PLAN;
        control.terminal_state = TICK_TERMINAL_REJECTED;
        control.terminal_failure = 1u;
        control.diagnostic = 1u;
        tick_commit_control_buffer[0] = control;
        publish_tick_status(plan, control);
        return;
    }

    bool predecessor_matches =
        control.committed_sequence_low == plan.expected_predecessor_sequence_low &&
        control.committed_sequence_high == plan.expected_predecessor_sequence_high &&
        control.state_generation_low == plan.expected_source_generation_low &&
        control.state_generation_high == plan.expected_source_generation_high &&
        control.active_state_set == plan.expected_source_state_set;

    control.dispatch_id_low = plan.dispatch_id_low;
    control.dispatch_id_high = plan.dispatch_id_high;
    control.reservation_token_low = plan.reservation_token_low;
    control.reservation_token_high = plan.reservation_token_high;
    control.required_validation_mask = plan.required_validation_mask;
    control.validation_mask = 0u;
    control.terminal_failure = 0u;
    control.diagnostic = 0u;

    if (!predecessor_matches)
    {
        control.last_gate_state = TICK_GATE_BLOCKED_PREDECESSOR;
        control.terminal_state = TICK_TERMINAL_BLOCKED;
    }
    else
    {
        control.last_gate_state = TICK_GATE_READY;
        control.terminal_state = TICK_TERMINAL_IN_FLIGHT;
    }

    tick_commit_control_buffer[0] = control;
    publish_tick_status(plan, control);
}

[numthreads(1, 1, 1)]
void tick_mark_empty_world_validated(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u)
    {
        return;
    }

    TickDispatchPlan plan = tick_dispatch_plan[0];
    TickCommitControl control = tick_commit_control_buffer[0];

    if (control.terminal_state == TICK_TERMINAL_IN_FLIGHT &&
        control.last_gate_state == TICK_GATE_READY &&
        (plan.flags & TICK_FLAG_EMPTY_WORLD) != 0u)
    {
        control.validation_mask = control.required_validation_mask;
        control.terminal_state = TICK_TERMINAL_VALIDATED;
    }

    tick_commit_control_buffer[0] = control;
    publish_tick_status(plan, control);
}

[numthreads(1, 1, 1)]
void tick_validate_soft_body(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u)
    {
        return;
    }

    TickDispatchPlan plan = tick_dispatch_plan[0];
    TickCommitControl control = tick_commit_control_buffer[0];

    if (control.last_gate_state != TICK_GATE_READY ||
        control.terminal_state != TICK_TERMINAL_IN_FLIGHT)
    {
        publish_tick_status(plan, control);
        return;
    }

    if (control.terminal_failure != 0u)
    {
        // A solver-side numerical / non-finite failure is terminal for the World.
        // REJECTED is reserved for an invalid or incomplete Tick contract.
        control.terminal_state = TICK_TERMINAL_FAILED;
        control.terminal_failure = 1u;
        control.validation_mask = 0u;
        control.diagnostic = 5u;
    }
    else
    {
        control.validation_mask = control.required_validation_mask;
        control.terminal_state = TICK_TERMINAL_VALIDATED;
        control.diagnostic = 0u;
    }

    tick_commit_control_buffer[0] = control;
    publish_tick_status(plan, control);
}

[numthreads(1, 1, 1)]
void tick_commit_control(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u)
    {
        return;
    }

    TickDispatchPlan plan = tick_dispatch_plan[0];
    TickCommitControl control = tick_commit_control_buffer[0];

    if (control.last_gate_state == TICK_GATE_INVALID_PLAN)
    {
        publish_tick_status(plan, control);
        return;
    }

    if (control.last_gate_state != TICK_GATE_READY ||
        control.terminal_state == TICK_TERMINAL_BLOCKED ||
        !tick_plan_identity_matches(plan, control))
    {
        control.terminal_state = TICK_TERMINAL_BLOCKED;
        tick_commit_control_buffer[0] = control;
        publish_tick_status(plan, control);
        return;
    }

    bool predecessor_still_matches =
        control.committed_sequence_low == plan.expected_predecessor_sequence_low &&
        control.committed_sequence_high == plan.expected_predecessor_sequence_high &&
        control.state_generation_low == plan.expected_source_generation_low &&
        control.state_generation_high == plan.expected_source_generation_high &&
        control.active_state_set == plan.expected_source_state_set;

    if (!predecessor_still_matches)
    {
        control.terminal_state = TICK_TERMINAL_BLOCKED;
        control.diagnostic = 2u;
        tick_commit_control_buffer[0] = control;
        publish_tick_status(plan, control);
        return;
    }

    if (control.terminal_state == TICK_TERMINAL_REJECTED ||
        control.terminal_state == TICK_TERMINAL_FAILED)
    {
        publish_tick_status(plan, control);
        return;
    }

    if (control.terminal_state != TICK_TERMINAL_VALIDATED ||
        control.terminal_failure != 0u ||
        (control.validation_mask & control.required_validation_mask) !=
            control.required_validation_mask)
    {
        control.terminal_state = TICK_TERMINAL_REJECTED;
        control.diagnostic = 3u;
        tick_commit_control_buffer[0] = control;
        publish_tick_status(plan, control);
        return;
    }

    if (control.state_generation_low == 0xffffffffu &&
        control.state_generation_high == 0xffffffffu)
    {
        control.terminal_state = TICK_TERMINAL_FAILED;
        control.diagnostic = 4u;
        tick_commit_control_buffer[0] = control;
        publish_tick_status(plan, control);
        return;
    }

    uint next_generation_low = control.state_generation_low + 1u;
    uint generation_carry = next_generation_low == 0u ? 1u : 0u;

    control.active_state_set = plan.destination_state_set;
    if (plan.reserved0 != 0xffffffffu)
    {
        control.reserved =
            ((plan.reserved1 & 0x0000ffffu) << 16u) |
            (plan.reserved0 & 0x0000ffffu);
    }
    control.committed_sequence_low = plan.tick_sequence_low;
    control.committed_sequence_high = plan.tick_sequence_high;
    control.state_generation_low = next_generation_low;
    control.state_generation_high += generation_carry;
    control.terminal_state = TICK_TERMINAL_COMMITTED;
    control.diagnostic = 0u;

    tick_commit_control_buffer[0] = control;
    publish_tick_status(plan, control);
}
