struct ClothStretchConstraint
{
    uint2 vertices;
    float rest_length;
    float reserved;
};

cbuffer ClothStretchSolveParameters : register(b0)
{
    uint first_color_index;
    uint color_constraint_count;
    uint vertex_count;
    uint constraint_count;
    float delta_time;
    float compliance;
    float relaxation;
    uint reserved;
};

StructuredBuffer<float> inverse_masses : register(t0);
StructuredBuffer<ClothStretchConstraint> stretch_constraints : register(t1);
StructuredBuffer<uint> color_constraint_indices : register(t2);
RWStructuredBuffer<float4> positions : register(u0);
RWStructuredBuffer<float> stretch_lambdas : register(u1);
RWStructuredBuffer<uint> solver_status : register(u2);

static const uint STATUS_INVALID_STRETCH = 1u << 0u;
static const float MINIMUM_LENGTH = 1.0e-12f;
static const float MINIMUM_DENOMINATOR = 1.0e-20f;

void fail_stretch()
{
    uint ignored = 0u;
    InterlockedOr(solver_status[0], STATUS_INVALID_STRETCH, ignored);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint local_index = dispatch_thread_id.x;
    if (local_index >= color_constraint_count)
    {
        return;
    }

    uint position_count = 0u;
    uint position_stride = 0u;
    uint mass_count = 0u;
    uint mass_stride = 0u;
    uint constraint_buffer_count = 0u;
    uint constraint_stride = 0u;
    uint color_index_count = 0u;
    uint color_index_stride = 0u;
    uint lambda_count = 0u;
    uint lambda_stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    positions.GetDimensions(position_count, position_stride);
    inverse_masses.GetDimensions(mass_count, mass_stride);
    stretch_constraints.GetDimensions(
        constraint_buffer_count, constraint_stride);
    color_constraint_indices.GetDimensions(
        color_index_count, color_index_stride);
    stretch_lambdas.GetDimensions(lambda_count, lambda_stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (reserved != 0u ||
        vertex_count == 0u ||
        vertex_count != position_count ||
        vertex_count != mass_count ||
        position_stride != 16u ||
        mass_stride != 4u ||
        constraint_count == 0u ||
        constraint_count > constraint_buffer_count ||
        constraint_stride != 16u ||
        constraint_count != lambda_count ||
        lambda_stride != 4u ||
        status_count != 1u ||
        status_stride != 4u ||
        color_constraint_count == 0u ||
        first_color_index > color_index_count ||
        color_constraint_count > color_index_count - first_color_index ||
        !isfinite(delta_time) || delta_time <= 0.0f ||
        !isfinite(compliance) || compliance < 0.0f ||
        !isfinite(relaxation) || relaxation <= 0.0f || relaxation > 1.0f)
    {
        fail_stretch();
        return;
    }

    const uint constraint_index =
        color_constraint_indices[first_color_index + local_index];
    if (constraint_index >= constraint_count)
    {
        fail_stretch();
        return;
    }

    const ClothStretchConstraint constraint =
        stretch_constraints[constraint_index];
    const uint first = constraint.vertices.x;
    const uint second = constraint.vertices.y;
    if (first >= vertex_count || second >= vertex_count ||
        first == second ||
        !isfinite(constraint.rest_length) ||
        constraint.rest_length <= MINIMUM_LENGTH ||
        !isfinite(constraint.reserved) ||
        constraint.reserved != 0.0f)
    {
        fail_stretch();
        return;
    }

    const float4 first_position = positions[first];
    const float4 second_position = positions[second];
    const float first_inverse_mass = inverse_masses[first];
    const float second_inverse_mass = inverse_masses[second];
    const float old_lambda = stretch_lambdas[constraint_index];
    if (!all(isfinite(first_position)) ||
        !all(isfinite(second_position)) ||
        !isfinite(first_inverse_mass) || first_inverse_mass < 0.0f ||
        !isfinite(second_inverse_mass) || second_inverse_mass < 0.0f ||
        !isfinite(old_lambda))
    {
        fail_stretch();
        return;
    }

    const float3 delta = first_position.xyz - second_position.xyz;
    const float current_length = length(delta);
    if (!isfinite(current_length) || current_length <= MINIMUM_LENGTH)
    {
        fail_stretch();
        return;
    }

    const float alpha = compliance / (delta_time * delta_time);
    const float denominator =
        first_inverse_mass + second_inverse_mass + alpha;
    if (!isfinite(alpha) || !isfinite(denominator))
    {
        fail_stretch();
        return;
    }
    if (denominator <= MINIMUM_DENOMINATOR)
    {
        return;
    }

    const float constraint_value = current_length - constraint.rest_length;
    const float delta_lambda =
        (-constraint_value - alpha * old_lambda) / denominator;
    const float applied_delta_lambda = relaxation * delta_lambda;
    const float next_lambda = old_lambda + applied_delta_lambda;
    const float3 direction = delta / current_length;
    const float3 first_correction =
        direction * (first_inverse_mass * applied_delta_lambda);
    const float3 second_correction =
        -direction * (second_inverse_mass * applied_delta_lambda);
    const float3 next_first_position =
        first_position.xyz + first_correction;
    const float3 next_second_position =
        second_position.xyz + second_correction;

    if (!isfinite(constraint_value) ||
        !isfinite(delta_lambda) ||
        !isfinite(applied_delta_lambda) ||
        !isfinite(next_lambda) ||
        !all(isfinite(next_first_position)) ||
        !all(isfinite(next_second_position)))
    {
        fail_stretch();
        return;
    }

    // A validated graph color guarantees no other lane writes either vertex.
    positions[first] = float4(next_first_position, first_position.w);
    positions[second] = float4(next_second_position, second_position.w);
    stretch_lambdas[constraint_index] = next_lambda;
}
