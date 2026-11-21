struct ClothConstraint
{
    uint4 indices;
    uint type;
    float rest_value;
    float normalization;
    float compliance;
};

struct ClothContribution
{
    float4 correction0;
    float4 correction1;
    float4 correction2;
    float4 correction3;
};

cbuffer ClothJacobiParameters : register(b0)
{
    uint constraint_count;
    uint vertex_count;
    float delta_time;
    float relaxation;
};

StructuredBuffer<float4> current_positions : register(t0);
StructuredBuffer<float> inverse_masses : register(t1);
StructuredBuffer<ClothConstraint> constraints : register(t2);
RWStructuredBuffer<ClothContribution> contributions : register(u0);
RWStructuredBuffer<float> constraint_lambdas : register(u1);
RWStructuredBuffer<uint> solver_status : register(u2);

static const uint CLOTH_CONSTRAINT_SHEAR = 1u;
static const uint CLOTH_CONSTRAINT_BEND = 2u;
static const uint STATUS_INVALID_CLOTH_CONSTRAINT = 1u << 1u;
static const float MINIMUM_LENGTH = 1.0e-10f;
static const float MINIMUM_DENOMINATOR = 1.0e-20f;

void fail_cloth_constraint()
{
    uint ignored = 0u;
    InterlockedOr(
        solver_status[0], STATUS_INVALID_CLOTH_CONSTRAINT, ignored);
}

float3 normalized_derivative(float3 value, float3 derivative)
{
    const float value_length = length(value);
    if (!isfinite(value_length) || value_length <= 1.0e-12f)
    {
        return 0.0f;
    }
    const float3 direction = value / value_length;
    return (derivative - direction * dot(direction, derivative)) /
        value_length;
}

float bend_directional_derivative(
    float3 p0, float3 p1, float3 p2, float3 p3,
    float3 d0, float3 d1, float3 d2, float3 d3)
{
    const float3 edge = p1 - p0;
    const float3 u = p2 - p0;
    const float3 v = p3 - p0;
    const float3 first_cross = cross(edge, u);
    const float3 second_cross = cross(v, edge);
    const float3 edge_direction = normalize(edge);
    const float3 normal_first = normalize(first_cross);
    const float3 normal_second = normalize(second_cross);
    const float sine_value =
        dot(edge_direction, cross(normal_first, normal_second));
    const float cosine_value = dot(normal_first, normal_second);

    const float3 delta_edge = d1 - d0;
    const float3 delta_u = d2 - d0;
    const float3 delta_v = d3 - d0;
    const float3 delta_first_cross =
        cross(delta_edge, u) + cross(edge, delta_u);
    const float3 delta_second_cross =
        cross(delta_v, edge) + cross(v, delta_edge);
    const float3 delta_normal_first =
        normalized_derivative(first_cross, delta_first_cross);
    const float3 delta_normal_second =
        normalized_derivative(second_cross, delta_second_cross);
    const float3 delta_edge_direction =
        normalized_derivative(edge, delta_edge);

    const float delta_sine =
        dot(delta_edge_direction, cross(normal_first, normal_second)) +
        dot(edge_direction,
            cross(delta_normal_first, normal_second) +
            cross(normal_first, delta_normal_second));
    const float delta_cosine =
        dot(delta_normal_first, normal_second) +
        dot(normal_first, delta_normal_second);
    const float denominator =
        sine_value * sine_value + cosine_value * cosine_value;
    if (!isfinite(denominator) || denominator <= 1.0e-20f)
    {
        return 0.0f;
    }
    return (cosine_value * delta_sine -
        sine_value * delta_cosine) / denominator;
}

float evaluate_bend(
    float3 p0, float3 p1, float3 p2, float3 p3,
    float rest_angle,
    out float3 gradient0,
    out float3 gradient1,
    out float3 gradient2,
    out float3 gradient3)
{
    const float3 edge = p1 - p0;
    const float3 u = p2 - p0;
    const float3 v = p3 - p0;
    const float3 first_cross = cross(edge, u);
    const float3 second_cross = cross(v, edge);
    const float edge_length = length(edge);
    const float first_length = length(first_cross);
    const float second_length = length(second_cross);

    gradient0 = 0.0f;
    gradient1 = 0.0f;
    gradient2 = 0.0f;
    gradient3 = 0.0f;
    if (!isfinite(edge_length) || !isfinite(first_length) ||
        !isfinite(second_length) || edge_length <= MINIMUM_LENGTH ||
        first_length <= MINIMUM_LENGTH || second_length <= MINIMUM_LENGTH)
    {
        return 0.0f;
    }

    const float3 edge_direction = edge / edge_length;
    const float3 normal_first = first_cross / first_length;
    const float3 normal_second = second_cross / second_length;
    const float sine_value =
        dot(edge_direction, cross(normal_first, normal_second));
    const float cosine_value = dot(normal_first, normal_second);
    const float angle = atan2(sine_value, cosine_value);
    const float constraint_value =
        atan2(sin(angle - rest_angle), cos(angle - rest_angle));

    [unroll]
    for (uint vertex_index = 0u; vertex_index < 4u; ++vertex_index)
    {
        const float3 axis_x = float3(1.0f, 0.0f, 0.0f);
        const float3 axis_y = float3(0.0f, 1.0f, 0.0f);
        const float3 axis_z = float3(0.0f, 0.0f, 1.0f);
        const float3 zero = 0.0f;
        const float3 d0x = vertex_index == 0u ? axis_x : zero;
        const float3 d1x = vertex_index == 1u ? axis_x : zero;
        const float3 d2x = vertex_index == 2u ? axis_x : zero;
        const float3 d3x = vertex_index == 3u ? axis_x : zero;
        const float3 d0y = vertex_index == 0u ? axis_y : zero;
        const float3 d1y = vertex_index == 1u ? axis_y : zero;
        const float3 d2y = vertex_index == 2u ? axis_y : zero;
        const float3 d3y = vertex_index == 3u ? axis_y : zero;
        const float3 d0z = vertex_index == 0u ? axis_z : zero;
        const float3 d1z = vertex_index == 1u ? axis_z : zero;
        const float3 d2z = vertex_index == 2u ? axis_z : zero;
        const float3 d3z = vertex_index == 3u ? axis_z : zero;

        const float gx = bend_directional_derivative(
            p0, p1, p2, p3, d0x, d1x, d2x, d3x);
        const float gy = bend_directional_derivative(
            p0, p1, p2, p3, d0y, d1y, d2y, d3y);
        const float gz = bend_directional_derivative(
            p0, p1, p2, p3, d0z, d1z, d2z, d3z);
        const float3 gradient = float3(gx, gy, gz);
        if (vertex_index == 0u) gradient0 = gradient;
        if (vertex_index == 1u) gradient1 = gradient;
        if (vertex_index == 2u) gradient2 = gradient;
        if (vertex_index == 3u) gradient3 = gradient;
    }
    return constraint_value;
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint constraint_index = dispatch_thread_id.x;
    if (constraint_index >= constraint_count)
    {
        return;
    }

    uint position_count = 0u;
    uint position_stride = 0u;
    uint mass_count = 0u;
    uint mass_stride = 0u;
    uint constraint_buffer_count = 0u;
    uint constraint_stride = 0u;
    uint contribution_count = 0u;
    uint contribution_stride = 0u;
    uint lambda_count = 0u;
    uint lambda_stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    current_positions.GetDimensions(position_count, position_stride);
    inverse_masses.GetDimensions(mass_count, mass_stride);
    constraints.GetDimensions(constraint_buffer_count, constraint_stride);
    contributions.GetDimensions(contribution_count, contribution_stride);
    constraint_lambdas.GetDimensions(lambda_count, lambda_stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (status_count != 1u || status_stride != 4u)
    {
        return;
    }
    if (constraint_count == 0u ||
        constraint_count > constraint_buffer_count ||
        constraint_count > contribution_count ||
        constraint_count > lambda_count ||
        vertex_count == 0u || vertex_count != position_count ||
        vertex_count != mass_count ||
        position_stride != 16u || mass_stride != 4u ||
        constraint_stride != 32u || contribution_stride != 64u ||
        lambda_stride != 4u ||
        !isfinite(delta_time) || delta_time <= 0.0f ||
        !isfinite(relaxation) || relaxation <= 0.0f ||
        relaxation > 1.0f)
    {
        fail_cloth_constraint();
        return;
    }

    // Clear the per-constraint output before validation / evaluation so a
    // failed constraint cannot leave a stale correction from a prior pass.
    contributions[constraint_index] = (ClothContribution)0;

    const ClothConstraint constraint = constraints[constraint_index];
    const uint4 ids = constraint.indices;
    if ((constraint.type != CLOTH_CONSTRAINT_SHEAR &&
            constraint.type != CLOTH_CONSTRAINT_BEND) ||
        ids.x >= vertex_count || ids.y >= vertex_count ||
        ids.z >= vertex_count ||
        ids.x == ids.y || ids.y == ids.z || ids.z == ids.x ||
        !isfinite(constraint.rest_value) ||
        !isfinite(constraint.normalization) ||
        !isfinite(constraint.compliance) ||
        constraint.compliance < 0.0f ||
        !isfinite(constraint_lambdas[constraint_index]))
    {
        fail_cloth_constraint();
        return;
    }
    if (constraint.type == CLOTH_CONSTRAINT_BEND &&
        (ids.w >= vertex_count || ids.w == ids.x ||
            ids.w == ids.y || ids.w == ids.z))
    {
        fail_cloth_constraint();
        return;
    }

    const float4 p0_value = current_positions[ids.x];
    const float4 p1_value = current_positions[ids.y];
    const float4 p2_value = current_positions[ids.z];
    const float4 p3_value = constraint.type == CLOTH_CONSTRAINT_BEND
        ? current_positions[ids.w] : 0.0f;
    const float w0 = inverse_masses[ids.x];
    const float w1 = inverse_masses[ids.y];
    const float w2 = inverse_masses[ids.z];
    const float w3 = constraint.type == CLOTH_CONSTRAINT_BEND
        ? inverse_masses[ids.w] : 0.0f;
    if (!all(isfinite(p0_value)) || !all(isfinite(p1_value)) ||
        !all(isfinite(p2_value)) || !all(isfinite(p3_value)) ||
        !isfinite(w0) || !isfinite(w1) || !isfinite(w2) ||
        !isfinite(w3) || w0 < 0.0f || w1 < 0.0f ||
        w2 < 0.0f || w3 < 0.0f)
    {
        fail_cloth_constraint();
        return;
    }

    const float3 p0 = p0_value.xyz;
    const float3 p1 = p1_value.xyz;
    const float3 p2 = p2_value.xyz;
    const float3 p3 = p3_value.xyz;
    float3 gradients[4] = {
        float3(0.0f, 0.0f, 0.0f),
        float3(0.0f, 0.0f, 0.0f),
        float3(0.0f, 0.0f, 0.0f),
        float3(0.0f, 0.0f, 0.0f)
    };
    float constraint_value = 0.0f;
    if (constraint.type == CLOTH_CONSTRAINT_SHEAR)
    {
        if (constraint.normalization <= MINIMUM_LENGTH)
        {
            fail_cloth_constraint();
            return;
        }
        const float3 u = p1 - p0;
        const float3 v = p2 - p0;
        gradients[0] = -(u + v) / constraint.normalization;
        gradients[1] = v / constraint.normalization;
        gradients[2] = u / constraint.normalization;
        constraint_value =
            (dot(u, v) - constraint.rest_value) /
            constraint.normalization;
    }
    else
    {
        constraint_value = evaluate_bend(
            p0, p1, p2, p3, constraint.rest_value,
            gradients[0], gradients[1], gradients[2], gradients[3]);
    }

    const float alpha =
        constraint.compliance / (delta_time * delta_time);
    float denominator = alpha +
        w0 * dot(gradients[0], gradients[0]) +
        w1 * dot(gradients[1], gradients[1]) +
        w2 * dot(gradients[2], gradients[2]);
    if (constraint.type == CLOTH_CONSTRAINT_BEND)
    {
        denominator += w3 * dot(gradients[3], gradients[3]);
    }
    if (!isfinite(constraint_value) || !isfinite(alpha) ||
        !isfinite(denominator))
    {
        fail_cloth_constraint();
        return;
    }

    ClothContribution output_value = (ClothContribution)0;
    if (denominator > MINIMUM_DENOMINATOR)
    {
        const float old_lambda = constraint_lambdas[constraint_index];
        const float delta_lambda =
            (-constraint_value - alpha * old_lambda) / denominator;
        const float applied = relaxation * delta_lambda;
        const float next_lambda = old_lambda + applied;
        if (!isfinite(delta_lambda) || !isfinite(applied) ||
            !isfinite(next_lambda))
        {
            fail_cloth_constraint();
            return;
        }

        output_value.correction0 =
            float4(gradients[0] * (w0 * applied), 0.0f);
        output_value.correction1 =
            float4(gradients[1] * (w1 * applied), 0.0f);
        output_value.correction2 =
            float4(gradients[2] * (w2 * applied), 0.0f);
        output_value.correction3 =
            float4(gradients[3] * (w3 * applied), 0.0f);
        constraint_lambdas[constraint_index] = next_lambda;
    }
    contributions[constraint_index] = output_value;
}
