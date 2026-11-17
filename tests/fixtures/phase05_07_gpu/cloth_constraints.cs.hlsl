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
    float4 lambda;
};

StructuredBuffer<float4> current_positions : register(t0);
StructuredBuffer<float> inverse_masses : register(t1);
StructuredBuffer<ClothConstraint> constraints : register(t2);
RWStructuredBuffer<ClothContribution> contributions : register(u0);

cbuffer StepParameters : register(b0)
{
    float delta_time;
    float relaxation;
    uint constraint_count;
    uint reserved;
};

float3 normalized_derivative(float3 value, float3 derivative)
{
    float value_length = length(value);
    if (value_length <= 1.0e-12f)
    {
        return float3(0.0f, 0.0f, 0.0f);
    }
    float3 direction = value / value_length;
    return (derivative - direction * dot(direction, derivative)) / value_length;
}

float bend_directional_derivative(
    float3 p0, float3 p1, float3 p2, float3 p3,
    float3 d0, float3 d1, float3 d2, float3 d3)
{
    float3 edge = p1 - p0;
    float3 u = p2 - p0;
    float3 v = p3 - p0;
    float3 first_cross = cross(edge, u);
    float3 second_cross = cross(v, edge);
    float3 edge_direction = normalize(edge);
    float3 normal_first = normalize(first_cross);
    float3 normal_second = normalize(second_cross);
    float sine_value = dot(edge_direction, cross(normal_first, normal_second));
    float cosine_value = dot(normal_first, normal_second);

    float3 delta_edge = d1 - d0;
    float3 delta_u = d2 - d0;
    float3 delta_v = d3 - d0;

    float3 delta_first_cross =
        cross(delta_edge, u) + cross(edge, delta_u);
    float3 delta_second_cross =
        cross(delta_v, edge) + cross(v, delta_edge);
    float3 delta_normal_first =
        normalized_derivative(first_cross, delta_first_cross);
    float3 delta_normal_second =
        normalized_derivative(second_cross, delta_second_cross);
    float3 delta_edge_direction =
        normalized_derivative(edge, delta_edge);

    float delta_sine =
        dot(delta_edge_direction, cross(normal_first, normal_second)) +
        dot(edge_direction,
            cross(delta_normal_first, normal_second) +
            cross(normal_first, delta_normal_second));
    float delta_cosine =
        dot(delta_normal_first, normal_second) +
        dot(normal_first, delta_normal_second);

    float denominator = sine_value * sine_value + cosine_value * cosine_value;
    if (denominator <= 1.0e-20f)
    {
        return 0.0f;
    }
    return (cosine_value * delta_sine - sine_value * delta_cosine) / denominator;
}

float evaluate_bend(
    float3 p0, float3 p1, float3 p2, float3 p3,
    float rest_angle,
    out float3 gradient0,
    out float3 gradient1,
    out float3 gradient2,
    out float3 gradient3)
{
    float3 edge = p1 - p0;
    float3 u = p2 - p0;
    float3 v = p3 - p0;
    float3 first_cross = cross(edge, u);
    float3 second_cross = cross(v, edge);
    float edge_length = length(edge);
    float first_length = length(first_cross);
    float second_length = length(second_cross);

    gradient0 = 0.0f;
    gradient1 = 0.0f;
    gradient2 = 0.0f;
    gradient3 = 0.0f;
    if (edge_length <= 1.0e-10f ||
        first_length <= 1.0e-10f ||
        second_length <= 1.0e-10f)
    {
        return 0.0f;
    }

    float3 edge_direction = edge / edge_length;
    float3 normal_first = first_cross / first_length;
    float3 normal_second = second_cross / second_length;
    float sine_value = dot(edge_direction, cross(normal_first, normal_second));
    float cosine_value = dot(normal_first, normal_second);
    float angle = atan2(sine_value, cosine_value);
    float constraint_value = atan2(sin(angle - rest_angle), cos(angle - rest_angle));

    [unroll]
    for (uint vertex_index = 0; vertex_index < 4; ++vertex_index)
    {
        float3 axis_x = float3(0.0f, 0.0f, 0.0f);
        float3 axis_y = float3(0.0f, 0.0f, 0.0f);
        float3 axis_z = float3(0.0f, 0.0f, 0.0f);
        if (vertex_index == 0) { axis_x.x = 1.0f; axis_y.y = 1.0f; axis_z.z = 1.0f; }
        if (vertex_index == 1) { axis_x.x = 1.0f; axis_y.y = 1.0f; axis_z.z = 1.0f; }
        if (vertex_index == 2) { axis_x.x = 1.0f; axis_y.y = 1.0f; axis_z.z = 1.0f; }
        if (vertex_index == 3) { axis_x.x = 1.0f; axis_y.y = 1.0f; axis_z.z = 1.0f; }

        float3 zero = float3(0.0f, 0.0f, 0.0f);
        float3 d0x = vertex_index == 0 ? axis_x : zero;
        float3 d1x = vertex_index == 1 ? axis_x : zero;
        float3 d2x = vertex_index == 2 ? axis_x : zero;
        float3 d3x = vertex_index == 3 ? axis_x : zero;
        float3 d0y = vertex_index == 0 ? axis_y : zero;
        float3 d1y = vertex_index == 1 ? axis_y : zero;
        float3 d2y = vertex_index == 2 ? axis_y : zero;
        float3 d3y = vertex_index == 3 ? axis_y : zero;
        float3 d0z = vertex_index == 0 ? axis_z : zero;
        float3 d1z = vertex_index == 1 ? axis_z : zero;
        float3 d2z = vertex_index == 2 ? axis_z : zero;
        float3 d3z = vertex_index == 3 ? axis_z : zero;

        float gx = bend_directional_derivative(
            p0, p1, p2, p3, d0x, d1x, d2x, d3x);
        float gy = bend_directional_derivative(
            p0, p1, p2, p3, d0y, d1y, d2y, d3y);
        float gz = bend_directional_derivative(
            p0, p1, p2, p3, d0z, d1z, d2z, d3z);
        if (vertex_index == 0) gradient0 = float3(gx, gy, gz);
        if (vertex_index == 1) gradient1 = float3(gx, gy, gz);
        if (vertex_index == 2) gradient2 = float3(gx, gy, gz);
        if (vertex_index == 3) gradient3 = float3(gx, gy, gz);
    }
    return constraint_value;
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint index = dispatch_id.x;
    if (index >= constraint_count)
    {
        return;
    }

    ClothConstraint constraint = constraints[index];
    uint4 ids = constraint.indices;
    float3 p0 = current_positions[ids.x].xyz;
    float3 p1 = current_positions[ids.y].xyz;
    float3 p2 = current_positions[ids.z].xyz;
    float3 p3 = current_positions[ids.w].xyz;
    float3 gradients[4] = {
        float3(0.0f, 0.0f, 0.0f),
        float3(0.0f, 0.0f, 0.0f),
        float3(0.0f, 0.0f, 0.0f),
        float3(0.0f, 0.0f, 0.0f)
    };
    float constraint_value = 0.0f;

    if (constraint.type == 0)
    {
        float3 delta = p0 - p1;
        float current_length = length(delta);
        if (current_length > 1.0e-12f)
        {
            gradients[0] = delta / current_length;
            gradients[1] = -gradients[0];
            constraint_value = current_length - constraint.rest_value;
        }
    }
    else if (constraint.type == 1)
    {
        float3 u = p1 - p0;
        float3 v = p2 - p0;
        float normalization = constraint.normalization;
        if (normalization > 1.0e-12f)
        {
            gradients[0] = -(u + v) / normalization;
            gradients[1] = v / normalization;
            gradients[2] = u / normalization;
            constraint_value = (dot(u, v) - constraint.rest_value) / normalization;
        }
    }
    else if (constraint.type == 2)
    {
        constraint_value = evaluate_bend(
            p0, p1, p2, p3, constraint.rest_value,
            gradients[0], gradients[1], gradients[2], gradients[3]);
    }

    float alpha = constraint.compliance / (delta_time * delta_time);
    float denominator = alpha;
    denominator += inverse_masses[ids.x] * dot(gradients[0], gradients[0]);
    denominator += inverse_masses[ids.y] * dot(gradients[1], gradients[1]);
    if (constraint.type != 0)
    {
        denominator += inverse_masses[ids.z] * dot(gradients[2], gradients[2]);
    }
    if (constraint.type == 2)
    {
        denominator += inverse_masses[ids.w] * dot(gradients[3], gradients[3]);
    }

    float delta_lambda = 0.0f;
    if (denominator > 1.0e-20f)
    {
        delta_lambda = relaxation *
            (-constraint_value - alpha * 0.0f) / denominator;
    }

    ClothContribution output;
    output.correction0 = float4(gradients[0] * inverse_masses[ids.x] * delta_lambda, 0.0f);
    output.correction1 = float4(gradients[1] * inverse_masses[ids.y] * delta_lambda, 0.0f);
    output.correction2 = float4(gradients[2] * inverse_masses[ids.z] * delta_lambda, 0.0f);
    output.correction3 = float4(gradients[3] * inverse_masses[ids.w] * delta_lambda, 0.0f);
    output.lambda = float4(delta_lambda, 0.0f, 0.0f, 0.0f);
    contributions[index] = output;
}
