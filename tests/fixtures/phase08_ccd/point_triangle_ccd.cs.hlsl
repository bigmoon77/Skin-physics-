StructuredBuffer<float4> swept_geometry : register(t0);
RWStructuredBuffer<float4> ccd_output : register(u0);

cbuffer CcdParameters : register(b0)
{
    float safety_factor;
    float distance_tolerance;
    float combined_thickness;
    uint maximum_iterations;
};

float3 closest_point_triangle(
    float3 position_value,
    float3 a,
    float3 b,
    float3 c,
    out float3 barycentric)
{
    float3 ab = b - a;
    float3 ac = c - a;
    float3 ap = position_value - a;
    float d1 = dot(ab, ap);
    float d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        barycentric = float3(1.0f, 0.0f, 0.0f);
        return a;
    }
    float3 bp = position_value - b;
    float d3 = dot(ab, bp);
    float d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
    {
        barycentric = float3(0.0f, 1.0f, 0.0f);
        return b;
    }
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        float v = d1 / (d1 - d3);
        barycentric = float3(1.0f - v, v, 0.0f);
        return a + v * ab;
    }
    float3 cp = position_value - c;
    float d5 = dot(ab, cp);
    float d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
    {
        barycentric = float3(0.0f, 0.0f, 1.0f);
        return c;
    }
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        float w = d2 / (d2 - d6);
        barycentric = float3(1.0f - w, 0.0f, w);
        return a + w * ac;
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        barycentric = float3(0.0f, 1.0f - w, w);
        return b + w * (c - b);
    }
    float denominator = 1.0f / (va + vb + vc);
    float v_inside = vb * denominator;
    float w_inside = vc * denominator;
    barycentric = float3(1.0f - v_inside - w_inside, v_inside, w_inside);
    return a + ab * v_inside + ac * w_inside;
}

[numthreads(1, 1, 1)]
void point_triangle_ccd(uint3 dispatch_id : SV_DispatchThreadID)
{
    float3 start_position = swept_geometry[0].xyz;
    float3 end_position = swept_geometry[1].xyz;
    float3 triangle0 = swept_geometry[2].xyz;
    float3 triangle1 = swept_geometry[3].xyz;
    float3 triangle2 = swept_geometry[4].xyz;
    float3 velocity = end_position - start_position;
    float speed = length(velocity);
    float time = 0.0f;
    float distance_value = 0.0f;
    float3 barycentric = float3(0.0f, 0.0f, 0.0f);
    float3 normal = float3(0.0f, 0.0f, 1.0f);
    bool hit = false;
    uint iterations = 0;

    [loop]
    for (uint iteration = 0; iteration < maximum_iterations; ++iteration)
    {
        iterations = iteration + 1;
        float3 position_value = start_position + velocity * time;
        float3 closest = closest_point_triangle(
            position_value, triangle0, triangle1, triangle2, barycentric);
        float3 separation = position_value - closest;
        distance_value = length(separation);
        float gap = distance_value - combined_thickness;
        if (gap <= distance_tolerance)
        {
            hit = true;
            if (distance_value > 1.0e-10f)
            {
                normal = separation / distance_value;
            }
            break;
        }
        if (speed <= 1.0e-12f)
        {
            break;
        }
        float advance = safety_factor * gap / speed;
        if (advance <= 1.0e-9f)
        {
            break;
        }
        time += advance;
        if (time > 1.0f)
        {
            break;
        }
    }

    float interval_seconds = swept_geometry[0].w;
    float toi_seconds = hit ? time * interval_seconds : -1.0f;
    ccd_output[0] = float4(toi_seconds, hit ? 1.0f : 0.0f,
        (float)iterations, distance_value);
    ccd_output[1] = float4(barycentric, 0.0f);
    ccd_output[2] = float4(normal, 0.0f);
}
