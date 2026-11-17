struct VtResult
{
    float4 corrected0;
    float4 corrected1;
    float4 corrected2;
    float4 corrected3;
    float4 barycentric_penetration;
    float4 normal_hit;
    float4 metrics;
};

struct EeResult
{
    float4 closest_a;
    float4 closest_b;
    float4 parameters;
    float4 normal_hit;
};

struct Aabb
{
    float4 minimum;
    float4 maximum;
};

struct BvhNode
{
    float4 minimum;
    float4 maximum;
    uint left;
    uint right;
    uint primitive_id;
    uint is_leaf;
};

StructuredBuffer<float4> contact_positions : register(t0);
RWStructuredBuffer<VtResult> vt_results : register(u0);
RWStructuredBuffer<EeResult> ee_results : register(u1);

StructuredBuffer<BvhNode> bvh_nodes : register(t2);
StructuredBuffer<Aabb> primitive_bounds : register(t3);
StructuredBuffer<uint> primitive_mesh_ids : register(t4);
StructuredBuffer<uint> mesh_pair_filter : register(t5);
RWStructuredBuffer<uint2> candidate_pairs : register(u2);
RWStructuredBuffer<uint> broadphase_counters : register(u3);

cbuffer Parameters : register(b0)
{
    float contact_radius;
    float relaxation;
    float combined_thickness;
    uint primitive_count;

    uint node_count;
    uint mesh_count;
    uint candidate_capacity;
    uint self_collision_enabled;

    float expansion;
    uint3 padding;
};

bool finite3(float3 value)
{
    return all(isfinite(value));
}

float3 closest_point_triangle(
    float3 point,
    float3 a,
    float3 b,
    float3 c,
    out float3 barycentric)
{
    float3 ab = b - a;
    float3 ac = c - a;
    float3 ap = point - a;
    float d1 = dot(ab, ap);
    float d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        barycentric = float3(1.0f, 0.0f, 0.0f);
        return a;
    }

    float3 bp = point - b;
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

    float3 cp = point - c;
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
void vertex_triangle_contact(uint3 dispatch_id : SV_DispatchThreadID)
{
    float3 point = contact_positions[0].xyz;
    float3 triangle0 = contact_positions[1].xyz;
    float3 triangle1 = contact_positions[2].xyz;
    float3 triangle2 = contact_positions[3].xyz;

    float3 barycentric;
    float3 closest = closest_point_triangle(
        point, triangle0, triangle1, triangle2, barycentric);
    float3 separation = point - closest;
    float distance_value = length(separation);
    float3 normal = distance_value > 1.0e-10f
        ? separation / distance_value
        : normalize(cross(triangle1 - triangle0, triangle2 - triangle0));
    float penetration = max(0.0f, contact_radius - distance_value);
    float lambda = 0.0f;

    float3 corrected0 = point;
    float3 corrected1 = triangle0;
    float3 corrected2 = triangle1;
    float3 corrected3 = triangle2;
    bool valid = finite3(point) && finite3(closest) &&
        finite3(normal) && isfinite(distance_value);

    if (valid && penetration > 0.0f)
    {
        float weight0 = contact_positions[0].w;
        float weight1 = contact_positions[1].w;
        float weight2 = contact_positions[2].w;
        float weight3 = contact_positions[3].w;
        float denominator = weight0 +
            weight1 * barycentric.x * barycentric.x +
            weight2 * barycentric.y * barycentric.y +
            weight3 * barycentric.z * barycentric.z;
        if (denominator > 1.0e-20f)
        {
            lambda = relaxation * penetration / denominator;
            corrected0 += normal * (weight0 * lambda);
            corrected1 -= normal * (weight1 * barycentric.x * lambda);
            corrected2 -= normal * (weight2 * barycentric.y * lambda);
            corrected3 -= normal * (weight3 * barycentric.z * lambda);
        }
    }

    float3 corrected_barycentric;
    float3 corrected_closest = closest_point_triangle(
        corrected0, corrected1, corrected2, corrected3, corrected_barycentric);
    float post_distance = length(corrected0 - corrected_closest);
    float post_penetration = max(0.0f, contact_radius - post_distance);

    VtResult output;
    output.corrected0 = float4(corrected0, 0.0f);
    output.corrected1 = float4(corrected1, 0.0f);
    output.corrected2 = float4(corrected2, 0.0f);
    output.corrected3 = float4(corrected3, 0.0f);
    output.barycentric_penetration = float4(barycentric, penetration);
    output.normal_hit = float4(normal, penetration > 0.0f ? 1.0f : 0.0f);
    output.metrics = float4(penetration, post_penetration, lambda, valid ? 1.0f : 0.0f);
    vt_results[0] = output;
}

[numthreads(1, 1, 1)]
void edge_edge_contact(uint3 dispatch_id : SV_DispatchThreadID)
{
    float3 a0 = contact_positions[0].xyz;
    float3 a1 = contact_positions[1].xyz;
    float3 b0 = contact_positions[2].xyz;
    float3 b1 = contact_positions[3].xyz;
    float3 d1 = a1 - a0;
    float3 d2 = b1 - b0;
    float3 r = a0 - b0;
    float a = dot(d1, d1);
    float e = dot(d2, d2);
    float f = dot(d2, r);
    float s = 0.0f;
    float t = 0.0f;

    if (a <= 1.0e-20f && e <= 1.0e-20f)
    {
        s = 0.0f;
        t = 0.0f;
    }
    else if (a <= 1.0e-20f)
    {
        s = 0.0f;
        t = saturate(f / e);
    }
    else
    {
        float c = dot(d1, r);
        if (e <= 1.0e-20f)
        {
            t = 0.0f;
            s = saturate(-c / a);
        }
        else
        {
            float b = dot(d1, d2);
            float denominator = a * e - b * b;
            if (abs(denominator) > 1.0e-20f)
            {
                s = saturate((b * f - c * e) / denominator);
            }
            t = (b * s + f) / e;
            if (t < 0.0f)
            {
                t = 0.0f;
                s = saturate(-c / a);
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
                s = saturate((b - c) / a);
            }
        }
    }

    float3 closest_a = a0 + d1 * s;
    float3 closest_b = b0 + d2 * t;
    float3 separation = closest_a - closest_b;
    float distance_value = length(separation);
    float3 normal = distance_value > 1.0e-10f
        ? separation / distance_value
        : normalize(cross(d1, d2));
    if (dot(normal, normal) <= 1.0e-20f)
    {
        normal = float3(0.0f, 0.0f, 1.0f);
    }
    float penetration = max(0.0f, combined_thickness - distance_value);

    EeResult output;
    output.closest_a = float4(closest_a, 0.0f);
    output.closest_b = float4(closest_b, 0.0f);
    output.parameters = float4(s, t, distance_value, penetration);
    output.normal_hit = float4(normal, penetration > 0.0f ? 1.0f : 0.0f);
    ee_results[0] = output;
}

bool aabb_overlaps(Aabb a, BvhNode b, float margin)
{
    return
        a.minimum.x - margin <= b.maximum.x + margin &&
        a.maximum.x + margin >= b.minimum.x - margin &&
        a.minimum.y - margin <= b.maximum.y + margin &&
        a.maximum.y + margin >= b.minimum.y - margin &&
        a.minimum.z - margin <= b.maximum.z + margin &&
        a.maximum.z + margin >= b.minimum.z - margin;
}

[numthreads(64, 1, 1)]
void bvh_broadphase(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint query_id = dispatch_id.x;
    if (query_id >= primitive_count || node_count == 0)
    {
        return;
    }

    Aabb query_bounds = primitive_bounds[query_id];
    uint stack[32];
    uint stack_size = 0;
    stack[stack_size++] = 0;

    while (stack_size > 0)
    {
        uint node_id = stack[--stack_size];
        if (node_id >= node_count)
        {
            continue;
        }
        BvhNode node = bvh_nodes[node_id];
        if (!aabb_overlaps(query_bounds, node, expansion))
        {
            continue;
        }

        if (node.is_leaf != 0)
        {
            uint candidate_id = node.primitive_id;
            if (candidate_id <= query_id || candidate_id >= primitive_count)
            {
                continue;
            }
            uint mesh_a = primitive_mesh_ids[query_id];
            uint mesh_b = primitive_mesh_ids[candidate_id];
            if (mesh_a >= mesh_count || mesh_b >= mesh_count)
            {
                continue;
            }
            bool enabled = mesh_a == mesh_b
                ? self_collision_enabled != 0
                : mesh_pair_filter[mesh_a * mesh_count + mesh_b] != 0;
            if (!enabled)
            {
                continue;
            }

            uint output_index = 0;
            InterlockedAdd(broadphase_counters[0], 1, output_index);
            if (output_index < candidate_capacity)
            {
                candidate_pairs[output_index] = uint2(query_id, candidate_id);
            }
            else
            {
                uint previous = 0;
                InterlockedOr(broadphase_counters[1], 1, previous);
            }
        }
        else
        {
            if (stack_size + 2 <= 32)
            {
                stack[stack_size++] = node.right;
                stack[stack_size++] = node.left;
            }
            else
            {
                uint previous = 0;
                InterlockedOr(broadphase_counters[1], 2, previous);
            }
        }
    }
}
