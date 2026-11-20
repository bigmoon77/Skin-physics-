struct RenderBindingInput
{
    uint4 physics_vertices;
    float4 barycentric;
    float4 rest_offset_local;
    uint4 metadata;
};

struct RenderVertexInput
{
    float4 texcoord;
    float4 fallback_normal;
    float4 fallback_tangent;
    uint4 metadata;
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

StructuredBuffer<float4> physics_positions : register(t0);
StructuredBuffer<RenderBindingInput> render_bindings : register(t1);
StructuredBuffer<RenderVertexInput> render_vertices : register(t2);
StructuredBuffer<uint> render_indices : register(t3);
StructuredBuffer<uint> vertex_triangle_offsets : register(t4);
StructuredBuffer<uint> vertex_triangle_ids : register(t5);

RWStructuredBuffer<float4> candidate_positions : register(u0);
RWStructuredBuffer<float4> candidate_normals : register(u1);
RWStructuredBuffer<float4> candidate_tangents : register(u2);
RWStructuredBuffer<TickCommitControl> tick_commit_control_buffer : register(u3);

cbuffer RenderFeedbackParameters : register(b0)
{
    uint render_vertex_count;
    uint render_index_count;
    uint2 reserved;
};

static const uint ATTRIBUTE_NORMAL = 1u;
static const uint ATTRIBUTE_TANGENT = 2u;
static const uint BINDING_TETRAHEDRON = 0u;
static const uint BINDING_BOUNDARY_TRIANGLE = 1u;
static const uint TICK_GATE_READY = 1u;
static const uint TICK_TERMINAL_IN_FLIGHT = 1u;

float3 safe_normalize(float3 value, float3 fallback_value)
{
    float squared_length = dot(value, value);
    if (!isfinite(squared_length) || squared_length <= 1.0e-20f)
    {
        float fallback_squared = dot(fallback_value, fallback_value);
        if (!isfinite(fallback_squared) || fallback_squared <= 1.0e-20f)
        {
            return float3(0.0f, 1.0f, 0.0f);
        }
        return fallback_value * rsqrt(fallback_squared);
    }
    return value * rsqrt(squared_length);
}

bool evaluate_render_position(uint render_vertex_index, out float3 position)
{
    position = 0.0f;
    RenderBindingInput binding = render_bindings[render_vertex_index];

    if (binding.metadata.x == BINDING_TETRAHEDRON)
    {
        [unroll]
        for (uint local_index = 0u; local_index < 4u; ++local_index)
        {
            float4 physics_position = physics_positions[binding.physics_vertices[local_index]];
            if (!all(isfinite(physics_position.xyz)) ||
                !isfinite(binding.barycentric[local_index]))
            {
                return false;
            }
            position += physics_position.xyz * binding.barycentric[local_index];
        }
    }
    else if (binding.metadata.x == BINDING_BOUNDARY_TRIANGLE)
    {
        uint ia = binding.physics_vertices.x;
        uint ib = binding.physics_vertices.y;
        uint ic = binding.physics_vertices.z;
        float3 p0 = physics_positions[ia].xyz;
        float3 p1 = physics_positions[ib].xyz;
        float3 p2 = physics_positions[ic].xyz;
        if (!all(isfinite(p0)) || !all(isfinite(p1)) || !all(isfinite(p2)) ||
            !all(isfinite(binding.barycentric.xyz)) ||
            !all(isfinite(binding.rest_offset_local.xyz))
        {
            return false;
        }

        float3 edge1 = p1 - p0;
        float3 edge2 = p2 - p0;
        float3 cross_value = cross(edge1, edge2);
        float cross_squared = dot(cross_value, cross_value);
        if (!isfinite(cross_squared) || cross_squared <= 1.0e-20f)
        {
            return false;
        }
        float3 normal = cross_value * rsqrt(cross_squared);
        float3 closest =
            p0 * binding.barycentric.x +
            p1 * binding.barycentric.y +
            p2 * binding.barycentric.z;
        position = closest +
            edge1 * binding.rest_offset_local.x +
            edge2 * binding.rest_offset_local.y +
            normal * binding.rest_offset_local.z;
    }
    else
    {
        return false;
    }

    return all(isfinite(position));
}

[numthreads(64, 1, 1)]
void prepare_render_candidate(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint render_vertex_index = dispatch_id.x;
    if (render_vertex_index >= render_vertex_count)
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

    float3 position = 0.0f;
    if (!evaluate_render_position(render_vertex_index, position))
    {
        uint previous_failure = 0u;
        InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
        return;
    }

    RenderVertexInput vertex = render_vertices[render_vertex_index];
    float3 normal = vertex.fallback_normal.xyz;
    float4 tangent = vertex.fallback_tangent;

    uint begin_index = vertex_triangle_offsets[render_vertex_index];
    uint end_index = vertex_triangle_offsets[render_vertex_index + 1u];
    if (end_index > render_index_count / 3u || begin_index > end_index)
    {
        uint previous_failure = 0u;
        InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
        return;
    }

    float3 normal_sum = 0.0f;
    float3 tangent_sum = 0.0f;
    float3 bitangent_sum = 0.0f;

    [loop]
    for (uint incidence = begin_index; incidence < end_index; ++incidence)
    {
        uint triangle_index = vertex_triangle_ids[incidence];
        uint index_offset = triangle_index * 3u;
        if (index_offset + 2u >= render_index_count)
        {
            uint previous_failure = 0u;
            InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
            return;
        }

        uint i0 = render_indices[index_offset];
        uint i1 = render_indices[index_offset + 1u];
        uint i2 = render_indices[index_offset + 2u];
        if (i0 >= render_vertex_count || i1 >= render_vertex_count ||
            i2 >= render_vertex_count ||
            (i0 != render_vertex_index && i1 != render_vertex_index &&
                i2 != render_vertex_index))
        {
            uint previous_failure = 0u;
            InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
            return;
        }

        float3 p0 = 0.0f;
        float3 p1 = 0.0f;
        float3 p2 = 0.0f;
        if (!evaluate_render_position(i0, p0) ||
            !evaluate_render_position(i1, p1) ||
            !evaluate_render_position(i2, p2))
        {
            uint previous_failure = 0u;
            InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
            return;
        }

        float3 edge1 = p1 - p0;
        float3 edge2 = p2 - p0;
        float3 face_normal = cross(edge1, edge2);
        float face_normal_squared = dot(face_normal, face_normal);
        if (isfinite(face_normal_squared) && face_normal_squared > 1.0e-20f)
        {
            normal_sum += face_normal;
        }

        RenderVertexInput v0 = render_vertices[i0];
        RenderVertexInput v1 = render_vertices[i1];
        RenderVertexInput v2 = render_vertices[i2];
        if (((v0.metadata.x | v1.metadata.x | v2.metadata.x) &
            ATTRIBUTE_TANGENT) != 0u)
        {
            float2 delta_uv1 = v1.texcoord.xy - v0.texcoord.xy;
            float2 delta_uv2 = v2.texcoord.xy - v0.texcoord.xy;
            float determinant_uv =
                delta_uv1.x * delta_uv2.y - delta_uv1.y * delta_uv2.x;
            if (isfinite(determinant_uv) && abs(determinant_uv) > 1.0e-12f)
            {
                float inverse_uv = 1.0f / determinant_uv;
                tangent_sum += (edge1 * delta_uv2.y - edge2 * delta_uv1.y) * inverse_uv;
                bitangent_sum += (edge2 * delta_uv1.x - edge1 * delta_uv2.x) * inverse_uv;
            }
        }
    }

    if ((vertex.metadata.x & ATTRIBUTE_NORMAL) != 0u)
    {
        normal = safe_normalize(normal_sum, vertex.fallback_normal.xyz);
    }

    if ((vertex.metadata.x & ATTRIBUTE_TANGENT) != 0u)
    {
        normal = safe_normalize(normal, vertex.fallback_normal.xyz);
        float3 projected_tangent = tangent_sum -
            normal * dot(normal, tangent_sum);
        float projected_squared = dot(projected_tangent, projected_tangent);
        if (!isfinite(projected_squared) || projected_squared <= 1.0e-20f)
        {
            float3 axis = abs(normal.z) < 0.9f
                ? float3(0.0f, 0.0f, 1.0f)
                : float3(0.0f, 1.0f, 0.0f);
            projected_tangent = cross(axis, normal);
        }
        float3 tangent_direction = safe_normalize(
            projected_tangent, float3(1.0f, 0.0f, 0.0f));
        float handedness =
            dot(cross(normal, tangent_direction), bitangent_sum) < 0.0f
                ? -1.0f : 1.0f;
        tangent = float4(tangent_direction, handedness);
    }

    if (!all(isfinite(position)) || !all(isfinite(normal)) ||
        !all(isfinite(tangent)))
    {
        uint previous_failure = 0u;
        InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
        return;
    }

    candidate_positions[render_vertex_index] = float4(position, 1.0f);
    candidate_normals[render_vertex_index] = float4(normal, 0.0f);
    candidate_tangents[render_vertex_index] = tangent;
}

[numthreads(64, 1, 1)]
void validate_render_candidate(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint render_vertex_index = dispatch_id.x;
    if (render_vertex_index >= render_vertex_count)
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

    float4 position = candidate_positions[render_vertex_index];
    float4 normal = candidate_normals[render_vertex_index];
    float4 tangent = candidate_tangents[render_vertex_index];
    if (!all(isfinite(position)) || !all(isfinite(normal)) ||
        !all(isfinite(tangent)))
    {
        uint previous_failure = 0u;
        InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
    }
}
