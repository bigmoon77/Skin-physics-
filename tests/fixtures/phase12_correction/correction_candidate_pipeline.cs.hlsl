struct CorrectionAabb
{
    float4 minimum;
    float4 maximum;
};

struct CorrectionCandidatePrimitive
{
    uint stable_id_low;
    uint stable_id_high;
    uint mesh_id;
    uint primitive_id;
    uint role;
    uint vertex_count;
    uint4 vertex_id_low;
    uint4 vertex_id_high;
};

struct CorrectionCandidatePair
{
    uint4 identity_a;
    uint4 identity_b;
    uint4 metadata;
};

StructuredBuffer<CorrectionCandidatePrimitive> correction_primitives : register(t0);
StructuredBuffer<CorrectionAabb> correction_bounds : register(t1);
StructuredBuffer<uint> correction_mesh_pair_filter : register(t2);
StructuredBuffer<uint2> correction_candidate_pair_seeds : register(t3);
StructuredBuffer<uint2> correction_excluded_pairs : register(t4);
StructuredBuffer<uint> correction_candidate_flags_srv : register(t5);
StructuredBuffer<uint> correction_candidate_prefix_srv : register(t6);

RWStructuredBuffer<uint> correction_candidate_flags : register(u0);
RWStructuredBuffer<uint> correction_candidate_prefix : register(u1);
RWStructuredBuffer<CorrectionCandidatePair> correction_candidate_output : register(u2);
RWStructuredBuffer<uint> correction_candidate_totals : register(u3);

cbuffer CandidateParameters : register(b0)
{
    uint primitive_count;
    uint mesh_count;
    uint pair_count;
    uint candidate_capacity;

    uint sort_capacity;
    uint exclusion_count;
    uint self_collision_enabled;
    uint reserved0;
};

cbuffer SortParameters : register(b1)
{
    uint sort_k;
    uint sort_j;
    uint sort_reserved0;
    uint sort_reserved1;
};

CorrectionCandidatePair correction_sentinel_pair()
{
    CorrectionCandidatePair value;
    value.identity_a = 0xffffffffu;
    value.identity_b = 0xffffffffu;
    value.metadata = 0xffffffffu;
    return value;
}

bool correction_identity_less(uint4 left, uint4 right)
{
    if (left.y != right.y) return left.y < right.y;
    if (left.x != right.x) return left.x < right.x;
    if (left.z != right.z) return left.z < right.z;
    return left.w < right.w;
}

bool correction_pair_less(
    CorrectionCandidatePair left,
    CorrectionCandidatePair right)
{
    if (any(left.identity_a != right.identity_a))
    {
        return correction_identity_less(left.identity_a, right.identity_a);
    }
    if (any(left.identity_b != right.identity_b))
    {
        return correction_identity_less(left.identity_b, right.identity_b);
    }
    return left.metadata.x < right.metadata.x;
}

bool correction_pair_equal(
    CorrectionCandidatePair left,
    CorrectionCandidatePair right)
{
    return all(left.identity_a == right.identity_a) &&
        all(left.identity_b == right.identity_b) &&
        left.metadata.x == right.metadata.x;
}

bool correction_aabb_overlap(CorrectionAabb left, CorrectionAabb right)
{
    return
        left.minimum.x <= right.maximum.x &&
        left.maximum.x >= right.minimum.x &&
        left.minimum.y <= right.maximum.y &&
        left.maximum.y >= right.minimum.y &&
        left.minimum.z <= right.maximum.z &&
        left.maximum.z >= right.minimum.z;
}

bool correction_primitive_valid(CorrectionCandidatePrimitive primitive)
{
    bool valid_role =
        (primitive.role == 0u && primitive.vertex_count == 1u) ||
        (primitive.role == 1u && primitive.vertex_count == 3u) ||
        (primitive.role == 2u && primitive.vertex_count == 2u);
    if (!valid_role)
    {
        return false;
    }

    [unroll]
    for (uint first = 0u; first < 3u; ++first)
    {
        if (first >= primitive.vertex_count) break;
        [unroll]
        for (uint second = first + 1u; second < 3u; ++second)
        {
            if (second >= primitive.vertex_count) break;
            if (primitive.vertex_id_low[first] ==
                    primitive.vertex_id_low[second] &&
                primitive.vertex_id_high[first] ==
                    primitive.vertex_id_high[second])
            {
                return false;
            }
        }
    }
    return true;
}

bool correction_primitive_identity_equal(
    CorrectionCandidatePrimitive left,
    CorrectionCandidatePrimitive right)
{
    return left.stable_id_low == right.stable_id_low &&
        left.stable_id_high == right.stable_id_high &&
        left.mesh_id == right.mesh_id &&
        left.primitive_id == right.primitive_id;
}

bool correction_has_shared_vertex(
    CorrectionCandidatePrimitive left,
    CorrectionCandidatePrimitive right)
{
    [unroll]
    for (uint left_index = 0; left_index < 3; ++left_index)
    {
        if (left_index >= left.vertex_count) break;
        [unroll]
        for (uint right_index = 0; right_index < 3; ++right_index)
        {
            if (right_index >= right.vertex_count) break;
            if (left.vertex_id_low[left_index] == right.vertex_id_low[right_index] &&
                left.vertex_id_high[left_index] == right.vertex_id_high[right_index])
            {
                return true;
            }
        }
    }
    return false;
}

bool correction_is_excluded(uint primitive_a, uint primitive_b)
{
    [loop]
    for (uint index = 0; index < exclusion_count; ++index)
    {
        uint2 pair = correction_excluded_pairs[index];
        if ((pair.x == primitive_a && pair.y == primitive_b) ||
            (pair.x == primitive_b && pair.y == primitive_a))
        {
            return true;
        }
    }
    return false;
}

bool correction_mesh_pair_enabled(
    CorrectionCandidatePrimitive left,
    CorrectionCandidatePrimitive right)
{
    if (left.mesh_id == right.mesh_id)
    {
        return self_collision_enabled != 0;
    }
    if (left.mesh_id >= mesh_count || right.mesh_id >= mesh_count)
    {
        return false;
    }
    return correction_mesh_pair_filter[left.mesh_id * mesh_count + right.mesh_id] != 0;
}

bool correction_pair_type(
    CorrectionCandidatePrimitive left,
    CorrectionCandidatePrimitive right,
    out uint contact_type)
{
    contact_type = 0xffffffffu;
    bool vertex_triangle =
        (left.role == 0u && right.role == 1u) ||
        (left.role == 1u && right.role == 0u);
    if (vertex_triangle)
    {
        contact_type = 0u;
        return true;
    }
    if (left.role == 2u && right.role == 2u)
    {
        contact_type = 1u;
        return true;
    }
    return false;
}

CorrectionCandidatePair correction_make_pair(
    CorrectionCandidatePrimitive left,
    CorrectionCandidatePrimitive right,
    uint contact_type)
{
    uint4 identity_left = uint4(
        left.stable_id_low, left.stable_id_high,
        left.mesh_id, left.primitive_id);
    uint4 identity_right = uint4(
        right.stable_id_low, right.stable_id_high,
        right.mesh_id, right.primitive_id);

    CorrectionCandidatePair result;
    if (correction_identity_less(identity_right, identity_left))
    {
        result.identity_a = identity_right;
        result.identity_b = identity_left;
    }
    else
    {
        result.identity_a = identity_left;
        result.identity_b = identity_right;
    }
    result.metadata = uint4(contact_type, 0u, 0u, 0u);
    return result;
}

[numthreads(64, 1, 1)]
void correction_candidate_validate_primitives(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint primitive_index = dispatch_id.x;
    if (primitive_index >= primitive_count)
    {
        return;
    }

    CorrectionCandidatePrimitive primitive =
        correction_primitives[primitive_index];
    CorrectionAabb bounds = correction_bounds[primitive_index];
    bool valid_bounds =
        all(isfinite(bounds.minimum.xyz)) &&
        all(isfinite(bounds.maximum.xyz)) &&
        all(bounds.minimum.xyz <= bounds.maximum.xyz);
    if (!correction_primitive_valid(primitive) ||
        primitive.mesh_id >= mesh_count ||
        !valid_bounds)
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 2u, previous);
        return;
    }

    [loop]
    for (uint previous_index = 0u;
        previous_index < primitive_index; ++previous_index)
    {
        CorrectionCandidatePrimitive previous_primitive =
            correction_primitives[previous_index];
        if (correction_primitive_identity_equal(
                primitive, previous_primitive))
        {
            uint previous = 0u;
            InterlockedOr(correction_candidate_totals[1], 2u, previous);
            return;
        }
    }
}

[numthreads(64, 1, 1)]
void correction_candidate_count(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint pair_index = dispatch_id.x;
    if (pair_index >= pair_count) return;
    correction_candidate_flags[pair_index] = 0u;

    if (primitive_count == 0u)
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 2u, previous);
        return;
    }

    uint2 seed_pair = correction_candidate_pair_seeds[pair_index];
    uint primitive_a_index = seed_pair.x;
    uint primitive_b_index = seed_pair.y;
    if (primitive_a_index >= primitive_count ||
        primitive_b_index >= primitive_count ||
        primitive_a_index >= primitive_b_index)
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 2u, previous);
        return;
    }

    [loop]
    for (uint previous_pair_index = 0u;
        previous_pair_index < pair_index; ++previous_pair_index)
    {
        uint2 previous_pair =
            correction_candidate_pair_seeds[previous_pair_index];
        if (previous_pair.x == primitive_a_index &&
            previous_pair.y == primitive_b_index)
        {
            return;
        }
    }

    CorrectionCandidatePrimitive primitive_a =
        correction_primitives[primitive_a_index];
    CorrectionCandidatePrimitive primitive_b =
        correction_primitives[primitive_b_index];
    if (!correction_primitive_valid(primitive_a) ||
        !correction_primitive_valid(primitive_b))
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 2u, previous);
        return;
    }
    if (correction_primitive_identity_equal(primitive_a, primitive_b))
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 2u, previous);
        return;
    }

    uint contact_type = 0xffffffffu;
    if (!correction_pair_type(primitive_a, primitive_b, contact_type))
    {
        return;
    }
    if (!correction_mesh_pair_enabled(primitive_a, primitive_b))
    {
        return;
    }
    if (correction_has_shared_vertex(primitive_a, primitive_b) ||
        correction_is_excluded(primitive_a_index, primitive_b_index))
    {
        return;
    }

    CorrectionAabb bounds_a = correction_bounds[primitive_a_index];
    CorrectionAabb bounds_b = correction_bounds[primitive_b_index];
    if (!all(isfinite(bounds_a.minimum.xyz)) ||
        !all(isfinite(bounds_a.maximum.xyz)) ||
        !all(isfinite(bounds_b.minimum.xyz)) ||
        !all(isfinite(bounds_b.maximum.xyz)) ||
        any(bounds_a.minimum.xyz > bounds_a.maximum.xyz) ||
        any(bounds_b.minimum.xyz > bounds_b.maximum.xyz))
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 2u, previous);
        return;
    }

    if (correction_aabb_overlap(bounds_a, bounds_b))
    {
        correction_candidate_flags[pair_index] = 1u;
    }
}

[numthreads(1, 1, 1)]
void correction_candidate_exclusive_scan(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u) return;
    uint prefix = 0u;
    [loop]
    for (uint pair_index = 0u; pair_index < pair_count; ++pair_index)
    {
        correction_candidate_prefix[pair_index] = prefix;
        prefix += correction_candidate_flags_srv[pair_index];
    }
    correction_candidate_totals[0] = prefix;
    if (prefix > candidate_capacity)
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 1u, previous);
    }
}

[numthreads(64, 1, 1)]
void correction_candidate_initialize(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint output_index = dispatch_id.x;
    if (output_index >= sort_capacity) return;
    correction_candidate_output[output_index] = correction_sentinel_pair();
}

[numthreads(64, 1, 1)]
void correction_candidate_fill(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (correction_candidate_totals[1] != 0u) return;
    uint pair_index = dispatch_id.x;
    if (pair_index >= pair_count ||
        correction_candidate_flags_srv[pair_index] == 0u)
    {
        return;
    }

    uint2 seed_pair = correction_candidate_pair_seeds[pair_index];
    uint primitive_a_index = seed_pair.x;
    uint primitive_b_index = seed_pair.y;
    if (primitive_a_index >= primitive_count ||
        primitive_b_index >= primitive_count ||
        primitive_a_index >= primitive_b_index)
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 2u, previous);
        return;
    }
    CorrectionCandidatePrimitive primitive_a =
        correction_primitives[primitive_a_index];
    CorrectionCandidatePrimitive primitive_b =
        correction_primitives[primitive_b_index];
    uint contact_type = 0xffffffffu;
    if (!correction_pair_type(primitive_a, primitive_b, contact_type))
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 2u, previous);
        return;
    }
    uint output_index = correction_candidate_prefix_srv[pair_index];
    if (output_index >= candidate_capacity || output_index >= sort_capacity)
    {
        uint previous = 0u;
        InterlockedOr(correction_candidate_totals[1], 1u, previous);
        return;
    }
    correction_candidate_output[output_index] =
        correction_make_pair(primitive_a, primitive_b, contact_type);
}

[numthreads(64, 1, 1)]
void correction_candidate_bitonic_sort(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (correction_candidate_totals[1] != 0u) return;
    uint index = dispatch_id.x;
    if (index >= sort_capacity) return;
    uint partner = index ^ sort_j;
    if (partner <= index || partner >= sort_capacity) return;

    CorrectionCandidatePair current_value = correction_candidate_output[index];
    CorrectionCandidatePair partner_value = correction_candidate_output[partner];
    bool current_less = correction_pair_less(current_value, partner_value);
    bool ascending = (index & sort_k) == 0u;
    bool should_swap = ascending ? !current_less : current_less;
    if (should_swap)
    {
        correction_candidate_output[index] = partner_value;
        correction_candidate_output[partner] = current_value;
    }
}

[numthreads(1, 1, 1)]
void correction_candidate_deduplicate(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u || correction_candidate_totals[1] != 0u)
    {
        return;
    }

    uint count = correction_candidate_totals[0];
    uint unique_count = 0u;
    [loop]
    for (uint index = 0u; index < count; ++index)
    {
        CorrectionCandidatePair current_value =
            correction_candidate_output[index];
        if (unique_count == 0u ||
            !correction_pair_equal(
                current_value, correction_candidate_output[index - 1u]))
        {
            correction_candidate_output[unique_count] = current_value;
            ++unique_count;
        }
    }
    correction_candidate_totals[0] = unique_count;
}
