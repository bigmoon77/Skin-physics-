struct ClothStretchConstraint
{
    uint2 vertices;
    float rest_length;
    float reserved;
};

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

struct ClothVertexConstraintRef
{
    uint type;
    uint constraint_index;
    uint local_vertex_index;
    uint reserved;
};

cbuffer ClothJacobiGatherParameters : register(b0)
{
    uint vertex_count;
    uint stretch_constraint_count;
    uint shear_constraint_count;
    uint bend_constraint_count;
    uint reference_count;
    uint reserved0;
    uint reserved1;
    uint reserved2;
};

StructuredBuffer<float4> current_positions : register(t0);
StructuredBuffer<uint> vertex_constraint_offsets : register(t1);
StructuredBuffer<ClothVertexConstraintRef> vertex_constraint_refs : register(t2);
StructuredBuffer<ClothStretchConstraint> stretch_constraints : register(t3);
StructuredBuffer<ClothConstraint> shear_constraints : register(t4);
StructuredBuffer<ClothConstraint> bend_constraints : register(t5);
StructuredBuffer<ClothContribution> shear_contributions : register(t6);
StructuredBuffer<ClothContribution> bend_contributions : register(t7);
RWStructuredBuffer<float4> next_positions : register(u0);
RWStructuredBuffer<uint> solver_status : register(u1);

static const uint CLOTH_CONSTRAINT_STRETCH = 0u;
static const uint CLOTH_CONSTRAINT_SHEAR = 1u;
static const uint CLOTH_CONSTRAINT_BEND = 2u;
static const uint STATUS_INVALID_CLOTH_CSR = 1u << 2u;

void fail_cloth_csr()
{
    uint ignored = 0u;
    InterlockedOr(solver_status[0], STATUS_INVALID_CLOTH_CSR, ignored);
}

bool finite3(float3 value)
{
    return all(isfinite(value));
}

float3 get_contribution(
    ClothContribution contribution,
    uint local_vertex_index)
{
    if (local_vertex_index == 0u) return contribution.correction0.xyz;
    if (local_vertex_index == 1u) return contribution.correction1.xyz;
    if (local_vertex_index == 2u) return contribution.correction2.xyz;
    return contribution.correction3.xyz;
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint vertex_index = dispatch_thread_id.x;
    if (vertex_index >= vertex_count)
    {
        return;
    }

    uint position_count = 0u;
    uint position_stride = 0u;
    uint offset_count = 0u;
    uint offset_stride = 0u;
    uint reference_buffer_count = 0u;
    uint reference_stride = 0u;
    uint stretch_constraint_buffer_count = 0u;
    uint stretch_constraint_stride = 0u;
    uint shear_constraint_buffer_count = 0u;
    uint shear_constraint_stride = 0u;
    uint bend_constraint_buffer_count = 0u;
    uint bend_constraint_stride = 0u;
    uint shear_contribution_count = 0u;
    uint shear_contribution_stride = 0u;
    uint bend_contribution_count = 0u;
    uint bend_contribution_stride = 0u;
    uint output_count = 0u;
    uint output_stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    current_positions.GetDimensions(position_count, position_stride);
    vertex_constraint_offsets.GetDimensions(offset_count, offset_stride);
    vertex_constraint_refs.GetDimensions(reference_buffer_count, reference_stride);
    stretch_constraints.GetDimensions(
        stretch_constraint_buffer_count, stretch_constraint_stride);
    shear_constraints.GetDimensions(
        shear_constraint_buffer_count, shear_constraint_stride);
    bend_constraints.GetDimensions(
        bend_constraint_buffer_count, bend_constraint_stride);
    shear_contributions.GetDimensions(
        shear_contribution_count, shear_contribution_stride);
    bend_contributions.GetDimensions(
        bend_contribution_count, bend_contribution_stride);
    next_positions.GetDimensions(output_count, output_stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (reserved0 != 0u || reserved1 != 0u || reserved2 != 0u ||
        vertex_count == 0u || vertex_count == 0xffffffffu ||
        vertex_count != position_count ||
        vertex_count != output_count || position_stride != 16u ||
        output_stride != 16u || offset_count != vertex_count + 1u ||
        offset_stride != 4u || reference_count > reference_buffer_count ||
        reference_stride != 16u ||
        stretch_constraint_count > stretch_constraint_buffer_count ||
        shear_constraint_count > shear_constraint_buffer_count ||
        bend_constraint_count > bend_constraint_buffer_count ||
        stretch_constraint_stride != 16u ||
        shear_constraint_stride != 32u || bend_constraint_stride != 32u ||
        shear_contribution_count < shear_constraint_count ||
        bend_contribution_count < bend_constraint_count ||
        shear_contribution_stride != 64u ||
        bend_contribution_stride != 64u ||
        status_count != 1u || status_stride != 4u)
    {
        fail_cloth_csr();
        next_positions[vertex_index] = current_positions[vertex_index];
        return;
    }

    const uint first_reference = vertex_constraint_offsets[vertex_index];
    const uint end_reference = vertex_constraint_offsets[vertex_index + 1u];
    if (first_reference > end_reference ||
        end_reference > reference_count ||
        (vertex_index == 0u && first_reference != 0u) ||
        (vertex_index + 1u == vertex_count &&
            end_reference != reference_count))
    {
        fail_cloth_csr();
        next_positions[vertex_index] = current_positions[vertex_index];
        return;
    }

    const float4 source_position = current_positions[vertex_index];
    if (!all(isfinite(source_position)))
    {
        fail_cloth_csr();
        next_positions[vertex_index] = source_position;
        return;
    }

    float3 total_correction = 0.0f;
    for (uint reference_index = first_reference;
        reference_index < end_reference; ++reference_index)
    {
        const ClothVertexConstraintRef reference =
            vertex_constraint_refs[reference_index];
        if (reference.reserved != 0u)
        {
            fail_cloth_csr();
            next_positions[vertex_index] = source_position;
            return;
        }

        if (reference.type == CLOTH_CONSTRAINT_STRETCH)
        {
            if (reference.constraint_index >= stretch_constraint_count ||
                reference.local_vertex_index >= 2u)
            {
                fail_cloth_csr();
                next_positions[vertex_index] = source_position;
                return;
            }
            const ClothStretchConstraint constraint =
                stretch_constraints[reference.constraint_index];
            if (constraint.vertices[reference.local_vertex_index] != vertex_index)
            {
                fail_cloth_csr();
                next_positions[vertex_index] = source_position;
                return;
            }
            continue;
        }

        ClothConstraint constraint;
        ClothContribution contribution;
        if (reference.type == CLOTH_CONSTRAINT_SHEAR)
        {
            if (reference.constraint_index >= shear_constraint_count ||
                reference.local_vertex_index >= 3u)
            {
                fail_cloth_csr();
                next_positions[vertex_index] = source_position;
                return;
            }
            constraint = shear_constraints[reference.constraint_index];
            contribution = shear_contributions[reference.constraint_index];
        }
        else if (reference.type == CLOTH_CONSTRAINT_BEND)
        {
            if (reference.constraint_index >= bend_constraint_count ||
                reference.local_vertex_index >= 4u)
            {
                fail_cloth_csr();
                next_positions[vertex_index] = source_position;
                return;
            }
            constraint = bend_constraints[reference.constraint_index];
            contribution = bend_contributions[reference.constraint_index];
        }
        else
        {
            fail_cloth_csr();
            next_positions[vertex_index] = source_position;
            return;
        }

        if (constraint.type != reference.type ||
            constraint.indices[reference.local_vertex_index] != vertex_index)
        {
            fail_cloth_csr();
            next_positions[vertex_index] = source_position;
            return;
        }

        const float3 correction = get_contribution(
            contribution, reference.local_vertex_index);
        if (!finite3(correction))
        {
            fail_cloth_csr();
            next_positions[vertex_index] = source_position;
            return;
        }
        total_correction += correction;
    }

    const float3 result_position = source_position.xyz + total_correction;
    if (!finite3(total_correction) || !finite3(result_position))
    {
        fail_cloth_csr();
        next_positions[vertex_index] = source_position;
        return;
    }
    next_positions[vertex_index] =
        float4(result_position, source_position.w);
}

bool finite3(float3 value)
{
    return all(isfinite(value));
}
