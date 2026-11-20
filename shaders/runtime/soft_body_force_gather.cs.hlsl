struct TetInput
{
    uint4 vertex_ids;
    float4 rest_inverse_row0;
    float4 rest_inverse_row1;
    float4 rest_inverse_row2;
    float4 rest_gradient0;
    float4 rest_gradient1;
    float4 rest_gradient2;
    float4 rest_gradient3;
    float4 material0;
    float4 material1;
};

struct TetOutput
{
    float4 deformation_row0;
    float4 deformation_row1;
    float4 deformation_row2;
    float4 rotation_row0;
    float4 rotation_row1;
    float4 rotation_row2;
    float4 stress_row0;
    float4 stress_row1;
    float4 stress_row2;
    float4 force0;
    float4 force1;
    float4 force2;
    float4 force3;
    float4 metrics;
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
    uint render_slot_id;
    uint render_slot_generation;
    uint reserved0;
    uint reserved1;
    uint reserved2;
};

StructuredBuffer<TetOutput> tet_outputs : register(t0);
StructuredBuffer<TetInput> tetrahedra : register(t1);
StructuredBuffer<uint> vertex_tetrahedron_offsets : register(t2);
StructuredBuffer<uint> vertex_tetrahedron_ids : register(t3);
RWStructuredBuffer<float4> vertex_force_output : register(u0);
RWStructuredBuffer<TickCommitControl> tick_commit_control_buffer : register(u1);

cbuffer DispatchParameters : register(b0)
{
    uint vertex_count;
    uint3 reserved;
};

static const uint TICK_GATE_READY = 1u;
static const uint TICK_TERMINAL_IN_FLIGHT = 1u;

[numthreads(64, 1, 1)]
void soft_body_force_gather(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint vertex_index = dispatch_id.x;
    if (vertex_index >= vertex_count)
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

    float3 force_sum = 0.0f;
    uint begin_index = vertex_tetrahedron_offsets[vertex_index];
    uint end_index = vertex_tetrahedron_offsets[vertex_index + 1u];
    for (uint incidence = begin_index; incidence < end_index; ++incidence)
    {
        uint tet_index = vertex_tetrahedron_ids[incidence];
        TetInput tet = tetrahedra[tet_index];
        TetOutput output = tet_outputs[tet_index];

        if (tet.vertex_ids.x == vertex_index) force_sum += output.force0.xyz;
        else if (tet.vertex_ids.y == vertex_index) force_sum += output.force1.xyz;
        else if (tet.vertex_ids.z == vertex_index) force_sum += output.force2.xyz;
        else if (tet.vertex_ids.w == vertex_index) force_sum += output.force3.xyz;
    }

    vertex_force_output[vertex_index] = float4(force_sum, 0.0f);
}
