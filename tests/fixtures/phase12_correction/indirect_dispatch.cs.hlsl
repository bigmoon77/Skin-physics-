RWStructuredBuffer<uint> generated_candidate_count : register(u0);
StructuredBuffer<uint> candidate_count_input : register(t0);
RWBuffer<uint> dispatch_arguments : register(u1);
RWStructuredBuffer<uint> dispatch_output : register(u2);

[numthreads(1, 1, 1)]
void produce_candidate_count(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u)
    {
        return;
    }

    // Represents a GPU Candidate Count result consumed by the argument builder.
    generated_candidate_count[0] = 3u;
}

[numthreads(1, 1, 1)]
void build_dispatch_arguments(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u)
    {
        return;
    }

    dispatch_arguments[0] = candidate_count_input[0];
    dispatch_arguments[1] = 1u;
    dispatch_arguments[2] = 1u;
}

[numthreads(1, 1, 1)]
void consume_dispatch_arguments(uint3 dispatch_id : SV_DispatchThreadID)
{
    dispatch_output[dispatch_id.x] = 0x5a00u + dispatch_id.x;
}
