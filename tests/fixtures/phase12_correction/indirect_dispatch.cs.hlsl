RWBuffer<uint> dispatch_arguments : register(u0);
RWStructuredBuffer<uint> dispatch_output : register(u0);

[numthreads(1, 1, 1)]
void build_dispatch_arguments(uint3 dispatch_id : SV_DispatchThreadID)
{
    if (dispatch_id.x != 0u)
    {
        return;
    }

    dispatch_arguments[0] = 3u;
    dispatch_arguments[1] = 1u;
    dispatch_arguments[2] = 1u;
}

[numthreads(1, 1, 1)]
void consume_dispatch_arguments(uint3 dispatch_id : SV_DispatchThreadID)
{
    dispatch_output[dispatch_id.x] = 0x5a00u + dispatch_id.x;
}
