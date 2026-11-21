cbuffer ClothLambdaResetParameters : register(b0)
{
    uint lambda_count;
    uint reserved0;
    uint reserved1;
    uint reserved2;
};

RWStructuredBuffer<float> constraint_lambdas : register(u0);
RWStructuredBuffer<uint> solver_status : register(u1);

static const uint STATUS_INVALID_CLOTH_LAMBDA_RESET = 1u << 5u;

[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    const uint lambda_index = dispatch_thread_id.x;
    uint actual_count = 0u;
    uint stride = 0u;
    uint status_count = 0u;
    uint status_stride = 0u;
    constraint_lambdas.GetDimensions(actual_count, stride);
    solver_status.GetDimensions(status_count, status_stride);

    if (status_count != 1u || status_stride != 4u)
    {
        return;
    }
    if (reserved0 != 0u || reserved1 != 0u || reserved2 != 0u ||
        lambda_count != actual_count || stride != 4u)
    {
        uint ignored = 0u;
        InterlockedOr(
            solver_status[0], STATUS_INVALID_CLOTH_LAMBDA_RESET, ignored);
        return;
    }
    if (lambda_index < lambda_count)
    {
        constraint_lambdas[lambda_index] = 0.0f;
    }
}
