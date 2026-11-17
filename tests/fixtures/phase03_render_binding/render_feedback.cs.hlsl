struct RenderBinding
{
    uint4 indices;
    float4 weights;
    float4 rest_offset;
};

StructuredBuffer<float4> physics_positions : register(t0);
StructuredBuffer<RenderBinding> render_bindings : register(t1);
RWStructuredBuffer<float4> render_positions : register(u0);

[numthreads(64, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint binding_count;
    uint binding_stride;
    render_bindings.GetDimensions(binding_count, binding_stride);
    if (dispatch_id.x >= binding_count)
    {
        return;
    }

    RenderBinding binding = render_bindings[dispatch_id.x];
    float4 position;
    if (binding.indices.w == 1)
    {
        position = physics_positions[binding.indices.x];
    }
    else
    {
        position =
            physics_positions[binding.indices.x] * binding.weights.x +
            physics_positions[binding.indices.y] * binding.weights.y +
            physics_positions[binding.indices.z] * binding.weights.z +
            physics_positions[binding.indices.w] * binding.weights.w;
    }
    render_positions[dispatch_id.x] = float4(position.xyz + binding.rest_offset.xyz, 1.0f);
}
