cbuffer AnimationParameters : register(b0)
{
    float4 translation_start;
    float4 translation_end;
    float4 rotation_start;
    float4 rotation_end;
    float4 timing;
    float4 teleport_target;
};

RWStructuredBuffer<float4> animation_output : register(u0);

float4 normalize_quaternion(float4 value)
{
    float magnitude = length(value);
    return magnitude > 1.0e-12f ? value / magnitude : float4(0.0f, 0.0f, 0.0f, 1.0f);
}

float4 quaternion_slerp(float4 a, float4 b, float t)
{
    a = normalize_quaternion(a);
    b = normalize_quaternion(b);
    float cosine_value = dot(a, b);
    if (cosine_value < 0.0f)
    {
        b = -b;
        cosine_value = -cosine_value;
    }
    cosine_value = clamp(cosine_value, -1.0f, 1.0f);
    if (cosine_value > 0.9995f)
    {
        return normalize_quaternion(lerp(a, b, t));
    }
    float angle = acos(cosine_value);
    float sine_value = sin(angle);
    if (abs(sine_value) <= 1.0e-12f)
    {
        return a;
    }
    float weight_a = sin((1.0f - t) * angle) / sine_value;
    float weight_b = sin(t * angle) / sine_value;
    return normalize_quaternion(a * weight_a + b * weight_b);
}

[numthreads(1, 1, 1)]
void animation_target(uint3 dispatch_id : SV_DispatchThreadID)
{
    float duration = timing.y - timing.x;
    float normalized_time = duration > 1.0e-12f
        ? saturate((timing.z - timing.x) / duration)
        : 0.0f;
    float3 translation = lerp(
        translation_start.xyz, translation_end.xyz, normalized_time);
    float3 velocity = duration > 1.0e-12f
        ? (translation_end.xyz - translation_start.xyz) / duration
        : float3(0.0f, 0.0f, 0.0f);
    float4 rotation = quaternion_slerp(
        rotation_start, rotation_end, normalized_time);

    if (timing.w > 0.5f)
    {
        translation = teleport_target.xyz;
        velocity = float3(0.0f, 0.0f, 0.0f);
    }

    animation_output[0] = float4(translation, 1.0f);
    animation_output[1] = float4(velocity, 0.0f);
    animation_output[2] = rotation;
}

StructuredBuffer<float> source_weights : register(t0);
StructuredBuffer<uint> selected_vertices : register(t1);
RWStructuredBuffer<float> painted_weights : register(u0);

cbuffer PaintParameters : register(b1)
{
    uint vertex_count;
    uint paint_mode;
    float paint_target;
    float paint_strength;
    float minimum_weight;
    float maximum_weight;
    float selected_average;
    float reserved;
};

[numthreads(64, 1, 1)]
void attribute_paint(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint vertex_id = dispatch_id.x;
    if (vertex_id >= vertex_count)
    {
        return;
    }

    float source = source_weights[vertex_id];
    float result = source;
    if (selected_vertices[vertex_id] != 0)
    {
        float destination = paint_target;
        float strength = paint_strength;
        if (paint_mode == 1)
        {
            destination = selected_average;
        }
        else if (paint_mode == 2)
        {
            destination = selected_average;
            strength = 1.0f;
        }
        else if (paint_mode == 3)
        {
            destination = minimum_weight;
        }
        result = clamp(source + (destination - source) * strength,
            minimum_weight, maximum_weight);
    }
    painted_weights[vertex_id] = result;
}
