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

StructuredBuffer<float4> current_positions : register(t0);
StructuredBuffer<TetInput> tetrahedra : register(t1);
RWStructuredBuffer<TetOutput> outputs : register(u0);

cbuffer DispatchParameters : register(b0)
{
    uint tetrahedron_count;
    uint3 reserved;
};

float determinant3(float3x3 matrix_value)
{
    return
        matrix_value[0][0] * (matrix_value[1][1] * matrix_value[2][2] -
            matrix_value[1][2] * matrix_value[2][1]) -
        matrix_value[0][1] * (matrix_value[1][0] * matrix_value[2][2] -
            matrix_value[1][2] * matrix_value[2][0]) +
        matrix_value[0][2] * (matrix_value[1][0] * matrix_value[2][1] -
            matrix_value[1][1] * matrix_value[2][0]);
}

bool inverse3(float3x3 value, out float3x3 result)
{
    float determinant_value = determinant3(value);
    if (!isfinite(determinant_value) || abs(determinant_value) <= 1.0e-12f)
    {
        result = (float3x3)0.0f;
        return false;
    }

    float reciprocal = 1.0f / determinant_value;
    result[0][0] = (value[1][1] * value[2][2] - value[1][2] * value[2][1]) * reciprocal;
    result[0][1] = (value[0][2] * value[2][1] - value[0][1] * value[2][2]) * reciprocal;
    result[0][2] = (value[0][1] * value[1][2] - value[0][2] * value[1][1]) * reciprocal;
    result[1][0] = (value[1][2] * value[2][0] - value[1][0] * value[2][2]) * reciprocal;
    result[1][1] = (value[0][0] * value[2][2] - value[0][2] * value[2][0]) * reciprocal;
    result[1][2] = (value[0][2] * value[1][0] - value[0][0] * value[1][2]) * reciprocal;
    result[2][0] = (value[1][0] * value[2][1] - value[1][1] * value[2][0]) * reciprocal;
    result[2][1] = (value[0][1] * value[2][0] - value[0][0] * value[2][1]) * reciprocal;
    result[2][2] = (value[0][0] * value[1][1] - value[0][1] * value[1][0]) * reciprocal;
    return true;
}

float frobenius_squared(float3x3 value)
{
    return dot(value[0], value[0]) +
        dot(value[1], value[1]) +
        dot(value[2], value[2]);
}

float3x3 identity3()
{
    return float3x3(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f);
}

[numthreads(64, 1, 1)]
void main(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint index = dispatch_id.x;
    if (index >= tetrahedron_count)
    {
        return;
    }

    TetInput input = tetrahedra[index];
    float3 p0 = current_positions[input.vertex_ids.x].xyz;
    float3 p1 = current_positions[input.vertex_ids.y].xyz;
    float3 p2 = current_positions[input.vertex_ids.z].xyz;
    float3 p3 = current_positions[input.vertex_ids.w].xyz;

    float3x3 deformed_edges = float3x3(
        p1.x - p0.x, p2.x - p0.x, p3.x - p0.x,
        p1.y - p0.y, p2.y - p0.y, p3.y - p0.y,
        p1.z - p0.z, p2.z - p0.z, p3.z - p0.z);
    float3x3 rest_inverse = float3x3(
        input.rest_inverse_row0.xyz,
        input.rest_inverse_row1.xyz,
        input.rest_inverse_row2.xyz);
    float3x3 deformation = mul(deformed_edges, rest_inverse);
    float jacobian = determinant3(deformation);

    float3x3 rotation = identity3();
    bool rotation_valid = false;
    if (isfinite(jacobian) && jacobian > 1.0e-8f)
    {
        rotation = deformation;
        rotation_valid = true;
        [unroll]
        for (uint iteration = 0; iteration < 8; ++iteration)
        {
            float3x3 inverse_rotation;
            if (!inverse3(rotation, inverse_rotation))
            {
                rotation_valid = false;
                break;
            }
            float3x3 inverse_transpose = transpose(inverse_rotation);
            float source_norm = frobenius_squared(rotation);
            float inverse_norm = frobenius_squared(inverse_transpose);
            if (source_norm <= 1.0e-20f || inverse_norm <= 1.0e-20f ||
                !isfinite(source_norm) || !isfinite(inverse_norm))
            {
                rotation_valid = false;
                break;
            }
            float gamma_value = sqrt(sqrt(inverse_norm / source_norm));
            if (!isfinite(gamma_value) || gamma_value <= 1.0e-12f)
            {
                rotation_valid = false;
                break;
            }
            rotation = 0.5f * (
                gamma_value * rotation + (1.0f / gamma_value) * inverse_transpose);
        }
        float3x3 orthogonality = mul(transpose(rotation), rotation) - identity3();
        rotation_valid = rotation_valid &&
            sqrt(frobenius_squared(orthogonality)) <= 1.0e-3f &&
            determinant3(rotation) > 0.0f;
    }

    float3x3 corotated = mul(transpose(rotation), deformation);
    float3x3 strain = corotated - identity3();
    float trace_value = strain[0][0] + strain[1][1] + strain[2][2];
    float3x3 deviatoric = strain;
    deviatoric[0][0] -= trace_value / 3.0f;
    deviatoric[1][1] -= trace_value / 3.0f;
    deviatoric[2][2] -= trace_value / 3.0f;

    float shear_modulus = input.material0.y * input.material0.w;
    float bulk_modulus = input.material0.z * input.material1.x;
    float strain_energy_density =
        shear_modulus * frobenius_squared(deviatoric) +
        0.5f * bulk_modulus * trace_value * trace_value;
    float3x3 corotational_stress =
        2.0f * shear_modulus * deviatoric +
        bulk_modulus * trace_value * identity3();
    float3x3 first_piola = mul(rotation, corotational_stress);

    float inversion_distance = min(jacobian - input.material1.z, 0.0f);
    float inversion_energy_density =
        0.5f * input.material1.y * inversion_distance * inversion_distance;
    float3x3 inversion_stress = (float3x3)0.0f;
    float3x3 inverse_deformation;
    if (abs(jacobian) > 1.0e-12f && inverse3(deformation, inverse_deformation))
    {
        inversion_stress = transpose(inverse_deformation) *
            (jacobian * input.material1.y * inversion_distance);
    }
    float3x3 total_stress = first_piola + inversion_stress;

    float rest_volume = input.material0.x;
    float current_volume = rest_volume * jacobian;
    float elastic_energy = rest_volume *
        (strain_energy_density + inversion_energy_density);
    float3 force0 = -rest_volume * mul(total_stress, input.rest_gradient0.xyz);
    float3 force1 = -rest_volume * mul(total_stress, input.rest_gradient1.xyz);
    float3 force2 = -rest_volume * mul(total_stress, input.rest_gradient2.xyz);
    float3 force3 = -rest_volume * mul(total_stress, input.rest_gradient3.xyz);

    TetOutput output;
    output.deformation_row0 = float4(deformation[0], 0.0f);
    output.deformation_row1 = float4(deformation[1], 0.0f);
    output.deformation_row2 = float4(deformation[2], 0.0f);
    output.rotation_row0 = float4(rotation[0], 0.0f);
    output.rotation_row1 = float4(rotation[1], 0.0f);
    output.rotation_row2 = float4(rotation[2], 0.0f);
    output.stress_row0 = float4(first_piola[0], 0.0f);
    output.stress_row1 = float4(first_piola[1], 0.0f);
    output.stress_row2 = float4(first_piola[2], 0.0f);
    output.force0 = float4(force0, 0.0f);
    output.force1 = float4(force1, 0.0f);
    output.force2 = float4(force2, 0.0f);
    output.force3 = float4(force3, 0.0f);
    output.metrics = float4(jacobian, current_volume, elastic_energy,
        jacobian > 0.0f && rotation_valid ? 1.0f : 0.0f);
    outputs[index] = output;
}
