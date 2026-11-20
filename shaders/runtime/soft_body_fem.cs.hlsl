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

StructuredBuffer<float4> current_positions : register(t0);
StructuredBuffer<TetInput> tetrahedra : register(t1);
RWStructuredBuffer<TetOutput> outputs : register(u0);
RWStructuredBuffer<TickCommitControl> tick_commit_control_buffer : register(u1);

cbuffer DispatchParameters : register(b0)
{
    uint tetrahedron_count;
    uint3 reserved;
};

static const uint TICK_GATE_READY = 1u;
static const uint TICK_TERMINAL_IN_FLIGHT = 1u;

float determinant3(float3x3 value)
{
    return
        value[0][0] * (value[1][1] * value[2][2] - value[1][2] * value[2][1]) -
        value[0][1] * (value[1][0] * value[2][2] - value[1][2] * value[2][0]) +
        value[0][2] * (value[1][0] * value[2][1] - value[1][1] * value[2][0]);
}

bool inverse3(float3x3 value, out float3x3 result)
{
    float det = determinant3(value);
    if (!isfinite(det) || abs(det) <= 1.0e-12f)
    {
        result = (float3x3)0.0f;
        return false;
    }

    float reciprocal = 1.0f / det;
    result[0][0] = (value[1][1] * value[2][2] - value[1][2] * value[2][1]) * reciprocal;
    result[0][1] = (value[0][2] * value[2][1] - value[0][1] * value[2][2]) * reciprocal;
    result[0][2] = (value[0][1] * value[1][2] - value[0][2] * value[1][1]) * reciprocal;
    result[1][0] = (value[1][2] * value[2][0] - value[1][0] * value[2][2]) * reciprocal;
    result[1][1] = (value[0][0] * value[2][2] - value[0][2] * value[2][0]) * reciprocal;
    result[1][2] = (value[0][2] * value[1][0] - value[0][0] * value[1][2]) * reciprocal;
    result[2][0] = (value[1][0] * value[2][1] - value[1][1] * value[2][0]) * reciprocal;
    result[2][1] = (value[0][1] * value[2][0] - value[0][0] * value[2][1]) * reciprocal;
    result[2][2] = (value[0][0] * value[1][1] - value[0][1] * value[1][0]) * reciprocal;
    return all(isfinite(result[0])) && all(isfinite(result[1])) && all(isfinite(result[2]));
}

float frobenius_squared(float3x3 value)
{
    return dot(value[0], value[0]) + dot(value[1], value[1]) + dot(value[2], value[2]);
}

float3x3 identity3()
{
    return float3x3(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f);
}

float3x3 cofactor3(float3x3 value)
{
    return float3x3(
        value[1][1] * value[2][2] - value[1][2] * value[2][1],
        value[1][2] * value[2][0] - value[1][0] * value[2][2],
        value[1][0] * value[2][1] - value[1][1] * value[2][0],
        value[0][2] * value[2][1] - value[0][1] * value[2][2],
        value[0][0] * value[2][2] - value[0][2] * value[2][0],
        value[0][1] * value[2][0] - value[0][0] * value[2][1],
        value[0][1] * value[1][2] - value[0][2] * value[1][1],
        value[0][2] * value[1][0] - value[0][0] * value[1][2],
        value[0][0] * value[1][1] - value[0][1] * value[1][0]);
}

float3 matrix_column(float3x3 value, uint column)
{
    return float3(value[0][column], value[1][column], value[2][column]);
}

void set_matrix_column(inout float3x3 value, uint column, float3 column_value)
{
    value[0][column] = column_value.x;
    value[1][column] = column_value.y;
    value[2][column] = column_value.z;
}

bool jacobi_svd_rotation(float3x3 deformation, out float3x3 rotation)
{
    float3x3 symmetric = mul(transpose(deformation), deformation);
    float3x3 eigenvectors = identity3();

    [loop]
    for (uint iteration = 0u; iteration < 32u; ++iteration)
    {
        uint p = 0u;
        uint q = 1u;
        float maximum_off_diagonal = abs(symmetric[0][1]);
        if (abs(symmetric[0][2]) > maximum_off_diagonal)
        {
            p = 0u;
            q = 2u;
            maximum_off_diagonal = abs(symmetric[0][2]);
        }
        if (abs(symmetric[1][2]) > maximum_off_diagonal)
        {
            p = 1u;
            q = 2u;
            maximum_off_diagonal = abs(symmetric[1][2]);
        }
        if (maximum_off_diagonal <= 1.0e-12f)
        {
            break;
        }

        float app = symmetric[p][p];
        float aqq = symmetric[q][q];
        float apq = symmetric[p][q];
        float theta = (aqq - app) / (2.0f * apq);
        float sign_value = theta >= 0.0f ? 1.0f : -1.0f;
        float tangent = sign_value /
            (abs(theta) + sqrt(1.0f + theta * theta));
        float cosine = rsqrt(1.0f + tangent * tangent);
        float sine = tangent * cosine;

        symmetric[p][p] = app - tangent * apq;
        symmetric[q][q] = aqq + tangent * apq;
        symmetric[p][q] = 0.0f;
        symmetric[q][p] = 0.0f;

        [unroll]
        for (uint k = 0u; k < 3u; ++k)
        {
            if (k == p || k == q)
            {
                continue;
            }
            float akp = symmetric[k][p];
            float akq = symmetric[k][q];
            symmetric[k][p] = cosine * akp - sine * akq;
            symmetric[p][k] = symmetric[k][p];
            symmetric[k][q] = sine * akp + cosine * akq;
            symmetric[q][k] = symmetric[k][q];
        }

        [unroll]
        for (uint row = 0u; row < 3u; ++row)
        {
            float vkp = eigenvectors[row][p];
            float vkq = eigenvectors[row][q];
            eigenvectors[row][p] = cosine * vkp - sine * vkq;
            eigenvectors[row][q] = sine * vkp + cosine * vkq;
        }
    }

    uint order[3] = { 0u, 1u, 2u };
    [unroll]
    for (uint first = 0u; first < 3u; ++first)
    {
        [unroll]
        for (uint second = first + 1u; second < 3u; ++second)
        {
            if (symmetric[order[second]][order[second]] >
                symmetric[order[first]][order[first]])
            {
                uint temporary = order[first];
                order[first] = order[second];
                order[second] = temporary;
            }
        }
    }

    float3x3 sorted_vectors = (float3x3)0.0f;
    float singular_values[3];
    [unroll]
    for (uint column = 0u; column < 3u; ++column)
    {
        singular_values[column] = sqrt(max(
            0.0f, symmetric[order[column]][order[column]]));
        set_matrix_column(
            sorted_vectors, column, matrix_column(eigenvectors, order[column]));
    }

    float3x3 left_vectors = (float3x3)0.0f;
    [unroll]
    for (uint column = 0u; column < 3u; ++column)
    {
        float3 candidate = mul(deformation, matrix_column(sorted_vectors, column));
        if (singular_values[column] > 1.0e-8f)
        {
            candidate *= 1.0f / singular_values[column];
        }

        [unroll]
        for (uint previous = 0u; previous < column; ++previous)
        {
            float3 previous_axis = matrix_column(left_vectors, previous);
            candidate -= previous_axis * dot(candidate, previous_axis);
        }

        float candidate_length_squared = dot(candidate, candidate);
        if (!isfinite(candidate_length_squared) ||
            candidate_length_squared <= 1.0e-16f)
        {
            float3 axis_x = float3(1.0f, 0.0f, 0.0f);
            float3 axis_y = float3(0.0f, 1.0f, 0.0f);
            float3 axis_z = float3(0.0f, 0.0f, 1.0f);

            [unroll]
            for (uint previous = 0u; previous < column; ++previous)
            {
                float3 previous_axis = matrix_column(left_vectors, previous);
                axis_x -= previous_axis * dot(axis_x, previous_axis);
                axis_y -= previous_axis * dot(axis_y, previous_axis);
                axis_z -= previous_axis * dot(axis_z, previous_axis);
            }

            float length_x = dot(axis_x, axis_x);
            float length_y = dot(axis_y, axis_y);
            float length_z = dot(axis_z, axis_z);
            candidate = axis_x;
            candidate_length_squared = length_x;
            if (length_y > candidate_length_squared)
            {
                candidate = axis_y;
                candidate_length_squared = length_y;
            }
            if (length_z > candidate_length_squared)
            {
                candidate = axis_z;
                candidate_length_squared = length_z;
            }
        }

        if (!isfinite(candidate_length_squared) ||
            candidate_length_squared <= 1.0e-16f)
        {
            rotation = identity3();
            return false;
        }

        candidate *= rsqrt(candidate_length_squared);
        set_matrix_column(left_vectors, column, candidate);
    }

    float3x3 orientation = mul(left_vectors, transpose(sorted_vectors));
    float3x3 correction = identity3();
    if (determinant3(orientation) < 0.0f)
    {
        correction[2][2] = -1.0f;
    }

    rotation = mul(
        mul(left_vectors, correction),
        transpose(sorted_vectors));
    return all(isfinite(rotation[0])) &&
        all(isfinite(rotation[1])) &&
        all(isfinite(rotation[2])) &&
        determinant3(rotation) > 0.0f;
}

[numthreads(64, 1, 1)]
void soft_body_fem(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint index = dispatch_id.x;
    if (index >= tetrahedron_count)
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

    float3x3 rotation = deformation;
    bool rotation_valid = isfinite(jacobian) && jacobian > 1.0e-8f;
    bool used_svd_fallback = false;
    if (rotation_valid)
    {
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

    if (!rotation_valid && isfinite(jacobian) && jacobian > 0.0f)
    {
        rotation_valid = jacobi_svd_rotation(deformation, rotation);
        used_svd_fallback = rotation_valid;
    }

    if (!rotation_valid || !all(isfinite(deformation[0])) ||
        !all(isfinite(deformation[1])) || !all(isfinite(deformation[2])))
    {
        uint previous_failure = 0u;
        InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
        return;
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
    float3x3 inversion_stress =
        cofactor3(deformation) *
        (input.material1.y * inversion_distance);
    float3x3 total_stress = first_piola + inversion_stress;

    float rest_volume = input.material0.x;
    float current_volume = rest_volume * jacobian;
    float elastic_energy = rest_volume *
        (strain_energy_density + inversion_energy_density);
    float3 force0 = -rest_volume * mul(total_stress, input.rest_gradient0.xyz);
    float3 force1 = -rest_volume * mul(total_stress, input.rest_gradient1.xyz);
    float3 force2 = -rest_volume * mul(total_stress, input.rest_gradient2.xyz);
    float3 force3 = -rest_volume * mul(total_stress, input.rest_gradient3.xyz);

    bool output_finite =
        all(isfinite(force0)) && all(isfinite(force1)) &&
        all(isfinite(force2)) && all(isfinite(force3)) &&
        isfinite(current_volume) && isfinite(elastic_energy) &&
        jacobian > 0.0f;
    if (!output_finite)
    {
        uint previous_failure = 0u;
        InterlockedOr(tick_commit_control_buffer[0].terminal_failure, 1u, previous_failure);
        return;
    }

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
    output.metrics = float4(
        jacobian, current_volume, elastic_energy,
        used_svd_fallback ? 2.0f : 1.0f);
    outputs[index] = output;
}
