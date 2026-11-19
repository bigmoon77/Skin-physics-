#include "skin_physics/physics/soft_body_fem.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace skin_physics::physics
{
    namespace
    {
        using asset::vec3;
        constexpr float minimum_determinant = 1.0e-12f;
        constexpr float minimum_singular_value = 1.0e-8f;

        float& at(matrix3& value, uint32_t row, uint32_t column)
        {
            return value.value[row * 3 + column];
        }

        float at(const matrix3& value, uint32_t row, uint32_t column)
        {
            return value.value[row * 3 + column];
        }

        matrix3 zero_matrix()
        {
            matrix3 result;
            result.value.fill(0.0f);
            return result;
        }

        matrix3 identity_matrix()
        {
            return {};
        }

        matrix3 transpose(const matrix3& value)
        {
            matrix3 result;
            for (uint32_t row = 0; row < 3; ++row)
            {
                for (uint32_t column = 0; column < 3; ++column)
                {
                    at(result, row, column) = at(value, column, row);
                }
            }
            return result;
        }

        matrix3 add(const matrix3& left, const matrix3& right)
        {
            matrix3 result;
            for (std::size_t index = 0; index < result.value.size(); ++index)
            {
                result.value[index] = left.value[index] + right.value[index];
            }
            return result;
        }

        matrix3 subtract(const matrix3& left, const matrix3& right)
        {
            matrix3 result;
            for (std::size_t index = 0; index < result.value.size(); ++index)
            {
                result.value[index] = left.value[index] - right.value[index];
            }
            return result;
        }

        matrix3 scale(const matrix3& value, float factor)
        {
            matrix3 result;
            for (std::size_t index = 0; index < result.value.size(); ++index)
            {
                result.value[index] = value.value[index] * factor;
            }
            return result;
        }

        vec3 scale_vector(const vec3& value, float factor)
        {
            return { value.x * factor, value.y * factor, value.z * factor };
        }

        float frobenius_squared(const matrix3& value)
        {
            float result = 0.0f;
            for (float component : value.value)
            {
                result += component * component;
            }
            return result;
        }

        bool finite(const matrix3& value)
        {
            for (float component : value.value)
            {
                if (!std::isfinite(component)) return false;
            }
            return true;
        }

        bool finite(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        matrix3 multiply(const matrix3& left, const matrix3& right)
        {
            matrix3 result = zero_matrix();
            for (uint32_t row = 0; row < 3; ++row)
            {
                for (uint32_t column = 0; column < 3; ++column)
                {
                    for (uint32_t inner = 0; inner < 3; ++inner)
                    {
                        at(result, row, column) +=
                            at(left, row, inner) * at(right, inner, column);
                    }
                }
            }
            return result;
        }

        vec3 multiply(const matrix3& matrix, const vec3& vector)
        {
            return {
                at(matrix, 0, 0) * vector.x + at(matrix, 0, 1) * vector.y + at(matrix, 0, 2) * vector.z,
                at(matrix, 1, 0) * vector.x + at(matrix, 1, 1) * vector.y + at(matrix, 1, 2) * vector.z,
                at(matrix, 2, 0) * vector.x + at(matrix, 2, 1) * vector.y + at(matrix, 2, 2) * vector.z
            };
        }

        float determinant(const matrix3& value)
        {
            const float a = at(value, 0, 0);
            const float b = at(value, 0, 1);
            const float c = at(value, 0, 2);
            const float d = at(value, 1, 0);
            const float e = at(value, 1, 1);
            const float f = at(value, 1, 2);
            const float g = at(value, 2, 0);
            const float h = at(value, 2, 1);
            const float i = at(value, 2, 2);
            return a * (e * i - f * h) -
                b * (d * i - f * g) +
                c * (d * h - e * g);
        }

        bool inverse(const matrix3& value, matrix3& result)
        {
            const float det = determinant(value);
            if (!std::isfinite(det) || std::abs(det) <= minimum_determinant)
            {
                return false;
            }
            const float reciprocal = 1.0f / det;
            result.value = {
                (at(value, 1, 1) * at(value, 2, 2) - at(value, 1, 2) * at(value, 2, 1)) * reciprocal,
                (at(value, 0, 2) * at(value, 2, 1) - at(value, 0, 1) * at(value, 2, 2)) * reciprocal,
                (at(value, 0, 1) * at(value, 1, 2) - at(value, 0, 2) * at(value, 1, 1)) * reciprocal,
                (at(value, 1, 2) * at(value, 2, 0) - at(value, 1, 0) * at(value, 2, 2)) * reciprocal,
                (at(value, 0, 0) * at(value, 2, 2) - at(value, 0, 2) * at(value, 2, 0)) * reciprocal,
                (at(value, 0, 2) * at(value, 1, 0) - at(value, 0, 0) * at(value, 1, 2)) * reciprocal,
                (at(value, 1, 0) * at(value, 2, 1) - at(value, 1, 1) * at(value, 2, 0)) * reciprocal,
                (at(value, 0, 1) * at(value, 2, 0) - at(value, 0, 0) * at(value, 2, 1)) * reciprocal,
                (at(value, 0, 0) * at(value, 1, 1) - at(value, 0, 1) * at(value, 1, 0)) * reciprocal
            };
            return finite(result);
        }

        matrix3 cofactor(const matrix3& value)
        {
            matrix3 result;
            at(result, 0, 0) = at(value, 1, 1) * at(value, 2, 2) - at(value, 1, 2) * at(value, 2, 1);
            at(result, 0, 1) = at(value, 1, 2) * at(value, 2, 0) - at(value, 1, 0) * at(value, 2, 2);
            at(result, 0, 2) = at(value, 1, 0) * at(value, 2, 1) - at(value, 1, 1) * at(value, 2, 0);
            at(result, 1, 0) = at(value, 0, 2) * at(value, 2, 1) - at(value, 0, 1) * at(value, 2, 2);
            at(result, 1, 1) = at(value, 0, 0) * at(value, 2, 2) - at(value, 0, 2) * at(value, 2, 0);
            at(result, 1, 2) = at(value, 0, 1) * at(value, 2, 0) - at(value, 0, 0) * at(value, 2, 1);
            at(result, 2, 0) = at(value, 0, 1) * at(value, 1, 2) - at(value, 0, 2) * at(value, 1, 1);
            at(result, 2, 1) = at(value, 0, 2) * at(value, 1, 0) - at(value, 0, 0) * at(value, 1, 2);
            at(result, 2, 2) = at(value, 0, 0) * at(value, 1, 1) - at(value, 0, 1) * at(value, 1, 0);
            return result;
        }

        matrix3 make_deformation_gradient(
            const tetrahedron& tet,
            const std::vector<vec3>& current_positions)
        {
            matrix3 deformed_edges = zero_matrix();
            const vec3& origin = current_positions[tet.vertices[0]];
            for (uint32_t column = 0; column < 3; ++column)
            {
                const vec3& point = current_positions[tet.vertices[column + 1]];
                at(deformed_edges, 0, column) = point.x - origin.x;
                at(deformed_edges, 1, column) = point.y - origin.y;
                at(deformed_edges, 2, column) = point.z - origin.z;
            }

            matrix3 rest_inverse;
            rest_inverse.value = tet.rest_inverse;
            return multiply(deformed_edges, rest_inverse);
        }

        bool polar_rotation(const matrix3& deformation, matrix3& rotation)
        {
            rotation = deformation;
            for (uint32_t iteration = 0; iteration < 8; ++iteration)
            {
                matrix3 inverse_rotation;
                if (!inverse(rotation, inverse_rotation))
                {
                    return false;
                }
                const matrix3 inverse_transpose = transpose(inverse_rotation);
                const float source_norm = frobenius_squared(rotation);
                const float inverse_norm = frobenius_squared(inverse_transpose);
                if (source_norm <= 1.0e-20f || inverse_norm <= 1.0e-20f ||
                    !std::isfinite(source_norm) || !std::isfinite(inverse_norm))
                {
                    return false;
                }
                const float gamma = std::sqrt(std::sqrt(inverse_norm / source_norm));
                if (!std::isfinite(gamma) || gamma <= 1.0e-12f)
                {
                    return false;
                }
                rotation = scale(add(
                    scale(rotation, gamma),
                    scale(inverse_transpose, 1.0f / gamma)), 0.5f);
                if (!finite(rotation))
                {
                    return false;
                }
            }

            const matrix3 orthogonality = subtract(
                multiply(transpose(rotation), rotation),
                identity_matrix());
            const float residual = std::sqrt(frobenius_squared(orthogonality));
            return std::isfinite(residual) && residual <= 1.0e-3f &&
                determinant(rotation) > 0.0f;
        }

        bool jacobi_svd_rotation(const matrix3& deformation, matrix3& rotation)
        {
            matrix3 symmetric = multiply(transpose(deformation), deformation);
            matrix3 eigenvectors = identity_matrix();

            for (uint32_t iteration = 0; iteration < 32; ++iteration)
            {
                uint32_t p = 0;
                uint32_t q = 1;
                float maximum_off_diagonal = std::abs(at(symmetric, 0, 1));
                if (std::abs(at(symmetric, 0, 2)) > maximum_off_diagonal)
                {
                    p = 0;
                    q = 2;
                    maximum_off_diagonal = std::abs(at(symmetric, 0, 2));
                }
                if (std::abs(at(symmetric, 1, 2)) > maximum_off_diagonal)
                {
                    p = 1;
                    q = 2;
                    maximum_off_diagonal = std::abs(at(symmetric, 1, 2));
                }
                if (maximum_off_diagonal <= 1.0e-12f)
                {
                    break;
                }

                const float app = at(symmetric, p, p);
                const float aqq = at(symmetric, q, q);
                const float apq = at(symmetric, p, q);
                const float theta = (aqq - app) / (2.0f * apq);
                const float sign = theta >= 0.0f ? 1.0f : -1.0f;
                const float tangent = sign /
                    (std::abs(theta) + std::sqrt(1.0f + theta * theta));
                const float cosine = 1.0f / std::sqrt(1.0f + tangent * tangent);
                const float sine = tangent * cosine;

                at(symmetric, p, p) = app - tangent * apq;
                at(symmetric, q, q) = aqq + tangent * apq;
                at(symmetric, p, q) = 0.0f;
                at(symmetric, q, p) = 0.0f;
                for (uint32_t k = 0; k < 3; ++k)
                {
                    if (k == p || k == q) continue;
                    const float akp = at(symmetric, k, p);
                    const float akq = at(symmetric, k, q);
                    at(symmetric, k, p) = cosine * akp - sine * akq;
                    at(symmetric, p, k) = at(symmetric, k, p);
                    at(symmetric, k, q) = sine * akp + cosine * akq;
                    at(symmetric, q, k) = at(symmetric, k, q);
                }
                for (uint32_t k = 0; k < 3; ++k)
                {
                    const float vkp = at(eigenvectors, k, p);
                    const float vkq = at(eigenvectors, k, q);
                    at(eigenvectors, k, p) = cosine * vkp - sine * vkq;
                    at(eigenvectors, k, q) = sine * vkp + cosine * vkq;
                }
            }

            std::array<uint32_t, 3> order = { 0, 1, 2 };
            std::sort(order.begin(), order.end(),
                [&symmetric](uint32_t left, uint32_t right)
                {
                    return at(symmetric, left, left) > at(symmetric, right, right);
                });

            matrix3 sorted_vectors = zero_matrix();
            std::array<float, 3> singular_values = {};
            for (uint32_t column = 0; column < 3; ++column)
            {
                singular_values[column] = std::sqrt(std::max(0.0f,
                    at(symmetric, order[column], order[column])));
                for (uint32_t row = 0; row < 3; ++row)
                {
                    at(sorted_vectors, row, column) = at(eigenvectors, row, order[column]);
                }
            }

            matrix3 left_vectors = zero_matrix();
            for (uint32_t column = 0; column < 3; ++column)
            {
                vec3 candidate = {
                    at(multiply(deformation, sorted_vectors), 0, column),
                    at(multiply(deformation, sorted_vectors), 1, column),
                    at(multiply(deformation, sorted_vectors), 2, column)
                };
                if (singular_values[column] > minimum_singular_value)
                {
                    candidate = scale_vector(candidate, 1.0f / singular_values[column]);
                }

                for (uint32_t previous = 0; previous < column; ++previous)
                {
                    const vec3 previous_axis = {
                        at(left_vectors, 0, previous),
                        at(left_vectors, 1, previous),
                        at(left_vectors, 2, previous)
                    };
                    const float projection =
                        candidate.x * previous_axis.x +
                        candidate.y * previous_axis.y +
                        candidate.z * previous_axis.z;
                    candidate.x -= previous_axis.x * projection;
                    candidate.y -= previous_axis.y * projection;
                    candidate.z -= previous_axis.z * projection;
                }

                float candidate_length = std::sqrt(
                    candidate.x * candidate.x +
                    candidate.y * candidate.y +
                    candidate.z * candidate.z);
                if (candidate_length <= 1.0e-8f)
                {
                    const std::array<vec3, 3> axes = {{
                        { 1.0f, 0.0f, 0.0f },
                        { 0.0f, 1.0f, 0.0f },
                        { 0.0f, 0.0f, 1.0f }
                    }};
                    float best_length = -1.0f;
                    vec3 best_axis = {};
                    for (const vec3& axis : axes)
                    {
                        vec3 projected = axis;
                        for (uint32_t previous = 0; previous < column; ++previous)
                        {
                            const vec3 previous_axis = {
                                at(left_vectors, 0, previous),
                                at(left_vectors, 1, previous),
                                at(left_vectors, 2, previous)
                            };
                            const float projection =
                                projected.x * previous_axis.x +
                                projected.y * previous_axis.y +
                                projected.z * previous_axis.z;
                            projected.x -= previous_axis.x * projection;
                            projected.y -= previous_axis.y * projection;
                            projected.z -= previous_axis.z * projection;
                        }
                        const float projected_length = projected.x * projected.x +
                            projected.y * projected.y + projected.z * projected.z;
                        if (projected_length > best_length)
                        {
                            best_length = projected_length;
                            best_axis = projected;
                        }
                    }
                    candidate = best_axis;
                    candidate_length = std::sqrt(std::max(0.0f, best_length));
                }
                if (candidate_length <= 1.0e-8f || !std::isfinite(candidate_length))
                {
                    return false;
                }
                candidate = scale_vector(candidate, 1.0f / candidate_length);
                at(left_vectors, 0, column) = candidate.x;
                at(left_vectors, 1, column) = candidate.y;
                at(left_vectors, 2, column) = candidate.z;
            }

            const matrix3 orientation = multiply(left_vectors, transpose(sorted_vectors));
            matrix3 correction = identity_matrix();
            if (determinant(orientation) < 0.0f)
            {
                at(correction, 2, 2) = -1.0f;
            }
            rotation = multiply(
                multiply(left_vectors, correction),
                transpose(sorted_vectors));
            return finite(rotation) && determinant(rotation) > 0.0f;
        }

        void add_diagnostic(
            corotational_tet_result& result,
            soft_body_error code,
            uint32_t element,
            const std::string& message)
        {
            soft_body_diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.element_index = element;
            diagnostic.message = message;
            result.diagnostics.push_back(std::move(diagnostic));
        }

        bool valid_material(const soft_body_material& material)
        {
            return std::isfinite(material.shear_modulus) &&
                material.shear_modulus > 0.0f &&
                std::isfinite(material.bulk_modulus) &&
                material.bulk_modulus > 0.0f &&
                std::isfinite(material.density) &&
                material.density > 0.0f &&
                std::isfinite(material.shape_recovery_scale) &&
                material.shape_recovery_scale >= 0.0f &&
                std::isfinite(material.volume_retention_scale) &&
                material.volume_retention_scale >= 0.0f &&
                std::isfinite(material.inversion_stiffness) &&
                material.inversion_stiffness >= 0.0f &&
                std::isfinite(material.minimum_jacobian);
        }
    }

    matrix3 multiply_matrix3(const matrix3& left, const matrix3& right)
    {
        return multiply(left, right);
    }

    float determinant_matrix3(const matrix3& value)
    {
        return determinant(value);
    }

    corotational_tet_result evaluate_corotational_tetrahedron(
        const physics_mesh& mesh,
        uint32_t tetrahedron_index,
        const std::vector<vec3>& current_positions,
        const soft_body_material& material)
    {
        corotational_tet_result result;
        if (tetrahedron_index >= mesh.tetrahedra.size() ||
            mesh.rest_positions.empty() ||
            mesh.rest_positions.size() != current_positions.size())
        {
            add_diagnostic(result, soft_body_error::invalid_mesh, tetrahedron_index,
                "Physics Mesh, tetrahedron index or current position count is invalid.");
            return result;
        }
        if (!valid_material(material))
        {
            add_diagnostic(result, soft_body_error::invalid_material, tetrahedron_index,
                "Soft Body material parameters must be finite and physically valid.");
            return result;
        }

        const tetrahedron& tet = mesh.tetrahedra[tetrahedron_index];
        for (uint32_t vertex_id : tet.vertices)
        {
            if (vertex_id >= current_positions.size() || !finite(current_positions[vertex_id]))
            {
                add_diagnostic(result, soft_body_error::invalid_state, tetrahedron_index,
                    "Current Physics Vertex positions contain an invalid index or non-finite value.");
                return result;
            }
        }
        if (!std::isfinite(tet.rest_volume) || tet.rest_volume <= 0.0f)
        {
            add_diagnostic(result, soft_body_error::invalid_mesh, tetrahedron_index,
                "Tetrahedron rest volume must be positive and finite.");
            return result;
        }

        result.deformation_gradient = make_deformation_gradient(tet, current_positions);
        if (!finite(result.deformation_gradient))
        {
            add_diagnostic(result, soft_body_error::non_finite_result, tetrahedron_index,
                "Deformation gradient is not finite.");
            return result;
        }

        result.jacobian = determinant(result.deformation_gradient);
        result.current_volume = tet.rest_volume * result.jacobian;
        if (!std::isfinite(result.jacobian) || !std::isfinite(result.current_volume))
        {
            add_diagnostic(result, soft_body_error::non_finite_result, tetrahedron_index,
                "Deformation determinant or current volume is not finite.");
            return result;
        }

        if (result.jacobian > 1.0e-8f &&
            polar_rotation(result.deformation_gradient, result.rotation))
        {
            result.used_svd_fallback = false;
        }
        else
        {
            if (!jacobi_svd_rotation(result.deformation_gradient, result.rotation))
            {
                add_diagnostic(result, soft_body_error::singular_deformation, tetrahedron_index,
                    "Polar decomposition and Jacobi SVD could not construct a finite proper rotation.");
                return result;
            }
            result.used_svd_fallback = true;
        }

        const matrix3 corotated = multiply(
            transpose(result.rotation),
            result.deformation_gradient);
        result.corotational_strain = subtract(corotated, identity_matrix());
        const float trace = at(result.corotational_strain, 0, 0) +
            at(result.corotational_strain, 1, 1) +
            at(result.corotational_strain, 2, 2);
        matrix3 deviatoric = result.corotational_strain;
        for (uint32_t diagonal = 0; diagonal < 3; ++diagonal)
        {
            at(deviatoric, diagonal, diagonal) -= trace / 3.0f;
        }

        const float shear_effective =
            material.shear_modulus * material.shape_recovery_scale;
        const float bulk_effective =
            material.bulk_modulus * material.volume_retention_scale;
        result.strain_energy_density =
            shear_effective * frobenius_squared(deviatoric) +
            0.5f * bulk_effective * trace * trace;

        matrix3 corotational_stress = add(
            scale(deviatoric, 2.0f * shear_effective),
            scale(identity_matrix(), bulk_effective * trace));
        result.first_piola_stress = multiply(
            result.rotation,
            corotational_stress);

        const float inversion_distance =
            std::min(result.jacobian - material.minimum_jacobian, 0.0f);
        result.inversion_energy_density =
            0.5f * material.inversion_stiffness *
            inversion_distance * inversion_distance;
        result.inversion_stress = scale(
            cofactor(result.deformation_gradient),
            material.inversion_stiffness * inversion_distance);

        const matrix3 total_stress = add(
            result.first_piola_stress,
            result.inversion_stress);
        result.elastic_energy = tet.rest_volume *
            (result.strain_energy_density + result.inversion_energy_density);

        for (std::size_t local_index = 0; local_index < 4; ++local_index)
        {
            const vec3 force_density = multiply(
                total_stress,
                tet.rest_shape_gradients[local_index]);
            result.internal_forces[local_index] = {
                -tet.rest_volume * force_density.x,
                -tet.rest_volume * force_density.y,
                -tet.rest_volume * force_density.z
            };
            if (!finite(result.internal_forces[local_index]))
            {
                add_diagnostic(result, soft_body_error::non_finite_result, tetrahedron_index,
                    "Corotational internal force is not finite.");
                return result;
            }
        }

        result.inverted = result.jacobian <= 0.0f;
        result.state_committable = !result.inverted && finite(result.rotation) &&
            determinant(result.rotation) > 0.0f;
        if (result.inverted)
        {
            add_diagnostic(result, soft_body_error::inverted_tetrahedron, tetrahedron_index,
                "Tetrahedron has non-positive deformation determinant; the finite penalty is diagnostic only and the state must not be committed.");
        }

        if (!finite(result.first_piola_stress) ||
            !finite(result.inversion_stress) ||
            !std::isfinite(result.strain_energy_density) ||
            !std::isfinite(result.inversion_energy_density) ||
            !std::isfinite(result.elastic_energy) ||
            !finite(result.rotation))
        {
            add_diagnostic(result, soft_body_error::non_finite_result, tetrahedron_index,
                "Corotational stress or energy is not finite.");
            return result;
        }

        result.success = true;
        return result;
    }
}
