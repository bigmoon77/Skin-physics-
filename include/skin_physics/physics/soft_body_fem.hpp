#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"
#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    struct matrix3
    {
        // Row-major: value[row * 3 + column].
        std::array<float, 9> value = {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        };
    };

    struct soft_body_material
    {
        float shear_modulus = 1.0e4f;
        float bulk_modulus = 1.0e5f;
        float density = 1000.0f;
        float shape_recovery_scale = 1.0f;
        float volume_retention_scale = 1.0f;
        float inversion_stiffness = 1.0e5f;
        float minimum_jacobian = 0.1f;
    };

    enum class soft_body_error : uint8_t
    {
        none,
        invalid_mesh,
        invalid_element,
        invalid_state,
        invalid_material,
        singular_deformation,
        inverted_tetrahedron,
        non_finite_result
    };

    struct soft_body_diagnostic
    {
        soft_body_error code = soft_body_error::none;
        uint32_t element_index = asset::invalid_index;
        std::string message;
    };

    struct corotational_tet_result
    {
        matrix3 deformation_gradient;
        matrix3 rotation;
        matrix3 corotational_strain;
        matrix3 first_piola_stress;
        matrix3 inversion_stress;
        float jacobian = 0.0f;
        float current_volume = 0.0f;
        float strain_energy_density = 0.0f;
        float inversion_energy_density = 0.0f;
        float elastic_energy = 0.0f;
        std::array<asset::vec3, 4> internal_forces = {};
        bool used_svd_fallback = false;
        bool inverted = false;
        bool state_committable = false;
        bool success = false;
        std::vector<soft_body_diagnostic> diagnostics;
    };

    corotational_tet_result evaluate_corotational_tetrahedron(
        const physics_mesh& mesh,
        uint32_t tetrahedron_index,
        const std::vector<asset::vec3>& current_positions,
        const soft_body_material& material = {});

    matrix3 multiply_matrix3(const matrix3& left, const matrix3& right);
    float determinant_matrix3(const matrix3& value);
}
