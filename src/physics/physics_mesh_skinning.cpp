#include "skin_physics/physics/physics_mesh_skinning.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace skin_physics::physics
{
    namespace
    {
        bool finite_matrix(const asset::mat4& matrix)
        {
            for (float value : matrix.value)
            {
                if (!std::isfinite(value))
                {
                    return false;
                }
            }
            return true;
        }

        bool finite_vec3(const asset::vec3& value)
        {
            return std::isfinite(value.x) &&
                std::isfinite(value.y) &&
                std::isfinite(value.z);
        }

        bool to_finite_float(double value, float& output)
        {
            if (!std::isfinite(value) ||
                std::abs(value) >
                    static_cast<double>(std::numeric_limits<float>::max()))
            {
                return false;
            }
            output = static_cast<float>(value);
            return std::isfinite(output);
        }

        std::array<double, 3> transform_point(
            const asset::mat4& matrix,
            const asset::vec3& point)
        {
            return {
                static_cast<double>(matrix.value[0]) * point.x +
                    static_cast<double>(matrix.value[4]) * point.y +
                    static_cast<double>(matrix.value[8]) * point.z +
                    matrix.value[12],
                static_cast<double>(matrix.value[1]) * point.x +
                    static_cast<double>(matrix.value[5]) * point.y +
                    static_cast<double>(matrix.value[9]) * point.z +
                    matrix.value[13],
                static_cast<double>(matrix.value[2]) * point.x +
                    static_cast<double>(matrix.value[6]) * point.y +
                    static_cast<double>(matrix.value[10]) * point.z +
                    matrix.value[14]
            };
        }

        physics_mesh_skinning_result make_error(
            physics_mesh_skinning_error error,
            const std::string& message)
        {
            physics_mesh_skinning_result result;
            result.error = error;
            result.message = message;
            return result;
        }
    }

    physics_mesh_skinning_result evaluate_physics_mesh_skinning(
        const physics_mesh& mesh,
        uint32_t expected_mesh_node_index,
        uint32_t expected_skin_index,
        const animation_skin_pose& skin_pose)
    {
        if (expected_mesh_node_index == asset::invalid_index ||
            expected_skin_index == asset::invalid_index ||
            skin_pose.mesh_node_index != expected_mesh_node_index ||
            skin_pose.skin_index != expected_skin_index)
        {
            return make_error(
                physics_mesh_skinning_error::invalid_binding,
                "Skin Pose Mesh Node / Skin identity does not match the Physics Mesh binding.");
        }

        if (mesh.rest_positions.empty() ||
            !mesh.has_skin_weights ||
            mesh.skin_weights.size() != mesh.rest_positions.size())
        {
            return make_error(
                physics_mesh_skinning_error::missing_skin_weights,
                "Physics Mesh Skinning requires one transferred Physics Skin Weight record per Rest Vertex.");
        }

        if (skin_pose.skin_matrices.empty() ||
            skin_pose.skin_matrices.size() !=
                skin_pose.skin_matrix_derivatives.size())
        {
            return make_error(
                physics_mesh_skinning_error::invalid_skin_pose,
                "Skin Pose must contain matching non-empty Skin Matrix and analytic derivative arrays.");
        }

        for (std::size_t joint_index = 0u;
            joint_index < skin_pose.skin_matrices.size(); ++joint_index)
        {
            if (!finite_matrix(skin_pose.skin_matrices[joint_index]) ||
                !finite_matrix(skin_pose.skin_matrix_derivatives[joint_index]))
            {
                return make_error(
                    physics_mesh_skinning_error::invalid_skin_pose,
                    "Skin Pose contains a non-finite Skin Matrix or Matrix Derivative.");
            }
        }

        physics_mesh_skinning_result result;
        result.target_positions.resize(mesh.rest_positions.size());
        result.target_velocities.resize(mesh.rest_positions.size());

        for (std::size_t vertex_index = 0u;
            vertex_index < mesh.rest_positions.size(); ++vertex_index)
        {
            const asset::vec3& rest_position =
                mesh.rest_positions[vertex_index];
            const physics_skin_weight& skin_weights =
                mesh.skin_weights[vertex_index];
            if (!finite_vec3(rest_position))
            {
                return make_error(
                    physics_mesh_skinning_error::invalid_skin_weights,
                    "Physics Mesh Rest Position contains a non-finite component.");
            }

            std::array<double, 3> target_position = {};
            std::array<double, 3> target_velocity = {};
            double weight_sum = 0.0;

            for (std::size_t influence_index = 0u;
                influence_index < asset::max_skin_influences;
                ++influence_index)
            {
                const float weight = skin_weights.weights[influence_index];
                const uint16_t joint_index =
                    skin_weights.joints[influence_index];
                if (!std::isfinite(weight) || weight < 0.0f)
                {
                    return make_error(
                        physics_mesh_skinning_error::invalid_skin_weights,
                        "Physics Mesh Skin Weight contains a negative or non-finite influence.");
                }
                if (weight == 0.0f)
                {
                    continue;
                }
                if (joint_index >= skin_pose.skin_matrices.size())
                {
                    return make_error(
                        physics_mesh_skinning_error::invalid_skin_weights,
                        "Physics Mesh Skin Weight references a Joint outside the ordered Skin Pose.");
                }

                const std::array<double, 3> transformed_position =
                    transform_point(
                        skin_pose.skin_matrices[joint_index], rest_position);
                const std::array<double, 3> transformed_velocity =
                    transform_point(
                        skin_pose.skin_matrix_derivatives[joint_index],
                        rest_position);
                for (std::size_t axis = 0u; axis < 3u; ++axis)
                {
                    target_position[axis] +=
                        static_cast<double>(weight) *
                        transformed_position[axis];
                    target_velocity[axis] +=
                        static_cast<double>(weight) *
                        transformed_velocity[axis];
                }
                weight_sum += static_cast<double>(weight);
            }

            if (!std::isfinite(weight_sum) ||
                std::abs(weight_sum - 1.0) > 1.0e-3)
            {
                return make_error(
                    physics_mesh_skinning_error::invalid_skin_weights,
                    "Physics Mesh Skin Weight influences must sum to one.");
            }

            for (std::size_t axis = 0u; axis < 3u; ++axis)
            {
                target_position[axis] /= weight_sum;
                target_velocity[axis] /= weight_sum;
            }

            asset::vec3& output_position = result.target_positions[vertex_index];
            asset::vec3& output_velocity = result.target_velocities[vertex_index];
            if (!to_finite_float(target_position[0], output_position.x) ||
                !to_finite_float(target_position[1], output_position.y) ||
                !to_finite_float(target_position[2], output_position.z) ||
                !to_finite_float(target_velocity[0], output_velocity.x) ||
                !to_finite_float(target_velocity[1], output_velocity.y) ||
                !to_finite_float(target_velocity[2], output_velocity.z))
            {
                return make_error(
                    physics_mesh_skinning_error::non_finite_result,
                    "Physics Mesh LBS produced a Target Position or Velocity outside finite float range.");
            }
        }

        result.success = true;
        return result;
    }
}
