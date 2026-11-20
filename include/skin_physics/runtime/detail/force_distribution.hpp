#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::runtime::detail
{
    // Positions are sampled at the beginning of the current Integration Segment.
    // Mass is the immutable Rest Lumped Mass; Fixed vertices are intentionally included.
    struct force_distribution_mesh_view
    {
        const std::vector<asset::vec3>* segment_start_positions = nullptr;
        const std::vector<float>* rest_lumped_masses = nullptr;
    };

    struct force_distribution_weights
    {
        // Every input Mesh shares one denominator for the selected World / Model / Mesh scope.
        std::vector<std::vector<double>> mesh_weights;
        double unnormalized_weight_sum = 0.0;
    };

    inline bool is_finite_position(const asset::vec3& value)
    {
        return std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    // CPU double-precision reference for the normative mass-weighted Wendland C2
    // Force / Impulse distribution. The output is unchanged on every failure path.
    inline bool build_mass_weighted_wendland_c2_weights(
        const asset::vec3& action_position,
        float radius,
        const std::vector<force_distribution_mesh_view>& meshes,
        force_distribution_weights& out_weights,
        std::string& diagnostic)
    {
        diagnostic.clear();
        if (!is_finite_position(action_position))
        {
            diagnostic = "Force / Impulse action position must contain only finite values.";
            return false;
        }
        if (!std::isfinite(radius) || radius <= 0.0f)
        {
            diagnostic = "Force / Impulse radius must be finite and strictly positive.";
            return false;
        }

        const double radius_value = static_cast<double>(radius);
        const double radius_squared = radius_value * radius_value;
        if (!std::isfinite(radius_squared) || radius_squared <= 0.0)
        {
            diagnostic = "Force / Impulse radius squared is not representable.";
            return false;
        }

        force_distribution_weights candidate;
        candidate.mesh_weights.resize(meshes.size());
        double total_unnormalized_weight = 0.0;

        for (std::size_t mesh_index = 0u;
            mesh_index < meshes.size(); ++mesh_index)
        {
            const force_distribution_mesh_view& mesh = meshes[mesh_index];
            if (mesh.segment_start_positions == nullptr ||
                mesh.rest_lumped_masses == nullptr)
            {
                diagnostic = "Force distribution Mesh view contains a null input array.";
                return false;
            }

            const std::vector<asset::vec3>& positions =
                *mesh.segment_start_positions;
            const std::vector<float>& masses = *mesh.rest_lumped_masses;
            if (positions.size() != masses.size())
            {
                diagnostic = "Force distribution Position and Rest Mass counts differ.";
                return false;
            }

            std::vector<double>& weights = candidate.mesh_weights[mesh_index];
            weights.assign(positions.size(), 0.0);

            for (std::size_t vertex_index = 0u;
                vertex_index < positions.size(); ++vertex_index)
            {
                const asset::vec3& position = positions[vertex_index];
                const float mass = masses[vertex_index];
                if (!is_finite_position(position))
                {
                    diagnostic = "Force distribution contains a non-finite Segment-start Position.";
                    return false;
                }
                if (!std::isfinite(mass) || mass < 0.0f)
                {
                    diagnostic = "Force distribution contains an invalid Rest Lumped Mass.";
                    return false;
                }
                if (mass == 0.0f)
                {
                    continue;
                }

                const double dx =
                    static_cast<double>(position.x) -
                    static_cast<double>(action_position.x);
                const double dy =
                    static_cast<double>(position.y) -
                    static_cast<double>(action_position.y);
                const double dz =
                    static_cast<double>(position.z) -
                    static_cast<double>(action_position.z);
                const double distance_squared =
                    dx * dx + dy * dy + dz * dz;
                if (!std::isfinite(distance_squared))
                {
                    diagnostic = "Force distribution produced a non-finite action-point distance.";
                    return false;
                }
                if (distance_squared >= radius_squared)
                {
                    continue;
                }

                const double normalized_distance =
                    std::sqrt(distance_squared) / radius_value;
                if (normalized_distance >= 1.0)
                {
                    continue;
                }

                const double one_minus_s = 1.0 - normalized_distance;
                const double one_minus_s_squared = one_minus_s * one_minus_s;
                const double one_minus_s_fourth =
                    one_minus_s_squared * one_minus_s_squared;
                const double phi =
                    one_minus_s_fourth * (4.0 * normalized_distance + 1.0);
                const double unnormalized_weight =
                    static_cast<double>(mass) * phi;
                if (!std::isfinite(unnormalized_weight) ||
                    unnormalized_weight < 0.0)
                {
                    diagnostic = "Force distribution produced a non-finite Wendland C2 weight.";
                    return false;
                }

                weights[vertex_index] = unnormalized_weight;
                total_unnormalized_weight += unnormalized_weight;
                if (!std::isfinite(total_unnormalized_weight))
                {
                    diagnostic = "Force distribution normalization sum overflowed.";
                    return false;
                }
            }
        }

        if (!std::isfinite(total_unnormalized_weight) ||
            total_unnormalized_weight <= 0.0)
        {
            diagnostic =
                "NO_FORCE_TARGET: no positive mass-weighted Wendland C2 support exists in the selected target scope.";
            return false;
        }

        for (std::vector<double>& mesh_weights : candidate.mesh_weights)
        {
            for (double& weight : mesh_weights)
            {
                weight /= total_unnormalized_weight;
                if (!std::isfinite(weight) || weight < 0.0)
                {
                    diagnostic = "Force distribution normalization produced an invalid vertex weight.";
                    return false;
                }
            }
        }

        candidate.unnormalized_weight_sum = total_unnormalized_weight;
        out_weights = std::move(candidate);
        return true;
    }
}
