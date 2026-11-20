#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/physics/animation_track.hpp"
#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    enum class physics_mesh_skinning_error : uint8_t
    {
        none,
        invalid_binding,
        missing_skin_weights,
        invalid_skin_weights,
        invalid_skin_pose,
        non_finite_result
    };

    struct physics_mesh_skinning_result
    {
        std::vector<asset::vec3> target_positions;
        std::vector<asset::vec3> target_velocities;
        physics_mesh_skinning_error error =
            physics_mesh_skinning_error::none;
        std::string message;
        bool success = false;
    };

    // CPU double-precision LBS oracle for one Physics Mesh / one Skinned Mesh
    // Node. Skin Matrices map Rest Mesh-local positions to current Mesh-local
    // targets. Their analytic derivatives produce Mesh-local target velocity.
    physics_mesh_skinning_result evaluate_physics_mesh_skinning(
        const physics_mesh& mesh,
        uint32_t expected_mesh_node_index,
        uint32_t expected_skin_index,
        const animation_skin_pose& skin_pose);
}
