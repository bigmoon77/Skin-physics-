#include "skin_physics/asset/importer.hpp"

#include <cmath>
#include <string>

namespace skin_physics::asset
{
    namespace
    {
        bool finite(float value)
        {
            return std::isfinite(value);
        }

        void add_error(
            std::vector<diagnostic>& diagnostics,
            diagnostic_code code,
            const std::string& message)
        {
            diagnostic item;
            item.severity = diagnostic_severity::error;
            item.code = code;
            item.message = message;
            diagnostics.push_back(item);
        }
    }

    std::vector<diagnostic> validate_model(const model& asset)
    {
        std::vector<diagnostic> diagnostics;

        for (std::size_t mesh_index = 0; mesh_index < asset.meshes.size(); ++mesh_index)
        {
            const mesh& current_mesh = asset.meshes[mesh_index];
            const std::string prefix = "Mesh[" + std::to_string(mesh_index) + "] ";

            if (current_mesh.vertices.empty())
            {
                add_error(diagnostics, diagnostic_code::invalid_topology, prefix + "has no vertices.");
                continue;
            }

            if (current_mesh.indices.empty() || current_mesh.indices.size() % 3 != 0)
            {
                add_error(diagnostics, diagnostic_code::invalid_topology, prefix + "must contain triangle indices.");
            }

            for (uint32_t index : current_mesh.indices)
            {
                if (index >= current_mesh.vertices.size())
                {
                    add_error(diagnostics, diagnostic_code::invalid_index, prefix + "contains an out-of-range vertex index.");
                    break;
                }
            }

            for (const submesh& section : current_mesh.submeshes)
            {
                const uint64_t end_index = static_cast<uint64_t>(section.first_index) + section.index_count;
                if (end_index > current_mesh.indices.size() || section.index_count % 3 != 0)
                {
                    add_error(diagnostics, diagnostic_code::invalid_topology, prefix + "contains an invalid submesh range.");
                }

                if (section.material_index != invalid_index && section.material_index >= asset.materials.size())
                {
                    add_error(diagnostics, diagnostic_code::invalid_material, prefix + "references an invalid material.");
                }
            }

            for (const vertex& current_vertex : current_mesh.vertices)
            {
                if (!finite(current_vertex.position.x) ||
                    !finite(current_vertex.position.y) ||
                    !finite(current_vertex.position.z))
                {
                    add_error(diagnostics, diagnostic_code::invalid_attribute, prefix + "contains a non-finite position.");
                    break;
                }

                for (float weight : current_vertex.physics_weights)
                {
                    if (!finite(weight) || weight < 0.0f || weight > 1.0f)
                    {
                        add_error(diagnostics, diagnostic_code::invalid_attribute, prefix + "contains an invalid physics paint weight.");
                        break;
                    }
                }

                if ((current_vertex.attributes & attribute_skinning) != 0)
                {
                    float weight_sum = 0.0f;
                    for (float weight : current_vertex.weights)
                    {
                        if (!finite(weight) || weight < 0.0f)
                        {
                            add_error(diagnostics, diagnostic_code::invalid_attribute, prefix + "contains an invalid skin weight.");
                            break;
                        }
                        weight_sum += weight;
                    }
                    if (weight_sum <= 1.0e-8f || std::abs(weight_sum - 1.0f) > 1.0e-3f)
                    {
                        add_error(diagnostics, diagnostic_code::invalid_attribute, prefix + "has skin weights that are not normalized.");
                    }
                }
            }
        }

        for (std::size_t node_index = 0; node_index < asset.nodes.size(); ++node_index)
        {
            const node& current_node = asset.nodes[node_index];
            if (current_node.parent_index != invalid_index && current_node.parent_index >= asset.nodes.size())
            {
                add_error(diagnostics, diagnostic_code::invalid_transform, "Node has an invalid parent index.");
            }
            if (current_node.mesh_index != invalid_index && current_node.mesh_index >= asset.meshes.size())
            {
                add_error(diagnostics, diagnostic_code::invalid_index, "Node references an invalid mesh.");
            }
            if (current_node.skin_index != invalid_index && current_node.skin_index >= asset.skins.size())
            {
                add_error(diagnostics, diagnostic_code::invalid_skin, "Node references an invalid skin.");
            }
            for (uint32_t child_index : current_node.children)
            {
                if (child_index >= asset.nodes.size())
                {
                    add_error(diagnostics, diagnostic_code::invalid_transform, "Node contains an invalid child index.");
                }
            }
        }

        for (const node& current_node : asset.nodes)
        {
            if (current_node.mesh_index == invalid_index || current_node.skin_index == invalid_index ||
                current_node.mesh_index >= asset.meshes.size() || current_node.skin_index >= asset.skins.size())
            {
                continue;
            }

            const mesh& skinned_mesh = asset.meshes[current_node.mesh_index];
            const skin& node_skin = asset.skins[current_node.skin_index];
            for (const vertex& current_vertex : skinned_mesh.vertices)
            {
                if ((current_vertex.attributes & attribute_skinning) == 0)
                {
                    continue;
                }

                for (std::size_t influence_index = 0; influence_index < max_skin_influences; ++influence_index)
                {
                    if (current_vertex.weights[influence_index] > 0.0f &&
                        current_vertex.joints[influence_index] >= node_skin.joints.size())
                    {
                        add_error(
                            diagnostics,
                            diagnostic_code::invalid_skin,
                            "Skinned vertex references a joint outside the node skin.");
                        break;
                    }
                }
            }
        }

        for (const skin& current_skin : asset.skins)
        {
            if (current_skin.joints.empty())
            {
                add_error(diagnostics, diagnostic_code::invalid_skin, "Skin must contain at least one joint.");
            }
            if (current_skin.inverse_bind_matrices.size() != current_skin.joints.size())
            {
                add_error(diagnostics, diagnostic_code::invalid_skin, "Skin inverse bind matrix count must match joint count.");
            }
            for (uint32_t joint_index : current_skin.joints)
            {
                if (joint_index >= asset.nodes.size())
                {
                    add_error(diagnostics, diagnostic_code::invalid_skin, "Skin references an invalid joint node.");
                }
            }
        }

        for (const animation_clip& clip : asset.animations)
        {
            if (!finite(clip.duration) || clip.duration < 0.0f)
            {
                add_error(diagnostics, diagnostic_code::invalid_animation, "Animation duration is invalid.");
            }
            for (const animation_track& track : clip.tracks)
            {
                if (track.node_index >= asset.nodes.size() || track.component_count == 0 || track.keys.empty())
                {
                    add_error(diagnostics, diagnostic_code::invalid_animation, "Animation track metadata is invalid.");
                    continue;
                }
                float previous_time = -1.0f;
                for (const animation_key& key : track.keys)
                {
                    if (!finite(key.time) || key.time < 0.0f || key.time < previous_time ||
                        key.value.size() != track.component_count)
                    {
                        add_error(diagnostics, diagnostic_code::invalid_animation, "Animation key data is invalid.");
                        break;
                    }
                    if (track.mode == interpolation::cubic_spline &&
                        (key.in_tangent.size() != track.component_count || key.out_tangent.size() != track.component_count))
                    {
                        add_error(diagnostics, diagnostic_code::invalid_animation, "Cubic spline key tangents have invalid dimensions.");
                        break;
                    }
                    previous_time = key.time;
                }
            }
        }

        return diagnostics;
    }
}
