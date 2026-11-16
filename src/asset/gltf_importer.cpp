#include "importer_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cgltf.h"

namespace skin_physics::asset::detail
{
    namespace
    {
        struct cgltf_data_deleter
        {
            void operator()(cgltf_data* data) const
            {
                if (data != nullptr)
                {
                    cgltf_free(data);
                }
            }
        };

        const cgltf_accessor* find_attribute(
            const cgltf_primitive& primitive,
            cgltf_attribute_type type,
            cgltf_int index)
        {
            for (cgltf_size attribute_index = 0; attribute_index < primitive.attributes_count; ++attribute_index)
            {
                const cgltf_attribute& attribute = primitive.attributes[attribute_index];
                if (attribute.type == type && attribute.index == index)
                {
                    return attribute.data;
                }
            }
            return nullptr;
        }

        bool unpack_floats(
            const cgltf_accessor* accessor,
            cgltf_size expected_components,
            std::vector<float>& output)
        {
            if (accessor == nullptr || expected_components == 0 ||
                cgltf_num_components(accessor->type) != expected_components ||
                accessor->count > std::numeric_limits<cgltf_size>::max() / expected_components)
            {
                return false;
            }

            const cgltf_size value_count = accessor->count * expected_components;
            output.resize(static_cast<std::size_t>(value_count));
            if (value_count == 0)
            {
                return true;
            }

            const cgltf_size unpacked_count =
                cgltf_accessor_unpack_floats(accessor, output.data(), value_count);
            return unpacked_count == value_count;
        }

        bool unpack_attribute(
            const cgltf_accessor* accessor,
            cgltf_size expected_components,
            cgltf_size expected_vertices,
            std::vector<float>& output)
        {
            if (!unpack_floats(accessor, expected_components, output))
            {
                return false;
            }
            return accessor->count == expected_vertices;
        }

        uint32_t texture_index(const cgltf_data* data, const cgltf_texture_view& view)
        {
            if (view.texture == nullptr)
            {
                return invalid_index;
            }
            return static_cast<uint32_t>(cgltf_texture_index(data, view.texture));
        }

        vec3 read_vec3(const float* values)
        {
            return { values[0], values[1], values[2] };
        }

        vec4 read_vec4(const float* values)
        {
            return { values[0], values[1], values[2], values[3] };
        }

        bool append_primitive_indices(
            const cgltf_primitive& primitive,
            cgltf_size vertex_count,
            std::vector<uint32_t>& output)
        {
            std::vector<uint32_t> source_indices;
            if (primitive.indices != nullptr)
            {
                source_indices.resize(static_cast<std::size_t>(primitive.indices->count));
                for (cgltf_size index = 0; index < primitive.indices->count; ++index)
                {
                    const cgltf_size value = cgltf_accessor_read_index(primitive.indices, index);
                    if (value >= vertex_count || value > std::numeric_limits<uint32_t>::max())
                    {
                        return false;
                    }
                    source_indices[static_cast<std::size_t>(index)] = static_cast<uint32_t>(value);
                }
            }
            else
            {
                if (vertex_count > std::numeric_limits<uint32_t>::max())
                {
                    return false;
                }
                source_indices.resize(static_cast<std::size_t>(vertex_count));
                for (cgltf_size index = 0; index < vertex_count; ++index)
                {
                    source_indices[static_cast<std::size_t>(index)] = static_cast<uint32_t>(index);
                }
            }

            if (primitive.type == cgltf_primitive_type_triangles)
            {
                if (source_indices.size() % 3 != 0)
                {
                    return false;
                }
                output.insert(output.end(), source_indices.begin(), source_indices.end());
                return true;
            }

            if (primitive.type == cgltf_primitive_type_triangle_strip)
            {
                if (source_indices.size() < 3)
                {
                    return false;
                }
                for (std::size_t index = 2; index < source_indices.size(); ++index)
                {
                    const uint32_t a = source_indices[index - 2];
                    const uint32_t b = source_indices[index - 1];
                    const uint32_t c = source_indices[index];
                    if ((index & 1u) == 0)
                    {
                        output.push_back(a);
                        output.push_back(b);
                    }
                    else
                    {
                        output.push_back(b);
                        output.push_back(a);
                    }
                    output.push_back(c);
                }
                return true;
            }

            if (primitive.type == cgltf_primitive_type_triangle_fan)
            {
                if (source_indices.size() < 3)
                {
                    return false;
                }
                for (std::size_t index = 2; index < source_indices.size(); ++index)
                {
                    output.push_back(source_indices[0]);
                    output.push_back(source_indices[index - 1]);
                    output.push_back(source_indices[index]);
                }
                return true;
            }

            return false;
        }

        bool normalize_skin_weights(vertex& target, const float* joints_0, const float* weights_0,
            const float* joints_1, const float* weights_1)
        {
            struct influence
            {
                uint32_t joint;
                float weight;
            };

            std::vector<influence> influences;
            influences.reserve(8);

            for (uint32_t index = 0; index < 4; ++index)
            {
                const float joint_0 = joints_0[index];
                const float weight_0 = weights_0[index];
                if (!std::isfinite(joint_0) || joint_0 < 0.0f ||
                    joint_0 > static_cast<float>(std::numeric_limits<uint16_t>::max()) ||
                    std::floor(joint_0) != joint_0 || !std::isfinite(weight_0) || weight_0 < 0.0f)
                {
                    return false;
                }
                if (weight_0 > 0.0f)
                {
                    influences.push_back({ static_cast<uint32_t>(joint_0), weight_0 });
                }

                if (joints_1 != nullptr && weights_1 != nullptr)
                {
                    const float joint_1 = joints_1[index];
                    const float weight_1 = weights_1[index];
                    if (!std::isfinite(joint_1) || joint_1 < 0.0f ||
                        joint_1 > static_cast<float>(std::numeric_limits<uint16_t>::max()) ||
                        std::floor(joint_1) != joint_1 || !std::isfinite(weight_1) || weight_1 < 0.0f)
                    {
                        return false;
                    }
                    if (weight_1 > 0.0f)
                    {
                        influences.push_back({ static_cast<uint32_t>(joint_1), weight_1 });
                    }
                }
            }

            for (std::size_t index = 0; index < influences.size(); ++index)
            {
                for (std::size_t other = index + 1; other < influences.size();)
                {
                    if (influences[index].joint == influences[other].joint)
                    {
                        influences[index].weight += influences[other].weight;
                        influences.erase(influences.begin() + static_cast<std::ptrdiff_t>(other));
                    }
                    else
                    {
                        ++other;
                    }
                }
            }

            std::stable_sort(
                influences.begin(),
                influences.end(),
                [](const influence& left, const influence& right)
                {
                    return left.weight > right.weight;
                });

            if (influences.size() > max_skin_influences)
            {
                influences.resize(max_skin_influences);
            }

            float total_weight = 0.0f;
            for (const influence& item : influences)
            {
                total_weight += item.weight;
            }

            if (total_weight <= 1.0e-8f || !std::isfinite(total_weight))
            {
                return false;
            }

            target.joints.fill(0);
            target.weights.fill(0.0f);
            for (std::size_t index = 0; index < influences.size(); ++index)
            {
                target.joints[index] = static_cast<uint16_t>(influences[index].joint);
                target.weights[index] = influences[index].weight / total_weight;
            }
            target.attributes |= attribute_skinning;
            return true;
        }

        bool append_mesh_primitive(
            const cgltf_data* data,
            const cgltf_primitive& primitive,
            mesh& target_mesh,
            import_result& result,
            const std::string& mesh_name)
        {
            if (primitive.has_draco_mesh_compression)
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::unsupported_feature,
                    "KHR_draco_mesh_compression is not enabled in this build: " + mesh_name);
                return false;
            }

            if (primitive.targets_count > 0)
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::unsupported_feature,
                    "Morph target primitives are outside the initial import scope: " + mesh_name);
                return false;
            }

            const cgltf_accessor* position_accessor =
                find_attribute(primitive, cgltf_attribute_type_position, 0);
            if (position_accessor == nullptr || position_accessor->type != cgltf_type_vec3)
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                    "glTF primitive is missing a VEC3 POSITION accessor: " + mesh_name);
                return false;
            }

            const cgltf_size vertex_count = position_accessor->count;
            const cgltf_size base_vertex = static_cast<cgltf_size>(target_mesh.vertices.size());
            if (base_vertex + vertex_count > std::numeric_limits<uint32_t>::max())
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_topology,
                    "glTF mesh exceeds 32-bit vertex index capacity: " + mesh_name);
                return false;
            }

            std::vector<float> positions;
            if (!unpack_attribute(position_accessor, 3, vertex_count, positions))
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                    "Could not decode glTF POSITION accessor: " + mesh_name);
                return false;
            }

            const cgltf_accessor* normal_accessor = find_attribute(primitive, cgltf_attribute_type_normal, 0);
            const cgltf_accessor* tangent_accessor = find_attribute(primitive, cgltf_attribute_type_tangent, 0);
            const cgltf_accessor* texcoord_accessor = find_attribute(primitive, cgltf_attribute_type_texcoord, 0);
            const cgltf_accessor* color_accessor = find_attribute(primitive, cgltf_attribute_type_color, 0);
            const cgltf_accessor* joints_0_accessor = find_attribute(primitive, cgltf_attribute_type_joints, 0);
            const cgltf_accessor* weights_0_accessor = find_attribute(primitive, cgltf_attribute_type_weights, 0);
            const cgltf_accessor* joints_1_accessor = find_attribute(primitive, cgltf_attribute_type_joints, 1);
            const cgltf_accessor* weights_1_accessor = find_attribute(primitive, cgltf_attribute_type_weights, 1);

            std::vector<std::pair<std::string, std::vector<float>>> source_vertex_groups;
            for (cgltf_size attribute_index = 0; attribute_index < primitive.attributes_count; ++attribute_index)
            {
                const cgltf_attribute& source_attribute = primitive.attributes[attribute_index];
                if (source_attribute.type != cgltf_attribute_type_custom || source_attribute.name == nullptr ||
                    source_attribute.data == nullptr)
                {
                    continue;
                }

                if (source_attribute.data->type != cgltf_type_scalar ||
                    source_attribute.data->count != vertex_count)
                {
                    append_diagnostic(result, diagnostic_severity::warning, diagnostic_code::unsupported_feature,
                        std::string("Custom non-scalar vertex attribute was not imported as a vertex group: ") +
                        source_attribute.name);
                    continue;
                }

                std::vector<float> group_weights;
                if (!unpack_attribute(source_attribute.data, 1, vertex_count, group_weights))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                        std::string("Could not decode custom vertex attribute: ") + source_attribute.name);
                    return false;
                }

                for (float weight : group_weights)
                {
                    if (!std::isfinite(weight) || weight < 0.0f || weight > 1.0f)
                    {
                        append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                            std::string("Custom vertex group must contain weights in [0, 1]: ") + source_attribute.name);
                        return false;
                    }
                }

                source_vertex_groups.emplace_back(source_attribute.name, std::move(group_weights));
            }


            std::vector<float> normals;
            std::vector<float> tangents;
            std::vector<float> texcoords;
            std::vector<float> colors;
            std::vector<float> joints_0;
            std::vector<float> weights_0;
            std::vector<float> joints_1;
            std::vector<float> weights_1;

            if (normal_accessor != nullptr &&
                !unpack_attribute(normal_accessor, 3, vertex_count, normals))
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                    "Could not decode glTF NORMAL accessor: " + mesh_name);
                return false;
            }
            if (tangent_accessor != nullptr &&
                !unpack_attribute(tangent_accessor, 4, vertex_count, tangents))
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                    "Could not decode glTF TANGENT accessor: " + mesh_name);
                return false;
            }
            if (texcoord_accessor != nullptr &&
                !unpack_attribute(texcoord_accessor, 2, vertex_count, texcoords))
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                    "Could not decode glTF TEXCOORD_0 accessor: " + mesh_name);
                return false;
            }
            if (color_accessor != nullptr)
            {
                const cgltf_size color_components = cgltf_num_components(color_accessor->type);
                if ((color_components != 3 && color_components != 4) ||
                    !unpack_attribute(color_accessor, color_components, vertex_count, colors))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                        "Could not decode glTF COLOR_0 accessor: " + mesh_name);
                    return false;
                }
            }

            if ((joints_0_accessor == nullptr) != (weights_0_accessor == nullptr) ||
                (joints_1_accessor == nullptr) != (weights_1_accessor == nullptr) ||
                (joints_1_accessor != nullptr && joints_0_accessor == nullptr))
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                    "JOINTS_n and WEIGHTS_n accessors must be supplied in matching pairs: " + mesh_name);
                return false;
            }

            if (joints_0_accessor != nullptr)
            {
                if ((joints_0_accessor->component_type != cgltf_component_type_r_8u &&
                     joints_0_accessor->component_type != cgltf_component_type_r_16u) ||
                    joints_0_accessor->normalized ||
                    !unpack_attribute(joints_0_accessor, 4, vertex_count, joints_0) ||
                    !unpack_attribute(weights_0_accessor, 4, vertex_count, weights_0))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                        "Could not decode glTF JOINTS_0 / WEIGHTS_0 accessors: " + mesh_name);
                    return false;
                }
            }

            if (joints_1_accessor != nullptr)
            {
                if ((joints_1_accessor->component_type != cgltf_component_type_r_8u &&
                     joints_1_accessor->component_type != cgltf_component_type_r_16u) ||
                    joints_1_accessor->normalized ||
                    !unpack_attribute(joints_1_accessor, 4, vertex_count, joints_1) ||
                    !unpack_attribute(weights_1_accessor, 4, vertex_count, weights_1))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                        "Could not decode glTF JOINTS_1 / WEIGHTS_1 accessors: " + mesh_name);
                    return false;
                }
            }

            for (cgltf_size vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
            {
                vertex current;
                const std::size_t source_position = static_cast<std::size_t>(vertex_index * 3);
                current.position = read_vec3(positions.data() + source_position);

                if (normal_accessor != nullptr)
                {
                    const std::size_t source = static_cast<std::size_t>(vertex_index * 3);
                    current.normal = read_vec3(normals.data() + source);
                    current.attributes |= attribute_normal;
                }
                if (tangent_accessor != nullptr)
                {
                    const std::size_t source = static_cast<std::size_t>(vertex_index * 4);
                    current.tangent = read_vec4(tangents.data() + source);
                    current.attributes |= attribute_tangent;
                }
                if (texcoord_accessor != nullptr)
                {
                    const std::size_t source = static_cast<std::size_t>(vertex_index * 2);
                    current.texcoord_0 = { texcoords[source], texcoords[source + 1] };
                    current.attributes |= attribute_texcoord_0;
                }
                if (color_accessor != nullptr)
                {
                    const std::size_t source = static_cast<std::size_t>(
                        vertex_index * cgltf_num_components(color_accessor->type));
                    const cgltf_size components = cgltf_num_components(color_accessor->type);
                    current.color = {
                        colors[source],
                        colors[source + 1],
                        colors[source + 2],
                        components == 4 ? colors[source + 3] : 1.0f
                    };
                    current.attributes |= attribute_color_0;
                }

                if (joints_0_accessor != nullptr)
                {
                    const std::size_t source = static_cast<std::size_t>(vertex_index * 4);
                    const float* source_joints_1 = joints_1_accessor != nullptr
                        ? joints_1.data() + source
                        : nullptr;
                    const float* source_weights_1 = weights_1_accessor != nullptr
                        ? weights_1.data() + source
                        : nullptr;

                    for (uint32_t influence = 0; influence < 4; ++influence)
                    {
                        const float joint_value = joints_0[source + influence];
                        if (!std::isfinite(joint_value) || joint_value < 0.0f ||
                            joint_value > static_cast<float>(std::numeric_limits<uint16_t>::max()))
                        {
                            append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_skin,
                                "glTF joint index exceeds the supported 16-bit range: " + mesh_name);
                            return false;
                        }
                        if (source_joints_1 != nullptr &&
                            (!std::isfinite(source_joints_1[influence]) || source_joints_1[influence] < 0.0f ||
                             source_joints_1[influence] > static_cast<float>(std::numeric_limits<uint16_t>::max())))
                        {
                            append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_skin,
                                "glTF secondary joint index exceeds the supported 16-bit range: " + mesh_name);
                            return false;
                        }
                    }

                    if (!normalize_skin_weights(
                        current,
                        joints_0.data() + source,
                        weights_0.data() + source,
                        source_joints_1,
                        source_weights_1))
                    {
                        append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_skin,
                            "glTF vertex has invalid or empty skin influences: " + mesh_name);
                        return false;
                    }
                }

                target_mesh.vertices.push_back(current);
            }

            // Keep each named vertex group aligned with the combined mesh vertex array.
            for (vertex_group& existing_group : target_mesh.vertex_groups)
            {
                const auto source_group = std::find_if(
                    source_vertex_groups.begin(),
                    source_vertex_groups.end(),
                    [&existing_group](const auto& item)
                    {
                        return item.first == existing_group.name;
                    });

                if (source_group == source_vertex_groups.end())
                {
                    existing_group.weights.resize(
                        static_cast<std::size_t>(base_vertex + vertex_count),
                        0.0f);
                }
                else
                {
                    existing_group.weights.insert(
                        existing_group.weights.end(),
                        source_group->second.begin(),
                        source_group->second.end());
                }
            }

            for (const auto& source_group : source_vertex_groups)
            {
                const auto existing_group = std::find_if(
                    target_mesh.vertex_groups.begin(),
                    target_mesh.vertex_groups.end(),
                    [&source_group](const vertex_group& item)
                    {
                        return item.name == source_group.first;
                    });
                if (existing_group == target_mesh.vertex_groups.end())
                {
                    vertex_group new_group;
                    new_group.name = source_group.first;
                    new_group.weights.resize(static_cast<std::size_t>(base_vertex), 0.0f);
                    new_group.weights.insert(
                        new_group.weights.end(),
                        source_group.second.begin(),
                        source_group.second.end());
                    target_mesh.vertex_groups.push_back(std::move(new_group));
                }
            }

            const uint32_t first_index = static_cast<uint32_t>(target_mesh.indices.size());
            if (!append_primitive_indices(primitive, vertex_count, target_mesh.indices))
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_topology,
                    "Only valid triangle, triangle-strip and triangle-fan primitives are supported: " + mesh_name);
                return false;
            }

            const uint32_t base_vertex_index = static_cast<uint32_t>(base_vertex);
            for (std::size_t index = first_index; index < target_mesh.indices.size(); ++index)
            {
                target_mesh.indices[index] += base_vertex_index;
            }

            submesh section;
            section.name = mesh_name + "/primitive_" + std::to_string(target_mesh.submeshes.size());
            section.first_index = first_index;
            section.index_count = static_cast<uint32_t>(target_mesh.indices.size()) - first_index;
            section.material_index = primitive.material == nullptr
                ? invalid_index
                : static_cast<uint32_t>(cgltf_material_index(data, primitive.material));
            target_mesh.submeshes.push_back(section);
            return true;
        }

        bool read_animation_sampler(
            const cgltf_animation_sampler& source_sampler,
            animation_track& target_track,
            float& duration)
        {
            if (source_sampler.input == nullptr || source_sampler.output == nullptr ||
                source_sampler.input->type != cgltf_type_scalar)
            {
                return false;
            }

            cgltf_size component_count = 0;
            if (target_track.path == animation_path::translation || target_track.path == animation_path::scale)
            {
                component_count = 3;
            }
            else
            {
                component_count = 4;
            }

            if (cgltf_num_components(source_sampler.output->type) != component_count)
            {
                return false;
            }

            std::vector<float> times;
            std::vector<float> values;
            if (!unpack_floats(source_sampler.input, 1, times) ||
                !unpack_floats(source_sampler.output, component_count, values))
            {
                return false;
            }

            const interpolation mode = source_sampler.interpolation == cgltf_interpolation_type_step
                ? interpolation::step
                : (source_sampler.interpolation == cgltf_interpolation_type_cubic_spline
                    ? interpolation::cubic_spline
                    : interpolation::linear);

            const cgltf_size multiplier = mode == interpolation::cubic_spline ? 3 : 1;
            if (source_sampler.output->count != source_sampler.input->count * multiplier)
            {
                return false;
            }

            target_track.mode = mode;
            target_track.component_count = static_cast<uint32_t>(component_count);
            target_track.keys.reserve(static_cast<std::size_t>(source_sampler.input->count));

            for (cgltf_size key_index = 0; key_index < source_sampler.input->count; ++key_index)
            {
                animation_key key;
                key.time = times[static_cast<std::size_t>(key_index)];
                if (!std::isfinite(key.time) || key.time < 0.0f)
                {
                    return false;
                }

                const std::size_t base = static_cast<std::size_t>(key_index * multiplier * component_count);
                if (mode == interpolation::cubic_spline)
                {
                    key.in_tangent.assign(
                        values.begin() + static_cast<std::ptrdiff_t>(base),
                        values.begin() + static_cast<std::ptrdiff_t>(base + component_count));
                    key.value.assign(
                        values.begin() + static_cast<std::ptrdiff_t>(base + component_count),
                        values.begin() + static_cast<std::ptrdiff_t>(base + component_count * 2));
                    key.out_tangent.assign(
                        values.begin() + static_cast<std::ptrdiff_t>(base + component_count * 2),
                        values.begin() + static_cast<std::ptrdiff_t>(base + component_count * 3));
                }
                else
                {
                    key.value.assign(
                        values.begin() + static_cast<std::ptrdiff_t>(base),
                        values.begin() + static_cast<std::ptrdiff_t>(base + component_count));
                }

                if (!target_track.keys.empty() && key.time < target_track.keys.back().time)
                {
                    return false;
                }

                duration = std::max(duration, key.time);
                target_track.keys.push_back(std::move(key));
            }

            return !target_track.keys.empty();
        }
    }

    import_result import_gltf(const std::filesystem::path& path, const import_options&)
    {
        import_result result;

        cgltf_options parser_options = {};
        cgltf_data* raw_data = nullptr;
        const std::string file_path = path.string();
        const cgltf_result parse_result = cgltf_parse_file(&parser_options, file_path.c_str(), &raw_data);
        if (parse_result != cgltf_result_success || raw_data == nullptr)
        {
            append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_text,
                "cgltf could not parse the input file. Result code: " + std::to_string(static_cast<int>(parse_result)));
            return result;
        }

        std::unique_ptr<cgltf_data, cgltf_data_deleter> data(raw_data);

        const cgltf_result buffer_result = cgltf_load_buffers(&parser_options, data.get(), file_path.c_str());
        if (buffer_result != cgltf_result_success)
        {
            append_diagnostic(result, diagnostic_severity::error, diagnostic_code::io_error,
                "Could not load glTF buffers. Result code: " + std::to_string(static_cast<int>(buffer_result)));
            return result;
        }

        const cgltf_result validation_result = cgltf_validate(data.get());
        if (validation_result != cgltf_result_success)
        {
            append_diagnostic(result, diagnostic_severity::error, diagnostic_code::validation_failed,
                "cgltf validation failed. Result code: " + std::to_string(static_cast<int>(validation_result)));
            return result;
        }

        if (data->extensions_required_count > 0)
        {
            const char* extension_name = data->extensions_required[0];
            append_diagnostic(result, diagnostic_severity::error, diagnostic_code::unsupported_feature,
                std::string("Required glTF extension is not implemented by the initial importer: ") +
                (extension_name != nullptr ? extension_name : "<unnamed>"));
            return result;
        }

        for (cgltf_size view_index = 0; view_index < data->buffer_views_count; ++view_index)
        {
            if (data->buffer_views[view_index].has_meshopt_compression)
            {
                append_diagnostic(result, diagnostic_severity::error, diagnostic_code::unsupported_feature,
                    "Meshopt-compressed buffer views require an external decoder that is not bundled.");
                return result;
            }
        }

        result.asset.source_path = file_path;

        for (cgltf_size sampler_index = 0; sampler_index < data->samplers_count; ++sampler_index)
        {
            const cgltf_sampler& source_sampler = data->samplers[sampler_index];
            sampler target_sampler;
            target_sampler.mag_filter = static_cast<int32_t>(source_sampler.mag_filter);
            target_sampler.min_filter = static_cast<int32_t>(source_sampler.min_filter);
            target_sampler.wrap_s = static_cast<int32_t>(source_sampler.wrap_s);
            target_sampler.wrap_t = static_cast<int32_t>(source_sampler.wrap_t);
            result.asset.samplers.push_back(target_sampler);
        }

        for (cgltf_size image_index = 0; image_index < data->images_count; ++image_index)
        {
            const cgltf_image& source_image = data->images[image_index];
            image target_image;
            target_image.name = source_image.name != nullptr ? source_image.name : "";
            target_image.uri = source_image.uri != nullptr ? source_image.uri : "";
            if (!target_image.uri.empty() && target_image.uri.rfind("data:", 0) != 0)
            {
                target_image.uri = (path.parent_path() / target_image.uri).lexically_normal().string();
            }
            target_image.mime_type = source_image.mime_type != nullptr ? source_image.mime_type : "";

            if (source_image.buffer_view != nullptr)
            {
                const uint8_t* bytes = cgltf_buffer_view_data(source_image.buffer_view);
                if (bytes == nullptr)
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::io_error,
                        "Could not access embedded glTF image data.");
                    return result;
                }
                target_image.encoded_data.assign(
                    bytes,
                    bytes + static_cast<std::size_t>(source_image.buffer_view->size));
            }

            result.asset.images.push_back(std::move(target_image));
        }

        for (cgltf_size texture_index_value = 0; texture_index_value < data->textures_count; ++texture_index_value)
        {
            const cgltf_texture& source_texture = data->textures[texture_index_value];
            texture target_texture;
            target_texture.name = source_texture.name != nullptr ? source_texture.name : "";

            const cgltf_image* selected_image = source_texture.image;
            if (selected_image == nullptr && source_texture.has_basisu)
            {
                selected_image = source_texture.basisu_image;
            }
            if (selected_image == nullptr && source_texture.has_webp)
            {
                selected_image = source_texture.webp_image;
            }
            if (selected_image != nullptr)
            {
                target_texture.image_index = static_cast<uint32_t>(cgltf_image_index(data.get(), selected_image));
            }
            if (source_texture.sampler != nullptr)
            {
                target_texture.sampler_index =
                    static_cast<uint32_t>(cgltf_sampler_index(data.get(), source_texture.sampler));
            }
            result.asset.textures.push_back(target_texture);
        }

        for (cgltf_size material_index = 0; material_index < data->materials_count; ++material_index)
        {
            const cgltf_material& source_material = data->materials[material_index];
            material target_material;
            target_material.name = source_material.name != nullptr ? source_material.name : "";

            if (source_material.has_pbr_metallic_roughness)
            {
                target_material.base_color_factor = {
                    source_material.pbr_metallic_roughness.base_color_factor[0],
                    source_material.pbr_metallic_roughness.base_color_factor[1],
                    source_material.pbr_metallic_roughness.base_color_factor[2],
                    source_material.pbr_metallic_roughness.base_color_factor[3]
                };
                target_material.metallic_factor = source_material.pbr_metallic_roughness.metallic_factor;
                target_material.roughness_factor = source_material.pbr_metallic_roughness.roughness_factor;
                target_material.base_color_texture =
                    texture_index(data.get(), source_material.pbr_metallic_roughness.base_color_texture);
                target_material.metallic_roughness_texture =
                    texture_index(data.get(), source_material.pbr_metallic_roughness.metallic_roughness_texture);
            }

            target_material.emissive_factor = read_vec3(source_material.emissive_factor);
            target_material.normal_texture = texture_index(data.get(), source_material.normal_texture);
            target_material.emissive_texture = texture_index(data.get(), source_material.emissive_texture);
            target_material.alpha_cutoff = source_material.alpha_cutoff;
            target_material.double_sided = source_material.double_sided != 0;
            target_material.alpha_blend = source_material.alpha_mode == cgltf_alpha_mode_blend;
            target_material.alpha_mask = source_material.alpha_mode == cgltf_alpha_mode_mask;
            result.asset.materials.push_back(target_material);
        }

        for (cgltf_size mesh_index = 0; mesh_index < data->meshes_count; ++mesh_index)
        {
            const cgltf_mesh& source_mesh = data->meshes[mesh_index];
            mesh target_mesh;
            target_mesh.name = source_mesh.name != nullptr
                ? source_mesh.name
                : "mesh_" + std::to_string(mesh_index);

            for (cgltf_size primitive_index = 0; primitive_index < source_mesh.primitives_count; ++primitive_index)
            {
                if (!append_mesh_primitive(data.get(), source_mesh.primitives[primitive_index],
                    target_mesh, result, target_mesh.name))
                {
                    return result;
                }
            }

            if (!target_mesh.vertices.empty())
            {
                generate_missing_normals_and_tangents(target_mesh);
                result.asset.meshes.push_back(std::move(target_mesh));
            }
            else
            {
                append_diagnostic(result, diagnostic_severity::warning, diagnostic_code::invalid_topology,
                    "glTF mesh has no triangle primitives: " + target_mesh.name);
                result.asset.meshes.push_back(std::move(target_mesh));
            }
        }

        for (cgltf_size node_index = 0; node_index < data->nodes_count; ++node_index)
        {
            const cgltf_node& source_node = data->nodes[node_index];
            node target_node;
            target_node.name = source_node.name != nullptr
                ? source_node.name
                : "node_" + std::to_string(node_index);

            if (source_node.parent != nullptr)
            {
                target_node.parent_index = static_cast<uint32_t>(cgltf_node_index(data.get(), source_node.parent));
            }
            if (source_node.mesh != nullptr)
            {
                target_node.mesh_index = static_cast<uint32_t>(cgltf_mesh_index(data.get(), source_node.mesh));
            }
            if (source_node.skin != nullptr)
            {
                target_node.skin_index = static_cast<uint32_t>(cgltf_skin_index(data.get(), source_node.skin));
            }
            for (cgltf_size child_index = 0; child_index < source_node.children_count; ++child_index)
            {
                target_node.children.push_back(
                    static_cast<uint32_t>(cgltf_node_index(data.get(), source_node.children[child_index])));
            }

            target_node.uses_matrix = source_node.has_matrix != 0;
            cgltf_node_transform_local(&source_node, target_node.local_matrix.value.data());
            if (source_node.has_translation)
            {
                target_node.translation = read_vec3(source_node.translation);
            }
            if (source_node.has_rotation)
            {
                target_node.rotation = read_vec4(source_node.rotation);
            }
            if (source_node.has_scale)
            {
                target_node.scale = read_vec3(source_node.scale);
            }

            result.asset.nodes.push_back(std::move(target_node));
        }

        for (cgltf_size skin_index_value = 0; skin_index_value < data->skins_count; ++skin_index_value)
        {
            const cgltf_skin& source_skin = data->skins[skin_index_value];
            skin target_skin;
            target_skin.name = source_skin.name != nullptr
                ? source_skin.name
                : "skin_" + std::to_string(skin_index_value);

            if (source_skin.skeleton != nullptr)
            {
                target_skin.skeleton_node_index =
                    static_cast<uint32_t>(cgltf_node_index(data.get(), source_skin.skeleton));
            }

            for (cgltf_size joint_index = 0; joint_index < source_skin.joints_count; ++joint_index)
            {
                target_skin.joints.push_back(
                    static_cast<uint32_t>(cgltf_node_index(data.get(), source_skin.joints[joint_index])));
            }

            if (source_skin.inverse_bind_matrices != nullptr)
            {
                std::vector<float> matrices;
                if (!unpack_attribute(source_skin.inverse_bind_matrices, 16,
                    source_skin.joints_count, matrices))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_skin,
                        "Could not decode inverse bind matrices for skin: " + target_skin.name);
                    return result;
                }

                target_skin.inverse_bind_matrices.resize(source_skin.joints_count);
                for (cgltf_size matrix_index = 0; matrix_index < source_skin.joints_count; ++matrix_index)
                {
                    for (std::size_t component = 0; component < 16; ++component)
                    {
                        target_skin.inverse_bind_matrices[static_cast<std::size_t>(matrix_index)].value[component] =
                            matrices[static_cast<std::size_t>(matrix_index * 16 + component)];
                    }
                }
            }
            else
            {
                target_skin.inverse_bind_matrices.resize(source_skin.joints_count);
            }

            result.asset.skins.push_back(std::move(target_skin));
        }

        for (cgltf_size animation_index = 0; animation_index < data->animations_count; ++animation_index)
        {
            const cgltf_animation& source_animation = data->animations[animation_index];
            animation_clip target_clip;
            target_clip.name = source_animation.name != nullptr
                ? source_animation.name
                : "animation_" + std::to_string(animation_index);

            for (cgltf_size channel_index = 0; channel_index < source_animation.channels_count; ++channel_index)
            {
                const cgltf_animation_channel& source_channel = source_animation.channels[channel_index];
                if (source_channel.target_node == nullptr || source_channel.sampler == nullptr)
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_animation,
                        "glTF animation channel has a missing target node or sampler.");
                    return result;
                }

                animation_track target_track;
                target_track.node_index =
                    static_cast<uint32_t>(cgltf_node_index(data.get(), source_channel.target_node));

                if (source_channel.target_path == cgltf_animation_path_type_translation)
                {
                    target_track.path = animation_path::translation;
                }
                else if (source_channel.target_path == cgltf_animation_path_type_rotation)
                {
                    target_track.path = animation_path::rotation;
                }
                else if (source_channel.target_path == cgltf_animation_path_type_scale)
                {
                    target_track.path = animation_path::scale;
                }
                else
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::unsupported_feature,
                        "glTF morph-weight animation channels are outside the initial import scope.");
                    return result;
                }

                if (!read_animation_sampler(
                    *source_channel.sampler,
                    target_track,
                    target_clip.duration))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_animation,
                        "Could not decode glTF animation sampler: " + target_clip.name);
                    return result;
                }

                target_clip.tracks.push_back(std::move(target_track));
            }

            result.asset.animations.push_back(std::move(target_clip));
        }

        if (data->scene != nullptr)
        {
            for (cgltf_size root_index = 0; root_index < data->scene->nodes_count; ++root_index)
            {
                result.asset.scene_roots.push_back(
                    static_cast<uint32_t>(cgltf_node_index(data.get(), data->scene->nodes[root_index])));
            }
        }
        else
        {
            for (cgltf_size node_index = 0; node_index < data->nodes_count; ++node_index)
            {
                if (data->nodes[node_index].parent == nullptr)
                {
                    result.asset.scene_roots.push_back(static_cast<uint32_t>(node_index));
                }
            }
        }

        result.success = true;
        return result;
    }
}
