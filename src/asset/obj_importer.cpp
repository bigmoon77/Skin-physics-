#include "importer_internal.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

namespace skin_physics::asset::detail
{
    namespace
    {
        struct obj_index
        {
            int position = 0;
            int texcoord = 0;
            int normal = 0;
        };

        struct vertex_key
        {
            int position = -1;
            int texcoord = -1;
            int normal = -1;
            uint32_t smoothing_group = 0;

            bool operator==(const vertex_key& other) const
            {
                return position == other.position &&
                    texcoord == other.texcoord &&
                    normal == other.normal &&
                    smoothing_group == other.smoothing_group;
            }
        };

        struct vertex_key_hash
        {
            std::size_t operator()(const vertex_key& key) const
            {
                std::size_t value = static_cast<std::size_t>(key.position + 1);
                value = value * 16777619u ^ static_cast<std::size_t>(key.texcoord + 1);
                value = value * 16777619u ^ static_cast<std::size_t>(key.normal + 1);
                value = value * 16777619u ^ static_cast<std::size_t>(key.smoothing_group);
                return value;
            }
        };

        std::string trim(const std::string& value)
        {
            const std::size_t first = value.find_first_not_of(" \t\r\n");
            if (first == std::string::npos)
            {
                return {};
            }
            const std::size_t last = value.find_last_not_of(" \t\r\n");
            return value.substr(first, last - first + 1);
        }

        bool parse_integer(std::string_view text, int& output)
        {
            if (text.empty())
            {
                return false;
            }

            try
            {
                std::size_t parsed = 0;
                const int value = std::stoi(std::string(text), &parsed);
                if (parsed != text.size())
                {
                    return false;
                }
                output = value;
                return true;
            }
            catch (...)
            {
                return false;
            }
        }

        bool parse_face_index(const std::string& token, obj_index& output)
        {
            const std::size_t first_slash = token.find('/');
            if (first_slash == std::string::npos)
            {
                return parse_integer(token, output.position);
            }

            if (!parse_integer(std::string_view(token).substr(0, first_slash), output.position))
            {
                return false;
            }

            const std::size_t second_slash = token.find('/', first_slash + 1);
            if (second_slash == std::string::npos)
            {
                return parse_integer(std::string_view(token).substr(first_slash + 1), output.texcoord);
            }

            const std::string_view texcoord_text =
                std::string_view(token).substr(first_slash + 1, second_slash - first_slash - 1);
            const std::string_view normal_text = std::string_view(token).substr(second_slash + 1);

            if (!texcoord_text.empty() && !parse_integer(texcoord_text, output.texcoord))
            {
                return false;
            }
            if (!normal_text.empty() && !parse_integer(normal_text, output.normal))
            {
                return false;
            }
            return true;
        }

        bool resolve_obj_index(int raw_index, std::size_t element_count, int& resolved)
        {
            if (raw_index == 0)
            {
                return false;
            }

            const int64_t count = static_cast<int64_t>(element_count);
            const int64_t index = raw_index > 0
                ? static_cast<int64_t>(raw_index) - 1
                : count + static_cast<int64_t>(raw_index);

            if (index < 0 || index >= count)
            {
                return false;
            }

            resolved = static_cast<int>(index);
            return true;
        }

        uint32_t find_or_add_material(model& asset, const std::string& name)
        {
            for (std::size_t index = 0; index < asset.materials.size(); ++index)
            {
                if (asset.materials[index].name == name)
                {
                    return static_cast<uint32_t>(index);
                }
            }

            material new_material;
            new_material.name = name;
            asset.materials.push_back(new_material);
            return static_cast<uint32_t>(asset.materials.size() - 1);
        }

        void load_mtl_file(
            const std::filesystem::path& mtl_path,
            model& asset,
            import_result& result)
        {
            std::ifstream file(mtl_path);
            if (!file)
            {
                append_diagnostic(
                    result,
                    diagnostic_severity::warning,
                    diagnostic_code::io_error,
                    "Could not open material library: " + mtl_path.string());
                return;
            }

            uint32_t active_material = invalid_index;
            std::string line;
            uint32_t line_number = 0;

            while (std::getline(file, line))
            {
                ++line_number;
                std::istringstream stream(line);
                std::string command;
                if (!(stream >> command) || command[0] == '#')
                {
                    continue;
                }

                if (command == "newmtl")
                {
                    std::string name;
                    std::getline(stream, name);
                    name = trim(name);
                    if (name.empty())
                    {
                        append_diagnostic(
                            result,
                            diagnostic_severity::warning,
                            diagnostic_code::invalid_material,
                            "MTL material with an empty name was ignored.",
                            line_number);
                        active_material = invalid_index;
                        continue;
                    }
                    active_material = find_or_add_material(asset, name);
                }
                else if (active_material != invalid_index)
                {
                    material& current = asset.materials[active_material];
                    if (command == "Kd")
                    {
                        stream >> current.base_color_factor.x
                               >> current.base_color_factor.y
                               >> current.base_color_factor.z;
                    }
                    else if (command == "d")
                    {
                        stream >> current.base_color_factor.w;
                    }
                    else if (command == "Tr")
                    {
                        float transparency = 0.0f;
                        stream >> transparency;
                        current.base_color_factor.w = 1.0f - transparency;
                    }
                    else if (command == "Pm")
                    {
                        stream >> current.metallic_factor;
                    }
                    else if (command == "Pr")
                    {
                        stream >> current.roughness_factor;
                    }
                    else if (command == "map_Kd")
                    {
                        std::string map_path;
                        std::getline(stream, map_path);
                        map_path = trim(map_path);
                        if (!map_path.empty())
                        {
                            image texture_image;
                            texture_image.uri = (mtl_path.parent_path() / map_path).lexically_normal().string();
                            asset.images.push_back(texture_image);
                            texture texture_data;
                            texture_data.image_index = static_cast<uint32_t>(asset.images.size() - 1);
                            asset.textures.push_back(texture_data);
                            current.base_color_texture = static_cast<uint32_t>(asset.textures.size() - 1);
                        }
                    }
                }
            }

            if (file.bad())
            {
                append_diagnostic(
                    result,
                    diagnostic_severity::warning,
                    diagnostic_code::io_error,
                    "An error occurred while reading material library: " + mtl_path.string());
            }
        }

        vec3 subtract(const vec3& left, const vec3& right)
        {
            return { left.x - right.x, left.y - right.y, left.z - right.z };
        }

        vec3 add(const vec3& left, const vec3& right)
        {
            return { left.x + right.x, left.y + right.y, left.z + right.z };
        }

        vec3 multiply(const vec3& value, float scale)
        {
            return { value.x * scale, value.y * scale, value.z * scale };
        }

        vec3 cross(const vec3& left, const vec3& right)
        {
            return {
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x
            };
        }

        float dot(const vec3& left, const vec3& right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }

        vec3 normalize(const vec3& value)
        {
            const float length_squared = dot(value, value);
            if (length_squared <= 1.0e-20f || !std::isfinite(length_squared))
            {
                return { 0.0f, 1.0f, 0.0f };
            }
            return multiply(value, 1.0f / std::sqrt(length_squared));
        }
    }

    void append_diagnostic(
        import_result& result,
        diagnostic_severity severity,
        diagnostic_code code,
        const std::string& message,
        uint32_t line)
    {
        diagnostic item;
        item.severity = severity;
        item.code = code;
        item.message = message;
        item.line = line;
        result.diagnostics.push_back(item);
    }

    void generate_missing_normals_and_tangents(mesh& target)
    {
        std::vector<vec3> normal_sums(target.vertices.size());
        std::vector<vec3> tangent_sums(target.vertices.size());
        std::vector<vec3> bitangent_sums(target.vertices.size());
        std::vector<bool> needs_normal(target.vertices.size(), false);
        std::vector<bool> needs_tangent(target.vertices.size(), false);

        for (std::size_t index = 0; index < target.vertices.size(); ++index)
        {
            needs_normal[index] = (target.vertices[index].attributes & attribute_normal) == 0;
            needs_tangent[index] =
                (target.vertices[index].attributes & attribute_tangent) == 0 &&
                (target.vertices[index].attributes & attribute_texcoord_0) != 0;
        }

        for (std::size_t index = 0; index + 2 < target.indices.size(); index += 3)
        {
            const uint32_t i0 = target.indices[index];
            const uint32_t i1 = target.indices[index + 1];
            const uint32_t i2 = target.indices[index + 2];

            if (i0 >= target.vertices.size() || i1 >= target.vertices.size() || i2 >= target.vertices.size())
            {
                continue;
            }

            const vertex& v0 = target.vertices[i0];
            const vertex& v1 = target.vertices[i1];
            const vertex& v2 = target.vertices[i2];

            const vec3 edge_1 = subtract(v1.position, v0.position);
            const vec3 edge_2 = subtract(v2.position, v0.position);
            const vec3 face_normal = cross(edge_1, edge_2);

            if (needs_normal[i0]) normal_sums[i0] = add(normal_sums[i0], face_normal);
            if (needs_normal[i1]) normal_sums[i1] = add(normal_sums[i1], face_normal);
            if (needs_normal[i2]) normal_sums[i2] = add(normal_sums[i2], face_normal);

            if (needs_tangent[i0] && needs_tangent[i1] && needs_tangent[i2])
            {
                const float du1 = v1.texcoord_0.x - v0.texcoord_0.x;
                const float dv1 = v1.texcoord_0.y - v0.texcoord_0.y;
                const float du2 = v2.texcoord_0.x - v0.texcoord_0.x;
                const float dv2 = v2.texcoord_0.y - v0.texcoord_0.y;
                const float determinant = du1 * dv2 - du2 * dv1;

                if (std::abs(determinant) > 1.0e-12f)
                {
                    const float reciprocal = 1.0f / determinant;
                    const vec3 tangent = multiply(subtract(multiply(edge_1, dv2), multiply(edge_2, dv1)), reciprocal);
                    const vec3 bitangent = multiply(subtract(multiply(edge_2, du1), multiply(edge_1, du2)), reciprocal);

                    tangent_sums[i0] = add(tangent_sums[i0], tangent);
                    tangent_sums[i1] = add(tangent_sums[i1], tangent);
                    tangent_sums[i2] = add(tangent_sums[i2], tangent);
                    bitangent_sums[i0] = add(bitangent_sums[i0], bitangent);
                    bitangent_sums[i1] = add(bitangent_sums[i1], bitangent);
                    bitangent_sums[i2] = add(bitangent_sums[i2], bitangent);
                }
            }
        }

        for (std::size_t index = 0; index < target.vertices.size(); ++index)
        {
            vertex& current = target.vertices[index];

            if (needs_normal[index])
            {
                current.normal = normalize(normal_sums[index]);
                current.attributes |= attribute_normal;
            }

            if (needs_tangent[index])
            {
                const vec3 normal = normalize(current.normal);
                vec3 tangent = tangent_sums[index];
                tangent = subtract(tangent, multiply(normal, dot(normal, tangent)));

                if (dot(tangent, tangent) <= 1.0e-20f)
                {
                    const vec3 axis = std::abs(normal.y) < 0.9f
                        ? vec3{ 0.0f, 1.0f, 0.0f }
                        : vec3{ 1.0f, 0.0f, 0.0f };
                    tangent = cross(axis, normal);
                }

                tangent = normalize(tangent);
                const float handedness = dot(cross(normal, tangent), bitangent_sums[index]) < 0.0f
                    ? -1.0f
                    : 1.0f;
                current.tangent = { tangent.x, tangent.y, tangent.z, handedness };
                current.attributes |= attribute_tangent;
            }
        }
    }

    import_result import_obj(const std::filesystem::path& path, const import_options& options)
    {
        import_result result;

        if (!options.has_obj_unit_scale ||
            !std::isfinite(options.obj_unit_scale) ||
            options.obj_unit_scale <= 0.0f)
        {
            append_diagnostic(
                result,
                diagnostic_severity::error,
                diagnostic_code::invalid_unit_scale,
                "OBJ has no unit metadata. Set import_options::has_obj_unit_scale and a positive obj_unit_scale in meters.");
            return result;
        }

        std::ifstream file(path);
        if (!file)
        {
            append_diagnostic(
                result,
                diagnostic_severity::error,
                diagnostic_code::file_not_found,
                "Could not open OBJ file: " + path.string());
            return result;
        }

        result.asset.source_path = path.string();
        mesh imported_mesh;
        imported_mesh.name = path.stem().string();

        std::vector<vec3> positions;
        std::vector<vec3> normals;
        std::vector<vec2> texcoords;
        std::unordered_map<vertex_key, uint32_t, vertex_key_hash> vertex_map;
        uint32_t smoothing_group = 0;
        uint32_t face_number = 0;
        uint32_t current_material = invalid_index;
        std::string object_name = imported_mesh.name;
        std::string group_name;
        std::string current_section_name;
        uint32_t section_start = 0;
        bool section_active = false;
        std::string line;
        uint32_t line_number = 0;

        const auto close_section = [&]()
        {
            if (section_active)
            {
                const uint32_t index_count = static_cast<uint32_t>(imported_mesh.indices.size()) - section_start;
                if (index_count > 0)
                {
                    submesh section;
                    section.name = current_section_name;
                    section.first_index = section_start;
                    section.index_count = index_count;
                    section.material_index = current_material;
                    imported_mesh.submeshes.push_back(section);
                }
                section_active = false;
            }
        };

        const auto start_section = [&]()
        {
            if (!section_active)
            {
                current_section_name = object_name;
                if (!group_name.empty())
                {
                    current_section_name += "/" + group_name;
                }
                if (current_material != invalid_index && current_material < result.asset.materials.size())
                {
                    current_section_name += "/" + result.asset.materials[current_material].name;
                }
                section_start = static_cast<uint32_t>(imported_mesh.indices.size());
                section_active = true;
            }
        };

        const auto get_or_create_vertex = [&](const obj_index& input_index, uint32_t active_face) -> uint32_t
        {
            int position_index = -1;
            int texcoord_index = -1;
            int normal_index = -1;

            if (!resolve_obj_index(input_index.position, positions.size(), position_index))
            {
                throw std::runtime_error("OBJ position index is out of range.");
            }
            if (input_index.texcoord != 0 &&
                !resolve_obj_index(input_index.texcoord, texcoords.size(), texcoord_index))
            {
                throw std::runtime_error("OBJ texture coordinate index is out of range.");
            }
            if (input_index.normal != 0 &&
                !resolve_obj_index(input_index.normal, normals.size(), normal_index))
            {
                throw std::runtime_error("OBJ normal index is out of range.");
            }

            vertex_key key;
            key.position = position_index;
            key.texcoord = texcoord_index;
            key.normal = normal_index;
            key.smoothing_group = normal_index >= 0
                ? 0
                : (smoothing_group == 0 ? active_face : smoothing_group);

            const auto found = vertex_map.find(key);
            if (found != vertex_map.end())
            {
                return found->second;
            }

            vertex new_vertex;
            new_vertex.position = positions[static_cast<std::size_t>(position_index)];
            if (texcoord_index >= 0)
            {
                new_vertex.texcoord_0 = texcoords[static_cast<std::size_t>(texcoord_index)];
                new_vertex.attributes |= attribute_texcoord_0;
            }
            if (normal_index >= 0)
            {
                new_vertex.normal = normals[static_cast<std::size_t>(normal_index)];
                new_vertex.attributes |= attribute_normal;
            }

            const uint32_t new_index = static_cast<uint32_t>(imported_mesh.vertices.size());
            imported_mesh.vertices.push_back(new_vertex);
            vertex_map.emplace(key, new_index);
            return new_index;
        };

        while (std::getline(file, line))
        {
            ++line_number;
            const std::size_t comment_position = line.find('#');
            if (comment_position != std::string::npos)
            {
                line.erase(comment_position);
            }

            std::istringstream stream(line);
            std::string command;
            if (!(stream >> command))
            {
                continue;
            }

            if (command == "v")
            {
                vec3 position;
                float homogeneous_w = 1.0f;
                if (!(stream >> position.x >> position.y >> position.z))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_text,
                        "Malformed vertex position.", line_number);
                    return result;
                }
                if (stream >> homogeneous_w)
                {
                    if (std::abs(homogeneous_w) <= 1.0e-12f)
                    {
                        append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_attribute,
                            "OBJ vertex homogeneous coordinate must not be zero.", line_number);
                        return result;
                    }
                    position = multiply(position, 1.0f / homogeneous_w);
                }
                positions.push_back(multiply(position, options.obj_unit_scale));
            }
            else if (command == "vt")
            {
                vec2 texcoord;
                if (!(stream >> texcoord.x >> texcoord.y))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_text,
                        "Malformed texture coordinate.", line_number);
                    return result;
                }
                texcoords.push_back(texcoord);
            }
            else if (command == "vn")
            {
                vec3 normal;
                if (!(stream >> normal.x >> normal.y >> normal.z))
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_text,
                        "Malformed vertex normal.", line_number);
                    return result;
                }
                normals.push_back(normalize(normal));
            }
            else if (command == "f")
            {
                std::vector<obj_index> face;
                std::string token;
                while (stream >> token)
                {
                    obj_index index;
                    if (!parse_face_index(token, index))
                    {
                        append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_text,
                            "Malformed face index: " + token, line_number);
                        return result;
                    }
                    face.push_back(index);
                }

                if (face.size() < 3)
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_topology,
                        "A face must contain at least three vertices.", line_number);
                    return result;
                }

                ++face_number;
                start_section();

                try
                {
                    const uint32_t first_vertex = get_or_create_vertex(face[0], face_number);
                    for (std::size_t index = 1; index + 1 < face.size(); ++index)
                    {
                        const uint32_t second_vertex = get_or_create_vertex(face[index], face_number);
                        const uint32_t third_vertex = get_or_create_vertex(face[index + 1], face_number);
                        imported_mesh.indices.push_back(first_vertex);
                        imported_mesh.indices.push_back(second_vertex);
                        imported_mesh.indices.push_back(third_vertex);
                    }
                }
                catch (const std::exception& exception)
                {
                    append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_index,
                        exception.what(), line_number);
                    return result;
                }
            }
            else if (command == "o")
            {
                close_section();
                std::getline(stream, object_name);
                object_name = trim(object_name);
                if (object_name.empty())
                {
                    object_name = path.stem().string();
                }
            }
            else if (command == "g")
            {
                close_section();
                std::getline(stream, group_name);
                group_name = trim(group_name);
            }
            else if (command == "usemtl")
            {
                close_section();
                std::string material_name;
                std::getline(stream, material_name);
                material_name = trim(material_name);
                current_material = material_name.empty()
                    ? invalid_index
                    : find_or_add_material(result.asset, material_name);
            }
            else if (command == "mtllib")
            {
                std::string library_name;
                while (stream >> library_name)
                {
                    load_mtl_file(path.parent_path() / library_name, result.asset, result);
                }
            }
            else if (command == "s")
            {
                std::string smoothing;
                stream >> smoothing;
                if (smoothing.empty() || smoothing == "off" || smoothing == "0")
                {
                    smoothing_group = 0;
                }
                else
                {
                    try
                    {
                        smoothing_group = static_cast<uint32_t>(std::stoul(smoothing));
                    }
                    catch (...)
                    {
                        smoothing_group = static_cast<uint32_t>(std::hash<std::string>{}(smoothing));
                        if (smoothing_group == 0)
                        {
                            smoothing_group = 1;
                        }
                    }
                }
            }
        }

        close_section();

        if (file.bad())
        {
            append_diagnostic(result, diagnostic_severity::error, diagnostic_code::io_error,
                "An error occurred while reading OBJ file.");
            return result;
        }

        if (imported_mesh.vertices.empty() || imported_mesh.indices.empty())
        {
            append_diagnostic(result, diagnostic_severity::error, diagnostic_code::invalid_topology,
                "OBJ file does not contain any triangle mesh geometry.");
            return result;
        }

        generate_missing_normals_and_tangents(imported_mesh);
        result.asset.meshes.push_back(std::move(imported_mesh));
        result.success = true;
        return result;
    }
}
