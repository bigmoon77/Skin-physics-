#include "skin_physics/physics/render_binding.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace skin_physics::physics
{
    namespace
    {
        using asset::vec2;
        using asset::vec3;
        using asset::vec4;

        constexpr float minimum_length_squared = 1.0e-20f;

        vec3 add(const vec3& left, const vec3& right)
        {
            return { left.x + right.x, left.y + right.y, left.z + right.z };
        }

        vec3 subtract(const vec3& left, const vec3& right)
        {
            return { left.x - right.x, left.y - right.y, left.z - right.z };
        }

        vec3 multiply(const vec3& value, float scale)
        {
            return { value.x * scale, value.y * scale, value.z * scale };
        }

        float dot(const vec3& left, const vec3& right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }

        vec3 cross(const vec3& left, const vec3& right)
        {
            return {
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x
            };
        }

        float length_squared(const vec3& value)
        {
            return dot(value, value);
        }

        float length(const vec3& value)
        {
            return std::sqrt(std::max(0.0f, length_squared(value)));
        }

        vec3 normalize(const vec3& value)
        {
            const float squared = length_squared(value);
            if (!std::isfinite(squared) || squared <= minimum_length_squared)
            {
                return {};
            }
            return multiply(value, 1.0f / std::sqrt(squared));
        }

        bool finite(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        float determinant(const vec3& a, const vec3& b, const vec3& c)
        {
            return dot(a, cross(b, c));
        }

        void add_diagnostic(
            std::vector<render_binding_diagnostic>& diagnostics,
            render_binding_error code,
            uint32_t vertex,
            const std::string& message)
        {
            render_binding_diagnostic item;
            item.code = code;
            item.render_vertex = vertex;
            item.message = message;
            diagnostics.push_back(std::move(item));
        }

        bool tetrahedron_barycentric(
            const vec3& point,
            const std::array<vec3, 4>& positions,
            std::array<float, 4>& output)
        {
            const vec3 e1 = subtract(positions[1], positions[0]);
            const vec3 e2 = subtract(positions[2], positions[0]);
            const vec3 e3 = subtract(positions[3], positions[0]);
            const vec3 relative = subtract(point, positions[0]);
            const float det = determinant(e1, e2, e3);
            if (!std::isfinite(det) || std::abs(det) <= 1.0e-12f)
            {
                return false;
            }

            output[1] = determinant(relative, e2, e3) / det;
            output[2] = determinant(e1, relative, e3) / det;
            output[3] = determinant(e1, e2, relative) / det;
            output[0] = 1.0f - output[1] - output[2] - output[3];
            return std::all_of(output.begin(), output.end(),
                [](float value) { return std::isfinite(value); });
        }

        vec3 interpolate_triangle(
            const std::array<vec3, 3>& positions,
            const std::array<float, 3>& barycentric)
        {
            return add(add(
                multiply(positions[0], barycentric[0]),
                multiply(positions[1], barycentric[1])),
                multiply(positions[2], barycentric[2]));
        }

        // Ericson's closest-point region test. Output weights are non-negative and sum to one.
        bool closest_point_triangle(
            const vec3& point,
            const std::array<vec3, 3>& triangle,
            std::array<float, 3>& barycentric,
            vec3& closest)
        {
            const vec3& a = triangle[0];
            const vec3& b = triangle[1];
            const vec3& c = triangle[2];
            const vec3 ab = subtract(b, a);
            const vec3 ac = subtract(c, a);
            if (length_squared(cross(ab, ac)) <= minimum_length_squared)
            {
                return false;
            }

            const vec3 ap = subtract(point, a);
            const float d1 = dot(ab, ap);
            const float d2 = dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                barycentric = { 1.0f, 0.0f, 0.0f };
                closest = a;
                return true;
            }

            const vec3 bp = subtract(point, b);
            const float d3 = dot(ab, bp);
            const float d4 = dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3)
            {
                barycentric = { 0.0f, 1.0f, 0.0f };
                closest = b;
                return true;
            }

            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                const float v = d1 / (d1 - d3);
                barycentric = { 1.0f - v, v, 0.0f };
                closest = add(a, multiply(ab, v));
                return true;
            }

            const vec3 cp = subtract(point, c);
            const float d5 = dot(ab, cp);
            const float d6 = dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6)
            {
                barycentric = { 0.0f, 0.0f, 1.0f };
                closest = c;
                return true;
            }

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                const float w = d2 / (d2 - d6);
                barycentric = { 1.0f - w, 0.0f, w };
                closest = add(a, multiply(ac, w));
                return true;
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                barycentric = { 0.0f, 1.0f - w, w };
                closest = add(b, multiply(subtract(c, b), w));
                return true;
            }

            const float denominator = 1.0f / (va + vb + vc);
            const float v = vb * denominator;
            const float w = vc * denominator;
            barycentric = { 1.0f - v - w, v, w };
            closest = interpolate_triangle(triangle, barycentric);
            return finite(closest);
        }

        bool basis_coordinates(
            const vec3& value,
            const vec3& e1,
            const vec3& e2,
            const vec3& normal,
            vec3& output)
        {
            const float det = determinant(e1, e2, normal);
            if (!std::isfinite(det) || std::abs(det) <= 1.0e-12f)
            {
                return false;
            }
            output = {
                dot(value, cross(e2, normal)) / det,
                dot(value, cross(normal, e1)) / det,
                dot(value, cross(e1, e2)) / det
            };
            return finite(output);
        }

        float bbox_diagonal(const std::vector<asset::vertex>& vertices)
        {
            if (vertices.empty())
            {
                return 0.0f;
            }

            vec3 minimum = vertices[0].position;
            vec3 maximum = vertices[0].position;
            for (const asset::vertex& vertex : vertices)
            {
                minimum.x = std::min(minimum.x, vertex.position.x);
                minimum.y = std::min(minimum.y, vertex.position.y);
                minimum.z = std::min(minimum.z, vertex.position.z);
                maximum.x = std::max(maximum.x, vertex.position.x);
                maximum.y = std::max(maximum.y, vertex.position.y);
                maximum.z = std::max(maximum.z, vertex.position.z);
            }
            return length(subtract(maximum, minimum));
        }

        vec3 evaluate_binding_position(
            const render_vertex_binding& binding,
            const physics_mesh& physics_value,
            bool& valid)
        {
            valid = false;
            if (binding.kind == render_binding_kind::tetrahedron)
            {
                vec3 result = {};
                for (std::size_t index = 0; index < 4; ++index)
                {
                    const uint32_t vertex_id = binding.physics_vertices[index];
                    if (vertex_id >= physics_value.rest_positions.size())
                    {
                        return {};
                    }
                    result = add(result, multiply(
                        physics_value.rest_positions[vertex_id], binding.barycentric[index]));
                }
                valid = finite(result);
                return result;
            }

            const uint32_t a = binding.physics_vertices[0];
            const uint32_t b = binding.physics_vertices[1];
            const uint32_t c = binding.physics_vertices[2];
            if (a >= physics_value.rest_positions.size() ||
                b >= physics_value.rest_positions.size() ||
                c >= physics_value.rest_positions.size())
            {
                return {};
            }

            const std::array<vec3, 3> triangle = {
                physics_value.rest_positions[a],
                physics_value.rest_positions[b],
                physics_value.rest_positions[c]
            };
            const vec3 q = interpolate_triangle(triangle, {
                binding.barycentric[0], binding.barycentric[1], binding.barycentric[2] });
            const vec3 normal = normalize(cross(
                subtract(triangle[1], triangle[0]),
                subtract(triangle[2], triangle[0])));
            if (length_squared(normal) <= minimum_length_squared)
            {
                return {};
            }
            const vec3 e1 = subtract(triangle[1], triangle[0]);
            const vec3 e2 = subtract(triangle[2], triangle[0]);
            const vec3 reconstructed = add(q, add(add(
                multiply(e1, binding.rest_offset_local.x),
                multiply(e2, binding.rest_offset_local.y)),
                multiply(normal, binding.rest_offset_local.z)));
            valid = finite(reconstructed);
            return reconstructed;
        }

        void update_normals_and_tangents(asset::mesh& mesh)
        {
            std::vector<vec3> normal_sums(mesh.vertices.size());
            std::vector<vec3> tangent_sums(mesh.vertices.size());
            std::vector<vec3> bitangent_sums(mesh.vertices.size());

            for (std::size_t index = 0; index + 2 < mesh.indices.size(); index += 3)
            {
                const uint32_t i0 = mesh.indices[index];
                const uint32_t i1 = mesh.indices[index + 1];
                const uint32_t i2 = mesh.indices[index + 2];
                if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size())
                {
                    continue;
                }

                const asset::vertex& v0 = mesh.vertices[i0];
                const asset::vertex& v1 = mesh.vertices[i1];
                const asset::vertex& v2 = mesh.vertices[i2];
                const vec3 edge1 = subtract(v1.position, v0.position);
                const vec3 edge2 = subtract(v2.position, v0.position);
                const vec3 face_normal = cross(edge1, edge2);
                if (length_squared(face_normal) > minimum_length_squared)
                {
                    normal_sums[i0] = add(normal_sums[i0], face_normal);
                    normal_sums[i1] = add(normal_sums[i1], face_normal);
                    normal_sums[i2] = add(normal_sums[i2], face_normal);
                }

                if (((v0.attributes | v1.attributes | v2.attributes) & asset::attribute_tangent) != 0)
                {
                    const vec2 delta_uv1 = {
                        v1.texcoord_0.x - v0.texcoord_0.x,
                        v1.texcoord_0.y - v0.texcoord_0.y
                    };
                    const vec2 delta_uv2 = {
                        v2.texcoord_0.x - v0.texcoord_0.x,
                        v2.texcoord_0.y - v0.texcoord_0.y
                    };
                    const float determinant_uv =
                        delta_uv1.x * delta_uv2.y - delta_uv1.y * delta_uv2.x;
                    if (std::abs(determinant_uv) > 1.0e-12f)
                    {
                        const float inverse_uv = 1.0f / determinant_uv;
                        const vec3 tangent = multiply(subtract(
                            multiply(edge1, delta_uv2.y),
                            multiply(edge2, delta_uv1.y)), inverse_uv);
                        const vec3 bitangent = multiply(subtract(
                            multiply(edge2, delta_uv1.x),
                            multiply(edge1, delta_uv2.x)), inverse_uv);
                        tangent_sums[i0] = add(tangent_sums[i0], tangent);
                        tangent_sums[i1] = add(tangent_sums[i1], tangent);
                        tangent_sums[i2] = add(tangent_sums[i2], tangent);
                        bitangent_sums[i0] = add(bitangent_sums[i0], bitangent);
                        bitangent_sums[i1] = add(bitangent_sums[i1], bitangent);
                        bitangent_sums[i2] = add(bitangent_sums[i2], bitangent);
                    }
                }
            }

            for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index)
            {
                asset::vertex& vertex = mesh.vertices[vertex_index];
                if ((vertex.attributes & asset::attribute_normal) != 0)
                {
                    vec3 normal = normalize(normal_sums[vertex_index]);
                    if (length_squared(normal) <= minimum_length_squared)
                    {
                        normal = normalize(vertex.normal);
                    }
                    vertex.normal = normal;
                }

                if ((vertex.attributes & asset::attribute_tangent) != 0)
                {
                    const vec3 normal = normalize(vertex.normal);
                    vec3 tangent = tangent_sums[vertex_index];
                    tangent = subtract(tangent, multiply(normal, dot(normal, tangent)));
                    tangent = normalize(tangent);
                    if (length_squared(tangent) <= minimum_length_squared)
                    {
                        const vec3 axis = std::abs(normal.z) < 0.9f
                            ? vec3{ 0.0f, 0.0f, 1.0f }
                            : vec3{ 0.0f, 1.0f, 0.0f };
                        tangent = normalize(cross(axis, normal));
                    }
                    const float handedness = dot(cross(normal, tangent), bitangent_sums[vertex_index]) < 0.0f
                        ? -1.0f : 1.0f;
                    vertex.tangent = { tangent.x, tangent.y, tangent.z, handedness };
                }
            }
        }
    }

    render_binding_build_result build_render_vertex_bindings(
        const asset::mesh& render_mesh,
        const physics_mesh& physics_value,
        const render_binding_settings& settings)
    {
        render_binding_build_result result;
        if (render_mesh.vertices.empty() || render_mesh.indices.size() % 3 != 0)
        {
            add_diagnostic(result.diagnostics, render_binding_error::invalid_render_mesh,
                asset::invalid_index, "Render mesh must contain vertices and triangle-list indices.");
            return result;
        }
        if (physics_value.rest_positions.empty() || physics_value.tetrahedra.empty() ||
            physics_value.boundary_triangles.empty())
        {
            add_diagnostic(result.diagnostics, render_binding_error::invalid_physics_mesh,
                asset::invalid_index, "Physics Mesh must contain tetrahedra and a boundary surface.");
            return result;
        }
        if (!std::isfinite(settings.maximum_surface_distance) ||
            settings.maximum_surface_distance < 0.0f ||
            !std::isfinite(settings.maximum_rest_error_relative_to_bbox_diagonal) ||
            settings.maximum_rest_error_relative_to_bbox_diagonal < 0.0f ||
            !std::isfinite(settings.barycentric_tolerance) ||
            settings.barycentric_tolerance < 0.0f)
        {
            add_diagnostic(result.diagnostics, render_binding_error::invalid_settings,
                asset::invalid_index, "Render binding tolerances must be finite and non-negative.");
            return result;
        }

        const float diagonal = bbox_diagonal(render_mesh.vertices);
        const float maximum_distance_squared =
            settings.maximum_surface_distance * settings.maximum_surface_distance;
        result.bindings.reserve(render_mesh.vertices.size());

        for (std::size_t render_index = 0; render_index < render_mesh.vertices.size(); ++render_index)
        {
            const vec3 render_position = render_mesh.vertices[render_index].position;
            render_vertex_binding binding;
            bool found_inside = false;
            float best_inside_margin = -std::numeric_limits<float>::max();

            for (const tetrahedron& tet : physics_value.tetrahedra)
            {
                std::array<vec3, 4> tet_positions = {};
                bool valid_indices = true;
                for (std::size_t local_index = 0; local_index < 4; ++local_index)
                {
                    const uint32_t vertex_id = tet.vertices[local_index];
                    if (vertex_id >= physics_value.rest_positions.size())
                    {
                        valid_indices = false;
                        break;
                    }
                    tet_positions[local_index] = physics_value.rest_positions[vertex_id];
                }
                if (!valid_indices)
                {
                    continue;
                }

                std::array<float, 4> barycentric = {};
                if (!tetrahedron_barycentric(render_position, tet_positions, barycentric))
                {
                    continue;
                }

                const float minimum_weight = *std::min_element(barycentric.begin(), barycentric.end());
                if (minimum_weight >= -settings.barycentric_tolerance &&
                    minimum_weight > best_inside_margin)
                {
                    found_inside = true;
                    best_inside_margin = minimum_weight;
                    binding.kind = render_binding_kind::tetrahedron;
                    binding.physics_vertices = tet.vertices;
                    for (std::size_t i = 0; i < 4; ++i)
                    {
                        binding.barycentric[i] = std::max(0.0f, barycentric[i]);
                    }
                    const float sum = binding.barycentric[0] + binding.barycentric[1] +
                        binding.barycentric[2] + binding.barycentric[3];
                    if (sum <= 0.0f)
                    {
                        found_inside = false;
                        continue;
                    }
                    for (float& weight : binding.barycentric)
                    {
                        weight /= sum;
                    }
                }
            }

            if (!found_inside)
            {
                float best_distance_squared = std::numeric_limits<float>::max();
                bool found_boundary = false;
                for (const boundary_triangle& face : physics_value.boundary_triangles)
                {
                    std::array<vec3, 3> triangle = {};
                    bool valid_indices = true;
                    std::array<uint32_t, 3> sorted_vertices = face.vertices;
                    std::sort(sorted_vertices.begin(), sorted_vertices.end());
                    for (std::size_t local_index = 0; local_index < 3; ++local_index)
                    {
                        const uint32_t vertex_id = sorted_vertices[local_index];
                        if (vertex_id >= physics_value.rest_positions.size())
                        {
                            valid_indices = false;
                            break;
                        }
                        triangle[local_index] = physics_value.rest_positions[vertex_id];
                    }
                    if (!valid_indices)
                    {
                        continue;
                    }

                    std::array<float, 3> barycentric = {};
                    vec3 closest = {};
                    if (!closest_point_triangle(render_position, triangle, barycentric, closest))
                    {
                        continue;
                    }
                    const float distance_squared = length_squared(subtract(render_position, closest));
                    if (distance_squared < best_distance_squared)
                    {
                        best_distance_squared = distance_squared;
                        found_boundary = true;
                        binding.kind = render_binding_kind::boundary_triangle;
                        binding.physics_vertices = {
                            sorted_vertices[0], sorted_vertices[1], sorted_vertices[2], asset::invalid_index
                        };
                        binding.barycentric = {
                            barycentric[0], barycentric[1], barycentric[2], 0.0f
                        };

                        const vec3 normal = normalize(cross(
                            subtract(triangle[1], triangle[0]),
                            subtract(triangle[2], triangle[0])));
                        const vec3 edge1 = subtract(triangle[1], triangle[0]);
                        const vec3 edge2 = subtract(triangle[2], triangle[0]);
                        const vec3 offset = subtract(render_position, closest);
                        if (!basis_coordinates(offset, edge1, edge2, normal, binding.rest_offset_local))
                        {
                            found_boundary = false;
                            continue;
                        }
                    }
                }

                if (!found_boundary || best_distance_squared > maximum_distance_squared)
                {
                    add_diagnostic(result.diagnostics, render_binding_error::no_surface_embedding,
                        static_cast<uint32_t>(render_index),
                        "No boundary embedding within configured distance.");
                    result.bindings.clear();
                    return result;
                }
            }

            bool reconstruction_valid = false;
            const vec3 reconstructed = evaluate_binding_position(binding, physics_value, reconstruction_valid);
            if (!reconstruction_valid)
            {
                add_diagnostic(result.diagnostics, render_binding_error::invalid_binding,
                    static_cast<uint32_t>(render_index), "Render binding could not reconstruct its Rest Pose.");
                result.bindings.clear();
                return result;
            }
            const float rest_error = length(subtract(reconstructed, render_position));
            binding.rest_error = rest_error;
            result.maximum_rest_error = std::max(result.maximum_rest_error, rest_error);
            result.bindings.push_back(binding);
        }

        result.maximum_relative_rest_error = diagonal > 1.0e-12f
            ? result.maximum_rest_error / diagonal
            : result.maximum_rest_error;
        if (result.maximum_relative_rest_error >
            settings.maximum_rest_error_relative_to_bbox_diagonal)
        {
            add_diagnostic(result.diagnostics, render_binding_error::excessive_rest_error,
                asset::invalid_index, "Maximum Rest Pose binding error exceeds the configured threshold.");
            result.bindings.clear();
            return result;
        }

        result.success = true;
        return result;
    }

    render_feedback_result apply_render_feedback(
        const asset::mesh& source_mesh,
        const physics_mesh& physics_value,
        const std::vector<render_vertex_binding>& bindings)
    {
        render_feedback_result result;
        if (source_mesh.vertices.empty() || bindings.size() != source_mesh.vertices.size())
        {
            add_diagnostic(result.diagnostics, render_binding_error::invalid_binding,
                asset::invalid_index, "Binding count must match Render Mesh vertex count.");
            return result;
        }
        if (physics_value.rest_positions.empty())
        {
            add_diagnostic(result.diagnostics, render_binding_error::invalid_physics_mesh,
                asset::invalid_index, "Physics Mesh contains no positions.");
            return result;
        }

        result.value = source_mesh;
        for (std::size_t render_index = 0; render_index < bindings.size(); ++render_index)
        {
            const render_vertex_binding& binding = bindings[render_index];
            vec3 position = {};
            if (binding.kind == render_binding_kind::tetrahedron)
            {
                for (std::size_t local_index = 0; local_index < 4; ++local_index)
                {
                    const uint32_t physics_vertex = binding.physics_vertices[local_index];
                    if (physics_vertex >= physics_value.rest_positions.size() ||
                        !std::isfinite(binding.barycentric[local_index]))
                    {
                        add_diagnostic(result.diagnostics, render_binding_error::invalid_binding,
                            static_cast<uint32_t>(render_index), "Tetrahedron binding references invalid data.");
                        return result;
                    }
                    position = add(position, multiply(
                        physics_value.rest_positions[physics_vertex], binding.barycentric[local_index]));
                }
            }
            else
            {
                const uint32_t a = binding.physics_vertices[0];
                const uint32_t b = binding.physics_vertices[1];
                const uint32_t c = binding.physics_vertices[2];
                if (a >= physics_value.rest_positions.size() ||
                    b >= physics_value.rest_positions.size() ||
                    c >= physics_value.rest_positions.size())
                {
                    add_diagnostic(result.diagnostics, render_binding_error::invalid_binding,
                        static_cast<uint32_t>(render_index), "Boundary binding references invalid data.");
                    return result;
                }

                const vec3 p0 = physics_value.rest_positions[a];
                const vec3 p1 = physics_value.rest_positions[b];
                const vec3 p2 = physics_value.rest_positions[c];
                const vec3 e1 = subtract(p1, p0);
                const vec3 e2 = subtract(p2, p0);
                const vec3 normal = normalize(cross(e1, e2));
                if (length_squared(normal) <= minimum_length_squared)
                {
                    add_diagnostic(result.diagnostics, render_binding_error::degenerate_deformed_triangle,
                        static_cast<uint32_t>(render_index), "Deformed boundary triangle is degenerate.");
                    return result;
                }

                const vec3 closest = interpolate_triangle({ p0, p1, p2 }, {
                    binding.barycentric[0], binding.barycentric[1], binding.barycentric[2] });
                position = add(closest, add(add(
                    multiply(e1, binding.rest_offset_local.x),
                    multiply(e2, binding.rest_offset_local.y)),
                    multiply(normal, binding.rest_offset_local.z)));
            }

            if (!finite(position))
            {
                add_diagnostic(result.diagnostics, render_binding_error::invalid_binding,
                    static_cast<uint32_t>(render_index), "Render feedback produced a non-finite position.");
                return result;
            }
            result.value.vertices[render_index].position = position;
        }

        update_normals_and_tangents(result.value);
        result.success = true;
        return result;
    }
}
