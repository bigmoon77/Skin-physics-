#include "skin_physics/physics/cloth_solver.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <unordered_set>

namespace skin_physics::physics
{
    namespace
    {
        using asset::vec3;
        constexpr float minimum_length_squared = 1.0e-20f;
        constexpr float pi = 3.14159265358979323846f;

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

        bool finite(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
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

        float wrap_angle(float angle)
        {
            while (angle > pi) angle -= 2.0f * pi;
            while (angle < -pi) angle += 2.0f * pi;
            return angle;
        }

        void add_diagnostic(
            std::vector<cloth_diagnostic>& diagnostics,
            cloth_error code,
            uint32_t element,
            const std::string& message)
        {
            cloth_diagnostic item;
            item.code = code;
            item.element_index = element;
            item.message = message;
            diagnostics.push_back(std::move(item));
        }

        struct edge_occurrence
        {
            uint32_t first = asset::invalid_index;
            uint32_t second = asset::invalid_index;
            uint32_t opposite = asset::invalid_index;
        };

        struct edge_record
        {
            std::array<uint32_t, 2> key = {};
            std::vector<edge_occurrence> occurrences;
        };

        void build_vertex_constraint_csr(cloth_mesh& mesh)
        {
            const std::size_t vertex_count = mesh.rest_positions.size();
            mesh.vertex_constraint_offsets.assign(
                vertex_count + 1u, 0u);
            mesh.vertex_constraint_refs.clear();

            const auto count_constraint = [&mesh](
                const uint32_t* vertices,
                std::size_t vertex_indices_count)
            {
                for (std::size_t local_index = 0u;
                    local_index < vertex_indices_count; ++local_index)
                {
                    ++mesh.vertex_constraint_offsets[
                        static_cast<std::size_t>(vertices[local_index]) + 1u];
                }
            };

            for (const cloth_stretch_constraint& constraint :
                mesh.stretch_constraints)
            {
                count_constraint(constraint.vertices.data(), 2u);
            }
            for (const cloth_shear_constraint& constraint :
                mesh.shear_constraints)
            {
                count_constraint(constraint.vertices.data(), 3u);
            }
            for (const cloth_bend_constraint& constraint :
                mesh.bend_constraints)
            {
                count_constraint(constraint.vertices.data(), 4u);
            }

            for (std::size_t vertex_index = 1u;
                vertex_index < mesh.vertex_constraint_offsets.size();
                ++vertex_index)
            {
                mesh.vertex_constraint_offsets[vertex_index] +=
                    mesh.vertex_constraint_offsets[vertex_index - 1u];
            }

            mesh.vertex_constraint_refs.resize(
                mesh.vertex_constraint_offsets.back());
            std::vector<uint32_t> cursors =
                mesh.vertex_constraint_offsets;
            const auto append_constraint = [&mesh, &cursors](
                cloth_vertex_constraint_type type,
                uint32_t constraint_index,
                const uint32_t* vertices,
                std::size_t vertex_indices_count)
            {
                for (uint32_t local_index = 0u;
                    local_index < vertex_indices_count; ++local_index)
                {
                    cloth_vertex_constraint_ref reference;
                    reference.type = static_cast<uint32_t>(type);
                    reference.constraint_index = constraint_index;
                    reference.local_vertex_index = local_index;
                    const uint32_t vertex_index = vertices[local_index];
                    mesh.vertex_constraint_refs[
                        cursors[vertex_index]++] = reference;
                }
            };

            for (uint32_t constraint_index = 0u;
                constraint_index < mesh.stretch_constraints.size();
                ++constraint_index)
            {
                const auto& constraint =
                    mesh.stretch_constraints[constraint_index];
                append_constraint(
                    cloth_vertex_constraint_type::stretch,
                    constraint_index,
                    constraint.vertices.data(), 2u);
            }
            for (uint32_t constraint_index = 0u;
                constraint_index < mesh.shear_constraints.size();
                ++constraint_index)
            {
                const auto& constraint =
                    mesh.shear_constraints[constraint_index];
                append_constraint(
                    cloth_vertex_constraint_type::shear,
                    constraint_index,
                    constraint.vertices.data(), 3u);
            }
            for (uint32_t constraint_index = 0u;
                constraint_index < mesh.bend_constraints.size();
                ++constraint_index)
            {
                const auto& constraint =
                    mesh.bend_constraints[constraint_index];
                append_constraint(
                    cloth_vertex_constraint_type::bend,
                    constraint_index,
                    constraint.vertices.data(), 4u);
            }
        }

        void build_stretch_coloring(cloth_mesh& mesh)
        {
            std::vector<std::vector<uint32_t>> color_constraints;
            std::vector<std::vector<uint8_t>> color_vertex_occupancy;
            for (uint32_t constraint_index = 0u;
                constraint_index < mesh.stretch_constraints.size();
                ++constraint_index)
            {
                const cloth_stretch_constraint& constraint =
                    mesh.stretch_constraints[constraint_index];
                std::size_t selected_color = 0u;
                for (; selected_color < color_vertex_occupancy.size();
                    ++selected_color)
                {
                    const std::vector<uint8_t>& occupied =
                        color_vertex_occupancy[selected_color];
                    if (occupied[constraint.vertices[0]] == 0u &&
                        occupied[constraint.vertices[1]] == 0u)
                    {
                        break;
                    }
                }
                if (selected_color == color_vertex_occupancy.size())
                {
                    color_vertex_occupancy.emplace_back(
                        mesh.rest_positions.size(), static_cast<uint8_t>(0u));
                    color_constraints.emplace_back();
                }

                color_vertex_occupancy[selected_color][constraint.vertices[0]] = 1u;
                color_vertex_occupancy[selected_color][constraint.vertices[1]] = 1u;
                color_constraints[selected_color].push_back(constraint_index);
            }

            mesh.stretch_color_ranges.clear();
            mesh.stretch_color_constraint_indices.clear();
            mesh.stretch_color_ranges.reserve(color_constraints.size());
            mesh.stretch_color_constraint_indices.reserve(
                mesh.stretch_constraints.size());
            for (const std::vector<uint32_t>& color : color_constraints)
            {
                cloth_stretch_color_range range;
                range.first_constraint_index =
                    static_cast<uint32_t>(
                        mesh.stretch_color_constraint_indices.size());
                range.constraint_count = static_cast<uint32_t>(color.size());
                mesh.stretch_color_ranges.push_back(range);
                mesh.stretch_color_constraint_indices.insert(
                    mesh.stretch_color_constraint_indices.end(),
                    color.begin(), color.end());
            }
        }

        bool calculate_dihedral(
            const std::array<vec3, 4>& positions,
            float& angle)
        {
            const vec3 edge = subtract(positions[1], positions[0]);
            const vec3 u = subtract(positions[2], positions[0]);
            const vec3 v = subtract(positions[3], positions[0]);
            const vec3 first_cross = cross(edge, u);
            const vec3 second_cross = cross(v, edge);
            const float edge_squared = length_squared(edge);
            const float first_squared = length_squared(first_cross);
            const float second_squared = length_squared(second_cross);
            if (edge_squared <= minimum_length_squared ||
                first_squared <= minimum_length_squared ||
                second_squared <= minimum_length_squared)
            {
                return false;
            }

            const vec3 edge_direction = multiply(edge, 1.0f / std::sqrt(edge_squared));
            const vec3 normal_first = multiply(first_cross, 1.0f / std::sqrt(first_squared));
            const vec3 normal_second = multiply(second_cross, 1.0f / std::sqrt(second_squared));
            const float sine = dot(edge_direction, cross(normal_first, normal_second));
            const float cosine = dot(normal_first, normal_second);
            angle = std::atan2(sine, cosine);
            return std::isfinite(angle);
        }

        vec3 normalized_derivative(const vec3& value, const vec3& derivative)
        {
            const float value_length = length(value);
            if (value_length <= 1.0e-12f)
            {
                return {};
            }
            const vec3 direction = multiply(value, 1.0f / value_length);
            return multiply(subtract(
                derivative,
                multiply(direction, dot(direction, derivative))),
                1.0f / value_length);
        }

        float bend_directional_derivative(
            const std::array<vec3, 4>& positions,
            const std::array<vec3, 4>& displacement)
        {
            const vec3 edge = subtract(positions[1], positions[0]);
            const vec3 u = subtract(positions[2], positions[0]);
            const vec3 v = subtract(positions[3], positions[0]);
            const vec3 first_cross = cross(edge, u);
            const vec3 second_cross = cross(v, edge);
            const vec3 edge_direction = normalize(edge);
            const vec3 normal_first = normalize(first_cross);
            const vec3 normal_second = normalize(second_cross);
            const float sine = dot(edge_direction, cross(normal_first, normal_second));
            const float cosine = dot(normal_first, normal_second);

            const vec3 delta_edge = subtract(displacement[1], displacement[0]);
            const vec3 delta_u = subtract(displacement[2], displacement[0]);
            const vec3 delta_v = subtract(displacement[3], displacement[0]);

            const vec3 delta_first_cross =
                add(cross(delta_edge, u), cross(edge, delta_u));
            const vec3 delta_second_cross =
                add(cross(delta_v, edge), cross(v, delta_edge));
            const vec3 delta_normal_first =
                normalized_derivative(first_cross, delta_first_cross);
            const vec3 delta_normal_second =
                normalized_derivative(second_cross, delta_second_cross);
            const vec3 delta_edge_direction =
                normalized_derivative(edge, delta_edge);

            const float delta_sine =
                dot(delta_edge_direction, cross(normal_first, normal_second)) +
                dot(edge_direction, add(
                    cross(delta_normal_first, normal_second),
                    cross(normal_first, delta_normal_second)));
            const float delta_cosine =
                dot(delta_normal_first, normal_second) +
                dot(normal_first, delta_normal_second);

            const float denominator = sine * sine + cosine * cosine;
            if (!std::isfinite(denominator) || denominator <= 1.0e-20f)
            {
                return 0.0f;
            }
            return (cosine * delta_sine - sine * delta_cosine) / denominator;
        }

        float evaluate_shear(
            const std::array<vec3, 3>& positions,
            const cloth_shear_constraint& constraint,
            std::array<vec3, 3>& gradients)
        {
            const vec3 u = subtract(positions[1], positions[0]);
            const vec3 v = subtract(positions[2], positions[0]);
            const float normalization = constraint.normalization_length;
            if (!std::isfinite(normalization) || normalization <= 1.0e-12f)
            {
                return 0.0f;
            }
            gradients[1] = multiply(v, 1.0f / normalization);
            gradients[2] = multiply(u, 1.0f / normalization);
            gradients[0] = multiply(add(u, v), -1.0f / normalization);
            return (dot(u, v) - constraint.rest_dot) / normalization;
        }

        bool valid_settings(const cloth_solver_settings& settings)
        {
            return std::isfinite(settings.stretch_compliance) &&
                settings.stretch_compliance >= 0.0f &&
                std::isfinite(settings.shear_compliance) &&
                settings.shear_compliance >= 0.0f &&
                std::isfinite(settings.bend_compliance) &&
                settings.bend_compliance >= 0.0f &&
                std::isfinite(settings.fixed_delta_time) &&
                settings.fixed_delta_time > 0.0f &&
                std::isfinite(settings.relaxation) &&
                settings.relaxation > 0.0f && settings.relaxation <= 1.0f &&
                settings.iterations > 0;
        }
    }

    cloth_build_result build_cloth_mesh(const raw_cloth_mesh& input)
    {
        cloth_build_result result;
        if (input.rest_positions.empty() || input.triangles.empty())
        {
            add_diagnostic(result.diagnostics, cloth_error::empty_mesh,
                asset::invalid_index, "Cloth mesh requires positions and triangles.");
            return result;
        }

        for (std::size_t index = 0; index < input.rest_positions.size(); ++index)
        {
            if (!finite(input.rest_positions[index]))
            {
                add_diagnostic(result.diagnostics, cloth_error::non_finite_state,
                    static_cast<uint32_t>(index), "Cloth Rest Pose contains a non-finite position.");
                return result;
            }
        }

        std::map<std::array<uint32_t, 2>, edge_record> edges;
        result.value.rest_positions = input.rest_positions;
        result.value.triangles = input.triangles;

        for (std::size_t triangle_index = 0; triangle_index < input.triangles.size(); ++triangle_index)
        {
            const std::array<uint32_t, 3>& triangle = input.triangles[triangle_index];
            if (triangle[0] >= input.rest_positions.size() ||
                triangle[1] >= input.rest_positions.size() ||
                triangle[2] >= input.rest_positions.size() ||
                triangle[0] == triangle[1] || triangle[1] == triangle[2] ||
                triangle[2] == triangle[0])
            {
                add_diagnostic(result.diagnostics, cloth_error::invalid_index,
                    static_cast<uint32_t>(triangle_index), "Cloth triangle contains an invalid or repeated vertex index.");
                return result;
            }

            const vec3& p0 = input.rest_positions[triangle[0]];
            const vec3& p1 = input.rest_positions[triangle[1]];
            const vec3& p2 = input.rest_positions[triangle[2]];
            const vec3 edge1 = subtract(p1, p0);
            const vec3 edge2 = subtract(p2, p0);
            const float area_twice = length(cross(edge1, edge2));
            if (!std::isfinite(area_twice) || area_twice <= 1.0e-10f)
            {
                add_diagnostic(result.diagnostics, cloth_error::degenerate_triangle,
                    static_cast<uint32_t>(triangle_index), "Cloth triangle area is degenerate.");
                return result;
            }

            cloth_shear_constraint shear;
            shear.vertices = triangle;
            shear.rest_dot = dot(edge1, edge2);
            shear.normalization_length = length(edge1) * length(edge2);
            result.value.shear_constraints.push_back(shear);

            for (std::size_t edge_index = 0; edge_index < 3; ++edge_index)
            {
                const uint32_t first = triangle[edge_index];
                const uint32_t second = triangle[(edge_index + 1) % 3];
                const uint32_t opposite = triangle[(edge_index + 2) % 3];
                const std::array<uint32_t, 2> key = {
                    std::min(first, second), std::max(first, second)
                };
                edge_record& record = edges[key];
                record.key = key;
                record.occurrences.push_back({ first, second, opposite });
                if (record.occurrences.size() > 2)
                {
                    add_diagnostic(result.diagnostics, cloth_error::non_manifold_edge,
                        static_cast<uint32_t>(triangle_index), "A cloth edge is shared by more than two triangles.");
                    return result;
                }
            }
        }

        for (const auto& entry : edges)
        {
            const edge_record& edge = entry.second;
            const vec3 delta = subtract(
                input.rest_positions[edge.key[1]],
                input.rest_positions[edge.key[0]]);
            const float rest_length = length(delta);
            if (!std::isfinite(rest_length) || rest_length <= 1.0e-10f)
            {
                add_diagnostic(result.diagnostics, cloth_error::degenerate_constraint,
                    edge.key[0], "Cloth stretch edge has zero rest length.");
                return result;
            }
            result.value.stretch_constraints.push_back({ edge.key, rest_length });

            if (edge.occurrences.size() == 2)
            {
                const edge_occurrence& first = edge.occurrences[0];
                const edge_occurrence& second = edge.occurrences[1];
                cloth_bend_constraint bend;
                bend.vertices = {
                    first.first, first.second, first.opposite, second.opposite
                };
                std::array<vec3, 4> points = {
                    input.rest_positions[bend.vertices[0]],
                    input.rest_positions[bend.vertices[1]],
                    input.rest_positions[bend.vertices[2]],
                    input.rest_positions[bend.vertices[3]]
                };
                if (!calculate_dihedral(points, bend.rest_angle))
                {
                    add_diagnostic(result.diagnostics, cloth_error::degenerate_constraint,
                        edge.key[0], "Cloth bend hinge has a degenerate edge or adjacent triangle.");
                    return result;
                }
                result.value.bend_constraints.push_back(bend);
            }
        }

        build_stretch_coloring(result.value);
        build_vertex_constraint_csr(result.value);
        std::string coloring_diagnostic;
        if (!validate_cloth_stretch_coloring(
                result.value, coloring_diagnostic))
        {
            add_diagnostic(
                result.diagnostics,
                cloth_error::invalid_settings,
                asset::invalid_index,
                coloring_diagnostic);
            return result;
        }
        if (!validate_cloth_vertex_constraint_csr(
                result.value, coloring_diagnostic))
        {
            add_diagnostic(
                result.diagnostics,
                cloth_error::invalid_settings,
                asset::invalid_index,
                coloring_diagnostic);
            return result;
        }

        result.success = true;
        return result;
    }

    bool validate_cloth_stretch_coloring(
        const cloth_mesh& mesh,
        std::string& diagnostic)
    {
        diagnostic.clear();
        if (mesh.stretch_constraints.empty())
        {
            if (!mesh.stretch_color_ranges.empty() ||
                !mesh.stretch_color_constraint_indices.empty())
            {
                diagnostic =
                    "An empty Stretch constraint set must not contain color ranges or indices.";
                return false;
            }
            return true;
        }
        if (mesh.stretch_color_ranges.empty() ||
            mesh.stretch_color_constraint_indices.size() !=
                mesh.stretch_constraints.size())
        {
            diagnostic =
                "Stretch coloring must cover every constraint exactly once.";
            return false;
        }

        std::vector<uint8_t> visited(mesh.stretch_constraints.size(), static_cast<uint8_t>(0u));
        std::size_t expected_first = 0u;
        for (std::size_t color_index = 0u;
            color_index < mesh.stretch_color_ranges.size();
            ++color_index)
        {
            const cloth_stretch_color_range& range =
                mesh.stretch_color_ranges[color_index];
            if (range.first_constraint_index != expected_first ||
                range.constraint_count == 0u ||
                static_cast<std::size_t>(range.first_constraint_index) >
                    mesh.stretch_color_constraint_indices.size() ||
                static_cast<std::size_t>(range.constraint_count) >
                    mesh.stretch_color_constraint_indices.size() -
                        static_cast<std::size_t>(range.first_constraint_index))
            {
                diagnostic =
                    "Stretch color ranges must be non-empty, contiguous and inside the flattened index table.";
                return false;
            }

            std::vector<uint8_t> occupied(mesh.rest_positions.size(), static_cast<uint8_t>(0u));
            for (uint32_t local_index = 0u;
                local_index < range.constraint_count;
                ++local_index)
            {
                const uint32_t table_index =
                    range.first_constraint_index + local_index;
                const uint32_t constraint_index =
                    mesh.stretch_color_constraint_indices[table_index];
                if (constraint_index >= mesh.stretch_constraints.size() ||
                    visited[constraint_index] != 0u)
                {
                    diagnostic =
                        "Stretch coloring contains an out-of-range or duplicate constraint index.";
                    return false;
                }
                visited[constraint_index] = 1u;

                const cloth_stretch_constraint& constraint =
                    mesh.stretch_constraints[constraint_index];
                const uint32_t first = constraint.vertices[0];
                const uint32_t second = constraint.vertices[1];
                if (first >= mesh.rest_positions.size() ||
                    second >= mesh.rest_positions.size() ||
                    first == second ||
                    occupied[first] != 0u ||
                    occupied[second] != 0u)
                {
                    diagnostic =
                        "A Stretch color contains invalid constraints or two writes to the same vertex.";
                    return false;
                }
                occupied[first] = 1u;
                occupied[second] = 1u;
            }
            expected_first += range.constraint_count;
        }

        if (expected_first != mesh.stretch_color_constraint_indices.size() ||
            std::find(visited.begin(), visited.end(), 0u) != visited.end())
        {
            diagnostic =
                "Stretch coloring does not cover the complete constraint set.";
            return false;
        }
        return true;
    }

    bool validate_cloth_vertex_constraint_csr(
        const cloth_mesh& mesh,
        std::string& diagnostic)
    {
        diagnostic.clear();
        if (mesh.vertex_constraint_offsets.size() !=
                mesh.rest_positions.size() + 1u ||
            mesh.vertex_constraint_offsets.empty() ||
            mesh.vertex_constraint_offsets.front() != 0u ||
            mesh.vertex_constraint_offsets.back() !=
                mesh.vertex_constraint_refs.size())
        {
            diagnostic =
                "Vertex-to-Constraint CSR offsets must contain V+1 entries and span the complete reference table.";
            return false;
        }

        std::vector<uint8_t> stretch_seen(
            mesh.stretch_constraints.size() * 2u, static_cast<uint8_t>(0u));
        std::vector<uint8_t> shear_seen(
            mesh.shear_constraints.size() * 3u, static_cast<uint8_t>(0u));
        std::vector<uint8_t> bend_seen(
            mesh.bend_constraints.size() * 4u, static_cast<uint8_t>(0u));
        std::size_t expected_reference_count = 0u;
        for (std::size_t index = 1u;
            index < mesh.vertex_constraint_offsets.size(); ++index)
        {
            if (mesh.vertex_constraint_offsets[index] <
                mesh.vertex_constraint_offsets[index - 1u])
            {
                diagnostic =
                    "Vertex-to-Constraint CSR offsets must be monotonically increasing.";
                return false;
            }
        }

        for (std::size_t vertex_index = 0u;
            vertex_index < mesh.rest_positions.size(); ++vertex_index)
        {
            const uint32_t first =
                mesh.vertex_constraint_offsets[vertex_index];
            const uint32_t end =
                mesh.vertex_constraint_offsets[vertex_index + 1u];
            expected_reference_count += end - first;
            for (uint32_t reference_index = first;
                reference_index < end; ++reference_index)
            {
                const cloth_vertex_constraint_ref& reference =
                    mesh.vertex_constraint_refs[reference_index];
                if (reference.reserved != 0u)
                {
                    diagnostic =
                        "Vertex-to-Constraint CSR reserved fields must be zero.";
                    return false;
                }

                std::vector<uint8_t>* seen = nullptr;
                std::size_t incidence_index = 0u;
                if (reference.type == static_cast<uint32_t>(
                        cloth_vertex_constraint_type::stretch))
                {
                    if (reference.constraint_index >=
                            mesh.stretch_constraints.size() ||
                        reference.local_vertex_index >= 2u)
                    {
                        diagnostic =
                            "Vertex-to-Constraint CSR Stretch reference is out of range.";
                        return false;
                    }
                    const auto& constraint =
                        mesh.stretch_constraints[reference.constraint_index];
                    if (constraint.vertices[reference.local_vertex_index] !=
                        vertex_index)
                    {
                        diagnostic =
                            "Vertex-to-Constraint CSR Stretch reference targets the wrong vertex.";
                        return false;
                    }
                    seen = &stretch_seen;
                    incidence_index =
                        static_cast<std::size_t>(reference.constraint_index) * 2u +
                        reference.local_vertex_index;
                }
                else if (reference.type == static_cast<uint32_t>(
                        cloth_vertex_constraint_type::shear))
                {
                    if (reference.constraint_index >=
                            mesh.shear_constraints.size() ||
                        reference.local_vertex_index >= 3u)
                    {
                        diagnostic =
                            "Vertex-to-Constraint CSR Shear reference is out of range.";
                        return false;
                    }
                    const auto& constraint =
                        mesh.shear_constraints[reference.constraint_index];
                    if (constraint.vertices[reference.local_vertex_index] !=
                        vertex_index)
                    {
                        diagnostic =
                            "Vertex-to-Constraint CSR Shear reference targets the wrong vertex.";
                        return false;
                    }
                    seen = &shear_seen;
                    incidence_index =
                        static_cast<std::size_t>(reference.constraint_index) * 3u +
                        reference.local_vertex_index;
                }
                else if (reference.type == static_cast<uint32_t>(
                        cloth_vertex_constraint_type::bend))
                {
                    if (reference.constraint_index >=
                            mesh.bend_constraints.size() ||
                        reference.local_vertex_index >= 4u)
                    {
                        diagnostic =
                            "Vertex-to-Constraint CSR Bend reference is out of range.";
                        return false;
                    }
                    const auto& constraint =
                        mesh.bend_constraints[reference.constraint_index];
                    if (constraint.vertices[reference.local_vertex_index] !=
                        vertex_index)
                    {
                        diagnostic =
                            "Vertex-to-Constraint CSR Bend reference targets the wrong vertex.";
                        return false;
                    }
                    seen = &bend_seen;
                    incidence_index =
                        static_cast<std::size_t>(reference.constraint_index) * 4u +
                        reference.local_vertex_index;
                }
                else
                {
                    diagnostic =
                        "Vertex-to-Constraint CSR contains an unknown constraint type.";
                    return false;
                }

                if ((*seen)[incidence_index] != 0u)
                {
                    diagnostic =
                        "Vertex-to-Constraint CSR contains a duplicate constraint incidence.";
                    return false;
                }
                (*seen)[incidence_index] = 1u;
            }
        }

        if (expected_reference_count != mesh.vertex_constraint_refs.size() ||
            std::find(stretch_seen.begin(), stretch_seen.end(), 0u) !=
                stretch_seen.end() ||
            std::find(shear_seen.begin(), shear_seen.end(), 0u) !=
                shear_seen.end() ||
            std::find(bend_seen.begin(), bend_seen.end(), 0u) !=
                bend_seen.end())
        {
            diagnostic =
                "Vertex-to-Constraint CSR must contain every topology incidence exactly once.";
            return false;
        }
        return true;
    }

    cloth_gpu_topology_build_result build_cloth_gpu_topology_packet(
        const cloth_mesh& mesh,
        const cloth_solver_settings& settings)
    {
        cloth_gpu_topology_build_result result;
        if (mesh.rest_positions.empty() ||
            mesh.rest_positions.size() >=
                static_cast<std::size_t>(asset::invalid_index) ||
            mesh.vertex_constraint_offsets.size() !=
                mesh.rest_positions.size() + 1u)
        {
            result.error = cloth_gpu_topology_error::invalid_mesh;
            result.message =
                "Cloth GPU topology requires a non-empty Vertex array and a V+1 CSR offset table.";
            return result;
        }

        if (!std::isfinite(settings.stretch_compliance) ||
            settings.stretch_compliance < 0.0f ||
            !std::isfinite(settings.shear_compliance) ||
            settings.shear_compliance < 0.0f ||
            !std::isfinite(settings.bend_compliance) ||
            settings.bend_compliance < 0.0f ||
            !std::isfinite(settings.fixed_delta_time) ||
            settings.fixed_delta_time <= 0.0f ||
            !std::isfinite(settings.relaxation) ||
            settings.relaxation <= 0.0f ||
            settings.relaxation > 1.0f ||
            settings.iterations == 0u)
        {
            result.error = cloth_gpu_topology_error::invalid_settings;
            result.message =
                "Cloth GPU topology requires finite non-negative Compliance, positive fixed dt / iterations and relaxation in (0, 1].";
            return result;
        }

        std::string validation_diagnostic;
        if (!validate_cloth_stretch_coloring(mesh, validation_diagnostic))
        {
            result.error = cloth_gpu_topology_error::invalid_coloring;
            result.message =
                "Cloth GPU Stretch topology validation failed: " +
                validation_diagnostic;
            return result;
        }
        if (!validate_cloth_vertex_constraint_csr(
                mesh, validation_diagnostic))
        {
            result.error = cloth_gpu_topology_error::invalid_csr;
            result.message =
                "Cloth GPU Vertex-to-Constraint CSR validation failed: " +
                validation_diagnostic;
            return result;
        }

        const std::size_t maximum_gpu_count =
            static_cast<std::size_t>(asset::invalid_index);
        if (mesh.stretch_constraints.size() >= maximum_gpu_count ||
            mesh.stretch_color_constraint_indices.size() >= maximum_gpu_count ||
            mesh.shear_constraints.size() >= maximum_gpu_count ||
            mesh.bend_constraints.size() >= maximum_gpu_count ||
            mesh.vertex_constraint_refs.size() >= maximum_gpu_count)
        {
            result.error = cloth_gpu_topology_error::capacity_exceeded;
            result.message =
                "Cloth GPU topology count exceeds the 32-bit StructuredBuffer index range.";
            return result;
        }

        for (std::size_t vertex_index = 0u;
            vertex_index < mesh.rest_positions.size(); ++vertex_index)
        {
            if (!finite(mesh.rest_positions[vertex_index]))
            {
                result.error = cloth_gpu_topology_error::invalid_mesh;
                result.message =
                    "Cloth GPU topology contains a non-finite Rest Position at Vertex " +
                    std::to_string(vertex_index) + ".";
                return result;
            }
        }

        for (std::size_t constraint_index = 0u;
            constraint_index < mesh.stretch_constraints.size();
            ++constraint_index)
        {
            const cloth_stretch_constraint& constraint =
                mesh.stretch_constraints[constraint_index];
            if (!std::isfinite(constraint.rest_length) ||
                constraint.rest_length <= 1.0e-12f)
            {
                result.error = cloth_gpu_topology_error::invalid_mesh;
                result.message =
                    "Cloth GPU Stretch constraint has an invalid Rest Length at index " +
                    std::to_string(constraint_index) + ".";
                return result;
            }
        }

        for (std::size_t constraint_index = 0u;
            constraint_index < mesh.shear_constraints.size();
            ++constraint_index)
        {
            const cloth_shear_constraint& constraint =
                mesh.shear_constraints[constraint_index];
            if (!std::isfinite(constraint.rest_dot) ||
                !std::isfinite(constraint.normalization_length) ||
                constraint.normalization_length <= 1.0e-10f ||
                constraint.vertices[0] >= mesh.rest_positions.size() ||
                constraint.vertices[1] >= mesh.rest_positions.size() ||
                constraint.vertices[2] >= mesh.rest_positions.size() ||
                constraint.vertices[0] == constraint.vertices[1] ||
                constraint.vertices[1] == constraint.vertices[2] ||
                constraint.vertices[2] == constraint.vertices[0])
            {
                result.error = cloth_gpu_topology_error::invalid_mesh;
                result.message =
                    "Cloth GPU Shear constraint contains invalid Rest data or Vertex indices at index " +
                    std::to_string(constraint_index) + ".";
                return result;
            }
        }

        for (std::size_t constraint_index = 0u;
            constraint_index < mesh.bend_constraints.size();
            ++constraint_index)
        {
            const cloth_bend_constraint& constraint =
                mesh.bend_constraints[constraint_index];
            if (!std::isfinite(constraint.rest_angle))
            {
                result.error = cloth_gpu_topology_error::invalid_mesh;
                result.message =
                    "Cloth GPU Bend constraint contains a non-finite Rest Angle at index " +
                    std::to_string(constraint_index) + ".";
                return result;
            }
            for (std::size_t local_index = 0u; local_index < 4u; ++local_index)
            {
                if (constraint.vertices[local_index] >= mesh.rest_positions.size())
                {
                    result.error = cloth_gpu_topology_error::invalid_mesh;
                    result.message =
                        "Cloth GPU Bend constraint contains an out-of-range Vertex index at index " +
                        std::to_string(constraint_index) + ".";
                    return result;
                }
                for (std::size_t previous_index = 0u;
                    previous_index < local_index; ++previous_index)
                {
                    if (constraint.vertices[local_index] ==
                        constraint.vertices[previous_index])
                    {
                        result.error = cloth_gpu_topology_error::invalid_mesh;
                        result.message =
                            "Cloth GPU Bend constraint contains a repeated Vertex index at index " +
                            std::to_string(constraint_index) + ".";
                        return result;
                    }
                }
            }
        }

        cloth_gpu_topology_packet candidate;
        candidate.vertex_count = static_cast<uint32_t>(
            mesh.rest_positions.size());
        candidate.stretch_constraints.reserve(
            mesh.stretch_constraints.size());
        candidate.stretch_color_ranges = mesh.stretch_color_ranges;
        candidate.stretch_color_constraint_indices =
            mesh.stretch_color_constraint_indices;
        candidate.shear_constraints.reserve(mesh.shear_constraints.size());
        candidate.bend_constraints.reserve(mesh.bend_constraints.size());
        candidate.vertex_constraint_offsets =
            mesh.vertex_constraint_offsets;
        candidate.vertex_constraint_refs = mesh.vertex_constraint_refs;
        candidate.stretch_lambdas.assign(
            mesh.stretch_constraints.size(), 0.0f);
        candidate.shear_lambdas.assign(
            mesh.shear_constraints.size(), 0.0f);
        candidate.bend_lambdas.assign(
            mesh.bend_constraints.size(), 0.0f);

        for (const cloth_stretch_constraint& constraint :
            mesh.stretch_constraints)
        {
            cloth_gpu_stretch_constraint_record record;
            record.vertices = constraint.vertices;
            record.rest_length = constraint.rest_length;
            record.reserved = 0.0f;
            candidate.stretch_constraints.push_back(record);
        }

        for (const cloth_shear_constraint& constraint :
            mesh.shear_constraints)
        {
            cloth_gpu_constraint_record record;
            record.indices = {
                constraint.vertices[0],
                constraint.vertices[1],
                constraint.vertices[2],
                0u
            };
            record.type = static_cast<uint32_t>(
                cloth_vertex_constraint_type::shear);
            record.rest_value = constraint.rest_dot;
            record.normalization = constraint.normalization_length;
            record.compliance = settings.shear_compliance;
            candidate.shear_constraints.push_back(record);
        }

        for (const cloth_bend_constraint& constraint :
            mesh.bend_constraints)
        {
            cloth_gpu_constraint_record record;
            record.indices = {
                constraint.vertices[0],
                constraint.vertices[1],
                constraint.vertices[2],
                constraint.vertices[3]
            };
            record.type = static_cast<uint32_t>(
                cloth_vertex_constraint_type::bend);
            record.rest_value = constraint.rest_angle;
            record.normalization = 0.0f;
            record.compliance = settings.bend_compliance;
            candidate.bend_constraints.push_back(record);
        }

        result.value = std::move(candidate);
        result.success = true;
        return result;
    }

    cloth_stretch_color_step_result solve_cloth_stretch_color(
        const cloth_mesh& mesh,
        uint32_t color_index,
        const std::vector<vec3>& current_positions,
        const std::vector<float>& inverse_mass,
        const std::vector<float>& lambda_values,
        float delta_time,
        float compliance,
        float relaxation)
    {
        cloth_stretch_color_step_result result;
        std::string coloring_diagnostic;
        if (!validate_cloth_stretch_coloring(mesh, coloring_diagnostic))
        {
            add_diagnostic(
                result.diagnostics, cloth_error::invalid_settings,
                color_index, coloring_diagnostic);
            return result;
        }
        if (color_index >= mesh.stretch_color_ranges.size() ||
            current_positions.size() != mesh.rest_positions.size() ||
            inverse_mass.size() != mesh.rest_positions.size() ||
            lambda_values.size() != mesh.stretch_constraints.size() ||
            !std::isfinite(delta_time) || delta_time <= 0.0f ||
            !std::isfinite(compliance) || compliance < 0.0f ||
            !std::isfinite(relaxation) ||
            relaxation <= 0.0f || relaxation > 1.0f)
        {
            add_diagnostic(
                result.diagnostics, cloth_error::invalid_settings,
                color_index,
                "Stretch color evaluation requires a valid color, matching state arrays and finite solver coefficients.");
            return result;
        }
        for (std::size_t vertex_index = 0u;
            vertex_index < current_positions.size(); ++vertex_index)
        {
            if (!finite(current_positions[vertex_index]) ||
                !std::isfinite(inverse_mass[vertex_index]) ||
                inverse_mass[vertex_index] < 0.0f)
            {
                add_diagnostic(
                    result.diagnostics, cloth_error::non_finite_state,
                    static_cast<uint32_t>(vertex_index),
                    "Stretch color input contains a non-finite Position or invalid inverse mass.");
                return result;
            }
        }
        for (std::size_t constraint_index = 0u;
            constraint_index < lambda_values.size(); ++constraint_index)
        {
            if (!std::isfinite(lambda_values[constraint_index]))
            {
                add_diagnostic(
                    result.diagnostics, cloth_error::non_finite_state,
                    static_cast<uint32_t>(constraint_index),
                    "Stretch color Lambda input must be finite.");
                return result;
            }
        }

        result.positions = current_positions;
        result.lambda_values = lambda_values;
        const cloth_stretch_color_range& range =
            mesh.stretch_color_ranges[color_index];
        const float alpha = compliance / (delta_time * delta_time);
        if (!std::isfinite(alpha))
        {
            add_diagnostic(
                result.diagnostics, cloth_error::invalid_settings,
                color_index,
                "Stretch color Compliance / dt^2 exceeded finite range.");
            result.positions.clear();
            result.lambda_values.clear();
            return result;
        }

        for (uint32_t local_index = 0u;
            local_index < range.constraint_count;
            ++local_index)
        {
            const uint32_t constraint_index =
                mesh.stretch_color_constraint_indices[
                    range.first_constraint_index + local_index];
            const cloth_stretch_constraint& constraint =
                mesh.stretch_constraints[constraint_index];
            const uint32_t first = constraint.vertices[0];
            const uint32_t second = constraint.vertices[1];
            const vec3 delta = subtract(
                result.positions[first], result.positions[second]);
            const float current_length = length(delta);
            if (!std::isfinite(current_length) ||
                current_length <= 1.0e-12f ||
                !std::isfinite(constraint.rest_length) ||
                constraint.rest_length <= 0.0f)
            {
                add_diagnostic(
                    result.diagnostics, cloth_error::degenerate_constraint,
                    constraint_index,
                    "Stretch color encountered a degenerate edge or Rest Length.");
                result.positions.clear();
                result.lambda_values.clear();
                return result;
            }

            const float first_inverse_mass = inverse_mass[first];
            const float second_inverse_mass = inverse_mass[second];
            const float denominator =
                first_inverse_mass + second_inverse_mass + alpha;
            if (!std::isfinite(denominator))
            {
                add_diagnostic(
                    result.diagnostics, cloth_error::non_finite_state,
                    constraint_index,
                    "Stretch color denominator became non-finite.");
                result.positions.clear();
                result.lambda_values.clear();
                return result;
            }
            if (denominator <= 1.0e-20f)
            {
                continue;
            }

            const vec3 direction = multiply(delta, 1.0f / current_length);
            const float constraint_value =
                current_length - constraint.rest_length;
            const float delta_lambda =
                (-constraint_value -
                    alpha * result.lambda_values[constraint_index]) /
                denominator;
            const float applied = relaxation * delta_lambda;
            if (!std::isfinite(applied))
            {
                add_diagnostic(
                    result.diagnostics, cloth_error::non_finite_state,
                    constraint_index,
                    "Stretch color Lambda update became non-finite.");
                result.positions.clear();
                result.lambda_values.clear();
                return result;
            }

            result.lambda_values[constraint_index] += applied;
            result.positions[first] = add(
                result.positions[first],
                multiply(direction, first_inverse_mass * applied));
            result.positions[second] = add(
                result.positions[second],
                multiply(direction, -second_inverse_mass * applied));
            if (!finite(result.positions[first]) ||
                !finite(result.positions[second]) ||
                !std::isfinite(result.lambda_values[constraint_index]))
            {
                add_diagnostic(
                    result.diagnostics, cloth_error::non_finite_state,
                    constraint_index,
                    "Stretch color produced a non-finite Position or Lambda.");
                result.positions.clear();
                result.lambda_values.clear();
                return result;
            }
        }

        result.success = true;
        return result;
    }

    cloth_bend_evaluation evaluate_cloth_bend(
        const std::array<vec3, 4>& positions,
        float rest_angle)
    {
        cloth_bend_evaluation result;
        float angle = 0.0f;
        if (!std::isfinite(rest_angle) || !calculate_dihedral(positions, angle))
        {
            return result;
        }

        result.constraint = wrap_angle(angle - rest_angle);
        for (std::size_t vertex_index = 0; vertex_index < 4; ++vertex_index)
        {
            for (uint32_t axis = 0; axis < 3; ++axis)
            {
                std::array<vec3, 4> displacement = {};
                if (axis == 0) displacement[vertex_index].x = 1.0f;
                if (axis == 1) displacement[vertex_index].y = 1.0f;
                if (axis == 2) displacement[vertex_index].z = 1.0f;
                const float derivative = bend_directional_derivative(positions, displacement);
                if (axis == 0) result.gradients[vertex_index].x = derivative;
                if (axis == 1) result.gradients[vertex_index].y = derivative;
                if (axis == 2) result.gradients[vertex_index].z = derivative;
            }
        }

        result.valid = std::isfinite(result.constraint);
        for (const vec3& gradient : result.gradients)
        {
            result.valid = result.valid && finite(gradient);
        }
        return result;
    }

    cloth_step_result solve_cloth(
        const cloth_mesh& mesh,
        const std::vector<vec3>& current_positions,
        const std::vector<float>& inverse_mass,
        const cloth_solver_settings& settings)
    {
        cloth_step_result result;
        if (!valid_settings(settings))
        {
            add_diagnostic(result.diagnostics, cloth_error::invalid_settings,
                asset::invalid_index, "Cloth solver settings are invalid.");
            return result;
        }
        if (current_positions.size() != mesh.rest_positions.size() ||
            inverse_mass.size() != mesh.rest_positions.size() ||
            current_positions.empty())
        {
            add_diagnostic(result.diagnostics, cloth_error::mismatched_state,
                asset::invalid_index, "Cloth positions and inverse masses must match the mesh vertex count.");
            return result;
        }
        for (std::size_t index = 0; index < current_positions.size(); ++index)
        {
            if (!finite(current_positions[index]) ||
                !std::isfinite(inverse_mass[index]) || inverse_mass[index] < 0.0f)
            {
                add_diagnostic(result.diagnostics, cloth_error::non_finite_state,
                    static_cast<uint32_t>(index), "Cloth state must contain finite positions and non-negative inverse masses.");
                return result;
            }
        }

        result.positions = current_positions;
        std::vector<float> stretch_lambda(mesh.stretch_constraints.size(), 0.0f);
        std::vector<float> shear_lambda(mesh.shear_constraints.size(), 0.0f);
        std::vector<float> bend_lambda(mesh.bend_constraints.size(), 0.0f);
        const float inverse_dt_squared = 1.0f /
            (settings.fixed_delta_time * settings.fixed_delta_time);

        for (uint32_t iteration = 0; iteration < settings.iterations; ++iteration)
        {
            std::vector<vec3> corrections(result.positions.size());

            if (settings.solve_stretch)
            {
                for (std::size_t constraint_index = 0;
                    constraint_index < mesh.stretch_constraints.size(); ++constraint_index)
                {
                    const cloth_stretch_constraint& constraint =
                        mesh.stretch_constraints[constraint_index];
                    const uint32_t i0 = constraint.vertices[0];
                    const uint32_t i1 = constraint.vertices[1];
                    const vec3 delta = subtract(result.positions[i0], result.positions[i1]);
                    const float current_length = length(delta);
                    if (!std::isfinite(current_length) || current_length <= 1.0e-12f)
                    {
                        add_diagnostic(result.diagnostics, cloth_error::degenerate_constraint,
                            static_cast<uint32_t>(constraint_index), "Cloth stretch constraint became degenerate.");
                        return result;
                    }
                    const vec3 direction = multiply(delta, 1.0f / current_length);
                    const float constraint_value = current_length - constraint.rest_length;
                    const float alpha = settings.stretch_compliance * inverse_dt_squared;
                    const float denominator = inverse_mass[i0] + inverse_mass[i1] + alpha;
                    if (denominator <= 1.0e-20f)
                    {
                        continue;
                    }
                    const float delta_lambda =
                        (-constraint_value - alpha * stretch_lambda[constraint_index]) / denominator;
                    const float applied = settings.relaxation * delta_lambda;
                    stretch_lambda[constraint_index] += applied;
                    corrections[i0] = add(corrections[i0], multiply(direction, inverse_mass[i0] * applied));
                    corrections[i1] = add(corrections[i1], multiply(direction, -inverse_mass[i1] * applied));
                }
            }

            if (settings.solve_shear)
            {
                for (std::size_t constraint_index = 0;
                    constraint_index < mesh.shear_constraints.size(); ++constraint_index)
                {
                    const cloth_shear_constraint& constraint =
                        mesh.shear_constraints[constraint_index];
                    const std::array<uint32_t, 3>& ids = constraint.vertices;
                    const std::array<vec3, 3> points = {
                        result.positions[ids[0]], result.positions[ids[1]], result.positions[ids[2]]
                    };
                    std::array<vec3, 3> gradients = {};
                    const float constraint_value = evaluate_shear(points, constraint, gradients);
                    const float alpha = settings.shear_compliance * inverse_dt_squared;
                    float denominator = alpha;
                    for (std::size_t local_index = 0; local_index < 3; ++local_index)
                    {
                        denominator += inverse_mass[ids[local_index]] *
                            length_squared(gradients[local_index]);
                    }
                    if (!std::isfinite(constraint_value) || denominator <= 1.0e-20f)
                    {
                        continue;
                    }
                    const float delta_lambda =
                        (-constraint_value - alpha * shear_lambda[constraint_index]) / denominator;
                    const float applied = settings.relaxation * delta_lambda;
                    shear_lambda[constraint_index] += applied;
                    for (std::size_t local_index = 0; local_index < 3; ++local_index)
                    {
                        const uint32_t vertex_id = ids[local_index];
                        corrections[vertex_id] = add(corrections[vertex_id],
                            multiply(gradients[local_index], inverse_mass[vertex_id] * applied));
                    }
                }
            }

            if (settings.solve_bend)
            {
                for (std::size_t constraint_index = 0;
                    constraint_index < mesh.bend_constraints.size(); ++constraint_index)
                {
                    const cloth_bend_constraint& constraint =
                        mesh.bend_constraints[constraint_index];
                    std::array<vec3, 4> points = {};
                    for (std::size_t local_index = 0; local_index < 4; ++local_index)
                    {
                        points[local_index] = result.positions[constraint.vertices[local_index]];
                    }
                    const cloth_bend_evaluation evaluation =
                        evaluate_cloth_bend(points, constraint.rest_angle);
                    if (!evaluation.valid)
                    {
                        add_diagnostic(result.diagnostics, cloth_error::degenerate_constraint,
                            static_cast<uint32_t>(constraint_index), "Cloth bend constraint became degenerate.");
                        return result;
                    }

                    const float alpha = settings.bend_compliance * inverse_dt_squared;
                    float denominator = alpha;
                    for (std::size_t local_index = 0; local_index < 4; ++local_index)
                    {
                        const uint32_t vertex_id = constraint.vertices[local_index];
                        denominator += inverse_mass[vertex_id] *
                            length_squared(evaluation.gradients[local_index]);
                    }
                    if (denominator <= 1.0e-20f)
                    {
                        continue;
                    }

                    const float delta_lambda =
                        (-evaluation.constraint - alpha * bend_lambda[constraint_index]) / denominator;
                    const float applied = settings.relaxation * delta_lambda;
                    bend_lambda[constraint_index] += applied;
                    for (std::size_t local_index = 0; local_index < 4; ++local_index)
                    {
                        const uint32_t vertex_id = constraint.vertices[local_index];
                        corrections[vertex_id] = add(corrections[vertex_id],
                            multiply(evaluation.gradients[local_index], inverse_mass[vertex_id] * applied));
                    }
                }
            }

            for (std::size_t vertex_index = 0; vertex_index < result.positions.size(); ++vertex_index)
            {
                result.positions[vertex_index] = add(result.positions[vertex_index], corrections[vertex_index]);
                if (!finite(result.positions[vertex_index]))
                {
                    add_diagnostic(result.diagnostics, cloth_error::non_finite_state,
                        static_cast<uint32_t>(vertex_index), "Cloth solver produced a non-finite position.");
                    result.positions.clear();
                    return result;
                }
            }
            ++result.iterations_completed;
        }

        result.success = true;
        return result;
    }
}
