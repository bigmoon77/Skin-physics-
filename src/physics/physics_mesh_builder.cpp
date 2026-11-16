#include "skin_physics/physics/physics_mesh_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <iterator>
#include <utility>

namespace skin_physics::physics
{
    namespace
    {
        using asset::vec3;
        using face_key = std::array<uint32_t, 3>;

        struct face_occurrence
        {
            std::array<uint32_t, 3> oriented_vertices = {};
            uint32_t tetrahedron_index = asset::invalid_index;
            uint8_t orientation_parity = 0;
            uint32_t count = 0;
        };

        uint8_t calculate_orientation_parity(const face_key& sorted_key, const face_key& oriented_face)
        {
            std::array<uint32_t, 3> permutation = {};
            for (std::size_t index = 0; index < 3; ++index)
            {
                const auto iterator = std::find(sorted_key.begin(), sorted_key.end(), oriented_face[index]);
                permutation[index] = static_cast<uint32_t>(std::distance(sorted_key.begin(), iterator));
            }

            uint32_t inversions = 0;
            for (std::size_t first = 0; first < 3; ++first)
            {
                for (std::size_t second = first + 1; second < 3; ++second)
                {
                    if (permutation[first] > permutation[second])
                    {
                        ++inversions;
                    }
                }
            }
            return static_cast<uint8_t>(inversions & 1u);
        }

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

        float determinant(const vec3& a, const vec3& b, const vec3& c)
        {
            return dot(a, cross(b, c));
        }

        bool finite(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        vec3 normalize(const vec3& value)
        {
            const float value_length_squared = length_squared(value);
            if (value_length_squared <= 1.0e-30f || !std::isfinite(value_length_squared))
            {
                return {};
            }
            return multiply(value, 1.0f / std::sqrt(value_length_squared));
        }

        float clamp_unit(float value)
        {
            return std::max(-1.0f, std::min(1.0f, value));
        }

        void add_diagnostic(
            physics_mesh_build_result& result,
            physics_mesh_error code,
            uint32_t element_index,
            const std::string& message)
        {
            physics_mesh_diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.element_index = element_index;
            diagnostic.message = message;
            result.diagnostics.push_back(std::move(diagnostic));
        }

        bool invert_rest_matrix(
            const vec3& edge_1,
            const vec3& edge_2,
            const vec3& edge_3,
            std::array<float, 9>& output,
            float determinant_value)
        {
            const float a = edge_1.x;
            const float b = edge_2.x;
            const float c = edge_3.x;
            const float d = edge_1.y;
            const float e = edge_2.y;
            const float f = edge_3.y;
            const float g = edge_1.z;
            const float h = edge_2.z;
            const float i = edge_3.z;

            const float reciprocal = 1.0f / determinant_value;
            output = {
                (e * i - f * h) * reciprocal,
                (c * h - b * i) * reciprocal,
                (b * f - c * e) * reciprocal,
                (f * g - d * i) * reciprocal,
                (a * i - c * g) * reciprocal,
                (c * d - a * f) * reciprocal,
                (d * h - e * g) * reciprocal,
                (b * g - a * h) * reciprocal,
                (a * e - b * d) * reciprocal
            };

            for (float value : output)
            {
                if (!std::isfinite(value))
                {
                    return false;
                }
            }
            return true;
        }

        vec3 inverse_row(const std::array<float, 9>& matrix, std::size_t row)
        {
            const std::size_t base = row * 3;
            return { matrix[base], matrix[base + 1], matrix[base + 2] };
        }

        float calculate_quality(float volume, float sum_edge_squared)
        {
            if (volume <= 0.0f || sum_edge_squared <= 0.0f)
            {
                return 0.0f;
            }
            return 12.0f * std::pow(3.0f * volume, 2.0f / 3.0f) / sum_edge_squared;
        }

        float calculate_minimum_dihedral(const std::array<vec3, 4>& positions)
        {
            const std::array<std::array<uint32_t, 3>, 4> faces = {{
                {{ 1, 2, 3 }},
                {{ 0, 3, 2 }},
                {{ 0, 1, 3 }},
                {{ 0, 2, 1 }}
            }};

            std::array<vec3, 4> normals = {};
            for (std::size_t face_index = 0; face_index < faces.size(); ++face_index)
            {
                const auto& face = faces[face_index];
                normals[face_index] = normalize(cross(
                    subtract(positions[face[1]], positions[face[0]]),
                    subtract(positions[face[2]], positions[face[0]])));
            }

            float minimum_angle = 180.0f;
            for (std::size_t first = 0; first < normals.size(); ++first)
            {
                for (std::size_t second = first + 1; second < normals.size(); ++second)
                {
                    const float exterior_angle = std::acos(clamp_unit(dot(normals[first], normals[second])));
                    const float interior_angle = 3.14159265358979323846f - exterior_angle;
                    minimum_angle = std::min(minimum_angle, interior_angle * 57.29577951308232f);
                }
            }
            return minimum_angle;
        }

        void make_oriented_faces(
            const tetrahedron& tet,
            std::array<std::array<uint32_t, 3>, 4>& output)
        {
            const uint32_t a = tet.vertices[0];
            const uint32_t b = tet.vertices[1];
            const uint32_t c = tet.vertices[2];
            const uint32_t d = tet.vertices[3];

            output = {{
                {{ b, c, d }},
                {{ a, d, c }},
                {{ a, b, d }},
                {{ a, c, b }}
            }};
        }

        bool build_geometry(
            const raw_tet_mesh& input,
            const physics_mesh_build_settings& settings,
            physics_mesh_build_result& result)
        {
            if (input.positions.empty() || input.tetrahedra.empty())
            {
                add_diagnostic(result, physics_mesh_error::empty_mesh, asset::invalid_index,
                    "The tetrahedral mesh must contain vertices and tetrahedra.");
                return false;
            }

            if (input.tetrahedra.size() > settings.maximum_tetrahedra)
            {
                add_diagnostic(result, physics_mesh_error::capacity_exceeded, asset::invalid_index,
                    "Tetrahedron count exceeds the configured maximum.");
                return false;
            }

            if (input.positions.size() > std::numeric_limits<uint32_t>::max())
            {
                add_diagnostic(result, physics_mesh_error::capacity_exceeded, asset::invalid_index,
                    "Physics vertex count exceeds 32-bit index capacity.");
                return false;
            }

            if (!std::isfinite(settings.density) || settings.density <= 0.0f ||
                !std::isfinite(settings.minimum_volume) || settings.minimum_volume < 0.0f ||
                !std::isfinite(settings.minimum_quality) || settings.minimum_quality < 0.0f ||
                !std::isfinite(settings.minimum_dihedral_degrees) ||
                settings.minimum_dihedral_degrees < 0.0f || settings.minimum_dihedral_degrees > 180.0f ||
                settings.maximum_tetrahedra == 0)
            {
                add_diagnostic(result, physics_mesh_error::invalid_density, asset::invalid_index,
                    "Density must be positive and minimum volume must be finite and non-negative.");
                return false;
            }

            result.value.rest_positions = input.positions;
            result.value.tetrahedra.reserve(input.tetrahedra.size());
            result.value.mass.assign(input.positions.size(), 0.0f);

            for (std::size_t tet_index = 0; tet_index < input.tetrahedra.size(); ++tet_index)
            {
                tetrahedron current_tet;
                current_tet.vertices = input.tetrahedra[tet_index];

                for (uint32_t vertex_index : current_tet.vertices)
                {
                    if (vertex_index >= input.positions.size())
                    {
                        add_diagnostic(result, physics_mesh_error::invalid_index,
                            static_cast<uint32_t>(tet_index), "Tetrahedron references an out-of-range vertex.");
                        return false;
                    }
                }

                for (uint32_t first = 0; first < 4; ++first)
                {
                    for (uint32_t second = first + 1; second < 4; ++second)
                    {
                        if (current_tet.vertices[first] == current_tet.vertices[second])
                        {
                            add_diagnostic(result, physics_mesh_error::degenerate_tetrahedron,
                                static_cast<uint32_t>(tet_index), "Tetrahedron contains duplicate vertex indices.");
                            return false;
                        }
                    }
                }

                std::array<vec3, 4> positions = {
                    input.positions[current_tet.vertices[0]],
                    input.positions[current_tet.vertices[1]],
                    input.positions[current_tet.vertices[2]],
                    input.positions[current_tet.vertices[3]]
                };

                for (const vec3& position : positions)
                {
                    if (!finite(position))
                    {
                        add_diagnostic(result, physics_mesh_error::non_finite_value,
                            static_cast<uint32_t>(tet_index), "Tetrahedron contains a non-finite position.");
                        return false;
                    }
                }

                vec3 edge_1 = subtract(positions[1], positions[0]);
                vec3 edge_2 = subtract(positions[2], positions[0]);
                vec3 edge_3 = subtract(positions[3], positions[0]);
                float signed_six_volume = determinant(edge_1, edge_2, edge_3);

                if (!std::isfinite(signed_six_volume) ||
                    std::abs(signed_six_volume) <= settings.minimum_volume * 6.0f)
                {
                    add_diagnostic(result, physics_mesh_error::degenerate_tetrahedron,
                        static_cast<uint32_t>(tet_index), "Tetrahedron volume is below the configured minimum.");
                    return false;
                }

                if (signed_six_volume < 0.0f)
                {
                    std::swap(current_tet.vertices[1], current_tet.vertices[2]);
                    std::swap(positions[1], positions[2]);
                    edge_1 = subtract(positions[1], positions[0]);
                    edge_2 = subtract(positions[2], positions[0]);
                    signed_six_volume = determinant(edge_1, edge_2, edge_3);
                }

                current_tet.rest_volume = signed_six_volume / 6.0f;
                if (!invert_rest_matrix(edge_1, edge_2, edge_3,
                    current_tet.rest_inverse, signed_six_volume))
                {
                    add_diagnostic(result, physics_mesh_error::degenerate_tetrahedron,
                        static_cast<uint32_t>(tet_index), "Could not invert tetrahedron rest matrix.");
                    return false;
                }

                const vec3 gradient_1 = inverse_row(current_tet.rest_inverse, 0);
                const vec3 gradient_2 = inverse_row(current_tet.rest_inverse, 1);
                const vec3 gradient_3 = inverse_row(current_tet.rest_inverse, 2);
                current_tet.rest_shape_gradients[0] = multiply(
                    add(add(gradient_1, gradient_2), gradient_3), -1.0f);
                current_tet.rest_shape_gradients[1] = gradient_1;
                current_tet.rest_shape_gradients[2] = gradient_2;
                current_tet.rest_shape_gradients[3] = gradient_3;

                float sum_edge_squared = 0.0f;
                for (uint32_t first = 0; first < 4; ++first)
                {
                    for (uint32_t second = first + 1; second < 4; ++second)
                    {
                        sum_edge_squared += length_squared(subtract(positions[first], positions[second]));
                    }
                }
                current_tet.quality = calculate_quality(current_tet.rest_volume, sum_edge_squared);
                current_tet.minimum_dihedral_degrees = calculate_minimum_dihedral(positions);

                if (!std::isfinite(current_tet.quality) ||
                    current_tet.quality < settings.minimum_quality ||
                    current_tet.minimum_dihedral_degrees < settings.minimum_dihedral_degrees)
                {
                    add_diagnostic(result, physics_mesh_error::invalid_quality,
                        static_cast<uint32_t>(tet_index), "Tetrahedron quality or minimum dihedral angle is below the configured threshold.");
                    return false;
                }

                const float mass_per_vertex = settings.density * current_tet.rest_volume * 0.25f;
                if (!std::isfinite(mass_per_vertex))
                {
                    add_diagnostic(result, physics_mesh_error::non_finite_value,
                        static_cast<uint32_t>(tet_index), "Tetrahedron mass is not finite.");
                    return false;
                }
                for (uint32_t vertex_index : current_tet.vertices)
                {
                    result.value.mass[vertex_index] += mass_per_vertex;
                    if (!std::isfinite(result.value.mass[vertex_index]))
                    {
                        add_diagnostic(result, physics_mesh_error::non_finite_value,
                            vertex_index, "Accumulated vertex mass is not finite.");
                        return false;
                    }
                }

                result.value.tetrahedra.push_back(current_tet);
            }

            result.value.inverse_mass.resize(result.value.mass.size(), 0.0f);
            for (std::size_t vertex_index = 0; vertex_index < result.value.mass.size(); ++vertex_index)
            {
                const float mass = result.value.mass[vertex_index];
                if (!std::isfinite(mass) || mass <= 0.0f)
                {
                    add_diagnostic(result, physics_mesh_error::invalid_topology,
                        static_cast<uint32_t>(vertex_index),
                        "Physics vertex is not referenced by any tetrahedron or has invalid accumulated mass.");
                    return false;
                }
                result.value.inverse_mass[vertex_index] = 1.0f / mass;
                if (!std::isfinite(result.value.inverse_mass[vertex_index]))
                {
                    add_diagnostic(result, physics_mesh_error::non_finite_value,
                        static_cast<uint32_t>(vertex_index), "Inverse mass is not finite.");
                    return false;
                }
            }

            return true;
        }

        bool build_vertex_tetrahedron_csr(physics_mesh& mesh, physics_mesh_build_result& result)
        {
            const std::size_t vertex_count = mesh.rest_positions.size();
            mesh.vertex_tetrahedron_offsets.assign(vertex_count + 1, 0);

            for (const tetrahedron& tet : mesh.tetrahedra)
            {
                for (uint32_t vertex_index : tet.vertices)
                {
                    if (mesh.vertex_tetrahedron_offsets[vertex_index + 1] == std::numeric_limits<uint32_t>::max())
                    {
                        add_diagnostic(result, physics_mesh_error::capacity_exceeded, vertex_index,
                            "Vertex-to-tetrahedron adjacency exceeds 32-bit capacity.");
                        return false;
                    }
                    ++mesh.vertex_tetrahedron_offsets[vertex_index + 1];
                }
            }

            for (std::size_t vertex_index = 1; vertex_index < mesh.vertex_tetrahedron_offsets.size(); ++vertex_index)
            {
                const uint64_t offset = static_cast<uint64_t>(mesh.vertex_tetrahedron_offsets[vertex_index - 1]) +
                    mesh.vertex_tetrahedron_offsets[vertex_index];
                if (offset > std::numeric_limits<uint32_t>::max())
                {
                    add_diagnostic(result, physics_mesh_error::capacity_exceeded,
                        static_cast<uint32_t>(vertex_index - 1), "Vertex-to-tetrahedron CSR exceeds 32-bit capacity.");
                    return false;
                }
                mesh.vertex_tetrahedron_offsets[vertex_index] = static_cast<uint32_t>(offset);
            }

            mesh.vertex_tetrahedron_ids.resize(mesh.vertex_tetrahedron_offsets.back());
            std::vector<uint32_t> cursor = mesh.vertex_tetrahedron_offsets;
            for (std::size_t tet_index = 0; tet_index < mesh.tetrahedra.size(); ++tet_index)
            {
                for (uint32_t vertex_index : mesh.tetrahedra[tet_index].vertices)
                {
                    const uint32_t destination = cursor[vertex_index]++;
                    mesh.vertex_tetrahedron_ids[destination] = static_cast<uint32_t>(tet_index);
                }
            }

            return true;
        }

        bool extract_boundary(physics_mesh& mesh, physics_mesh_build_result& result)
        {
            std::map<face_key, face_occurrence> faces;

            for (std::size_t tet_index = 0; tet_index < mesh.tetrahedra.size(); ++tet_index)
            {
                std::array<std::array<uint32_t, 3>, 4> oriented_faces = {};
                make_oriented_faces(mesh.tetrahedra[tet_index], oriented_faces);

                for (const std::array<uint32_t, 3>& oriented_face : oriented_faces)
                {
                    face_key key = oriented_face;
                    std::sort(key.begin(), key.end());

                    face_occurrence& occurrence = faces[key];
                    const uint8_t orientation_parity = calculate_orientation_parity(key, oriented_face);
                    if (occurrence.count == 0)
                    {
                        occurrence.oriented_vertices = oriented_face;
                        occurrence.tetrahedron_index = static_cast<uint32_t>(tet_index);
                        occurrence.orientation_parity = orientation_parity;
                    }
                    else if (occurrence.count == 1 &&
                        occurrence.orientation_parity == orientation_parity)
                    {
                        add_diagnostic(result, physics_mesh_error::non_manifold_volume,
                            static_cast<uint32_t>(tet_index),
                            "Adjacent tetrahedra use a shared face with the same orientation; the volume is overlapping or inconsistently oriented.");
                        return false;
                    }
                    ++occurrence.count;

                    if (occurrence.count > 2)
                    {
                        add_diagnostic(result, physics_mesh_error::non_manifold_volume,
                            static_cast<uint32_t>(tet_index), "A tetrahedron face is shared by more than two tetrahedra.");
                        return false;
                    }
                }
            }

            mesh.boundary_triangles.clear();
            for (const auto& entry : faces)
            {
                if (entry.second.count == 1)
                {
                    boundary_triangle boundary;
                    boundary.vertices = entry.second.oriented_vertices;
                    boundary.source_tetrahedron = entry.second.tetrahedron_index;
                    mesh.boundary_triangles.push_back(boundary);
                }
            }

            if (mesh.boundary_triangles.empty())
            {
                add_diagnostic(result, physics_mesh_error::non_manifold_volume, asset::invalid_index,
                    "The tetrahedral mesh has no boundary surface.");
                return false;
            }

            return true;
        }
    }

    physics_mesh_build_result build_physics_mesh(
        const raw_tet_mesh& input,
        const physics_mesh_build_settings& settings)
    {
        physics_mesh_build_result result;

        if (!build_geometry(input, settings, result))
        {
            return result;
        }
        if (!build_vertex_tetrahedron_csr(result.value, result))
        {
            return result;
        }
        if (!extract_boundary(result.value, result))
        {
            return result;
        }

        result.success = true;
        return result;
    }
}
