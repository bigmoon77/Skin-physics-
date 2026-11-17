#include "skin_physics/physics/collision.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <utility>

namespace skin_physics::physics
{
    namespace
    {
        using asset::vec3;
        using cell_key = std::tuple<int64_t, int64_t, int64_t>;
        constexpr float minimum_length_squared = 1.0e-20f;

        vec3 add(const vec3& a, const vec3& b)
        {
            return { a.x + b.x, a.y + b.y, a.z + b.z };
        }

        vec3 subtract(const vec3& a, const vec3& b)
        {
            return { a.x - b.x, a.y - b.y, a.z - b.z };
        }

        vec3 multiply(const vec3& a, float scale)
        {
            return { a.x * scale, a.y * scale, a.z * scale };
        }

        float dot(const vec3& a, const vec3& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        vec3 cross(const vec3& a, const vec3& b)
        {
            return {
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
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

            const float reciprocal = 1.0f / (va + vb + vc);
            const float v = vb * reciprocal;
            const float w = vc * reciprocal;
            barycentric = { 1.0f - v - w, v, w };
            closest = add(add(multiply(a, barycentric[0]),
                multiply(b, barycentric[1])), multiply(c, barycentric[2]));
            return finite(closest);
        }

        bool closest_segment_segment(
            const std::array<vec3, 2>& first,
            const std::array<vec3, 2>& second,
            float& parameter_first,
            float& parameter_second,
            vec3& point_first,
            vec3& point_second)
        {
            const vec3 d1 = subtract(first[1], first[0]);
            const vec3 d2 = subtract(second[1], second[0]);
            const vec3 r = subtract(first[0], second[0]);
            const float a = dot(d1, d1);
            const float e = dot(d2, d2);
            const float f = dot(d2, r);
            if (!std::isfinite(a) || !std::isfinite(e))
            {
                return false;
            }

            float s = 0.0f;
            float t = 0.0f;
            if (a <= minimum_length_squared && e <= minimum_length_squared)
            {
                s = 0.0f;
                t = 0.0f;
            }
            else if (a <= minimum_length_squared)
            {
                s = 0.0f;
                t = std::max(0.0f, std::min(1.0f, f / e));
            }
            else
            {
                const float c = dot(d1, r);
                if (e <= minimum_length_squared)
                {
                    t = 0.0f;
                    s = std::max(0.0f, std::min(1.0f, -c / a));
                }
                else
                {
                    const float b = dot(d1, d2);
                    const float denominator = a * e - b * b;
                    if (std::abs(denominator) > 1.0e-20f)
                    {
                        s = std::max(0.0f, std::min(1.0f, (b * f - c * e) / denominator));
                    }
                    t = (b * s + f) / e;
                    if (t < 0.0f)
                    {
                        t = 0.0f;
                        s = std::max(0.0f, std::min(1.0f, -c / a));
                    }
                    else if (t > 1.0f)
                    {
                        t = 1.0f;
                        s = std::max(0.0f, std::min(1.0f, (b - c) / a));
                    }
                }
            }

            parameter_first = s;
            parameter_second = t;
            point_first = add(first[0], multiply(d1, s));
            point_second = add(second[0], multiply(d2, t));
            return finite(point_first) && finite(point_second);
        }

        bool aabb_valid(const collision_aabb& bounds)
        {
            return finite(bounds.minimum) && finite(bounds.maximum) &&
                bounds.minimum.x <= bounds.maximum.x &&
                bounds.minimum.y <= bounds.maximum.y &&
                bounds.minimum.z <= bounds.maximum.z;
        }

        bool aabb_overlap(
            const collision_aabb& left,
            const collision_aabb& right,
            float expansion)
        {
            return left.minimum.x - expansion <= right.maximum.x + expansion &&
                left.maximum.x + expansion >= right.minimum.x - expansion &&
                left.minimum.y - expansion <= right.maximum.y + expansion &&
                left.maximum.y + expansion >= right.minimum.y - expansion &&
                left.minimum.z - expansion <= right.maximum.z + expansion &&
                left.maximum.z + expansion >= right.minimum.z - expansion;
        }

        bool cell_coordinate(float value, float cell_size, int64_t& output)
        {
            const double coordinate = std::floor(static_cast<double>(value) /
                static_cast<double>(cell_size));
            if (!std::isfinite(coordinate) ||
                coordinate < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
                coordinate > static_cast<double>(std::numeric_limits<int64_t>::max()))
            {
                return false;
            }
            output = static_cast<int64_t>(coordinate);
            return true;
        }

        bool mesh_pair_allowed(
            uint32_t mesh_a,
            uint32_t mesh_b,
            const std::vector<broadphase_mesh_pair>& mesh_pairs,
            bool self_collision_enabled)
        {
            if (mesh_a == mesh_b)
            {
                return self_collision_enabled;
            }

            const uint32_t first = std::min(mesh_a, mesh_b);
            const uint32_t second = std::max(mesh_a, mesh_b);
            for (const broadphase_mesh_pair& pair : mesh_pairs)
            {
                if (std::min(pair.mesh_a, pair.mesh_b) == first &&
                    std::max(pair.mesh_a, pair.mesh_b) == second)
                {
                    return pair.enabled;
                }
            }
            return false;
        }

        void add_diagnostic(
            broadphase_result& result,
            collision_error code,
            uint32_t primitive,
            const std::string& message)
        {
            collision_diagnostic item;
            item.code = code;
            item.primitive_index = primitive;
            item.message = message;
            result.diagnostics.push_back(std::move(item));
        }
    }

    vertex_triangle_contact find_vertex_triangle_contact(
        const vec3& point,
        const std::array<vec3, 3>& triangle,
        float point_radius)
    {
        vertex_triangle_contact result;
        if (!finite(point) || !finite(triangle[0]) ||
            !finite(triangle[1]) || !finite(triangle[2]) ||
            !std::isfinite(point_radius) || point_radius < 0.0f)
        {
            return result;
        }

        if (!closest_point_triangle(point, triangle, result.barycentric, result.closest_point))
        {
            return result;
        }

        const vec3 separation = subtract(point, result.closest_point);
        result.distance = length(separation);
        if (!std::isfinite(result.distance))
        {
            return result;
        }
        if (result.distance > 1.0e-10f)
        {
            result.normal = multiply(separation, 1.0f / result.distance);
        }
        else
        {
            result.normal = normalize(cross(
                subtract(triangle[1], triangle[0]),
                subtract(triangle[2], triangle[0])));
        }

        result.penetration_depth = std::max(0.0f, point_radius - result.distance);
        result.hit = result.penetration_depth > 0.0f;
        result.valid = finite(result.normal) && length_squared(result.normal) > minimum_length_squared;
        return result;
    }

    vertex_triangle_solve_result solve_vertex_triangle_contact(
        const vertex_triangle_contact& contact,
        const std::array<vec3, 4>& positions,
        const std::array<float, 4>& inverse_masses,
        float point_radius,
        float relaxation)
    {
        vertex_triangle_solve_result result;
        result.positions = positions;
        if (!contact.valid || !std::isfinite(point_radius) || point_radius < 0.0f ||
            !std::isfinite(relaxation) || relaxation < 0.0f || relaxation > 1.0f)
        {
            return result;
        }
        for (float inverse_mass : inverse_masses)
        {
            if (!std::isfinite(inverse_mass) || inverse_mass < 0.0f)
            {
                return result;
            }
        }

        if (!contact.hit)
        {
            result.success = true;
            return result;
        }

        float denominator = inverse_masses[0];
        for (std::size_t index = 0; index < 3; ++index)
        {
            denominator += inverse_masses[index + 1] *
                contact.barycentric[index] * contact.barycentric[index];
        }
        if (!std::isfinite(denominator) || denominator <= 1.0e-20f)
        {
            return result;
        }

        const float lambda = relaxation * contact.penetration_depth / denominator;
        std::array<vec3, 4> corrections = {};
        corrections[0] = multiply(contact.normal, inverse_masses[0] * lambda);
        for (std::size_t index = 0; index < 3; ++index)
        {
            corrections[index + 1] = multiply(contact.normal,
                -inverse_masses[index + 1] * contact.barycentric[index] * lambda);
        }

        vec3 momentum_residual = {};
        for (std::size_t index = 0; index < 4; ++index)
        {
            result.positions[index] = add(result.positions[index], corrections[index]);
            if (inverse_masses[index] > 0.0f)
            {
                momentum_residual = add(momentum_residual,
                    multiply(corrections[index], 1.0f / inverse_masses[index]));
            }
        }
        result.momentum_conserved = length(momentum_residual) <= 1.0e-5f;

        const std::array<vec3, 3> corrected_triangle = {
            result.positions[1], result.positions[2], result.positions[3]
        };
        std::array<float, 3> barycentric = {};
        vec3 closest = {};
        if (!closest_point_triangle(result.positions[0], corrected_triangle, barycentric, closest))
        {
            return result;
        }
        const float post_distance = length(subtract(result.positions[0], closest));
        result.post_solve_penetration = std::max(0.0f, point_radius - post_distance);
        result.success = true;
        return result;
    }

    edge_edge_contact find_edge_edge_contact(
        const std::array<vec3, 2>& edge_a,
        const std::array<vec3, 2>& edge_b,
        float combined_thickness)
    {
        edge_edge_contact result;
        if (!finite(edge_a[0]) || !finite(edge_a[1]) ||
            !finite(edge_b[0]) || !finite(edge_b[1]) ||
            !std::isfinite(combined_thickness) || combined_thickness < 0.0f)
        {
            return result;
        }

        if (!closest_segment_segment(
            edge_a, edge_b,
            result.parameter_a, result.parameter_b,
            result.closest_point_a, result.closest_point_b))
        {
            return result;
        }

        const vec3 separation = subtract(result.closest_point_a, result.closest_point_b);
        result.distance = length(separation);
        if (!std::isfinite(result.distance))
        {
            return result;
        }
        if (result.distance > 1.0e-10f)
        {
            result.normal = multiply(separation, 1.0f / result.distance);
        }
        else
        {
            result.normal = normalize(cross(
                subtract(edge_a[1], edge_a[0]),
                subtract(edge_b[1], edge_b[0])));
            if (length_squared(result.normal) <= minimum_length_squared)
            {
                result.normal = { 0.0f, 0.0f, 1.0f };
            }
        }
        result.penetration_depth = std::max(0.0f, combined_thickness - result.distance);
        result.hit = result.penetration_depth > 0.0f;
        result.valid = finite(result.normal) && length_squared(result.normal) > minimum_length_squared;
        return result;
    }

    broadphase_result generate_broadphase_candidates(
        const std::vector<broadphase_primitive>& primitives,
        const std::vector<broadphase_mesh_pair>& mesh_pairs,
        const broadphase_settings& settings)
    {
        broadphase_result result;
        if (!std::isfinite(settings.cell_size) || settings.cell_size <= 0.0f ||
            !std::isfinite(settings.contact_thickness) || settings.contact_thickness < 0.0f ||
            settings.max_cells_per_primitive == 0)
        {
            add_diagnostic(result, collision_error::invalid_settings,
                asset::invalid_index, "Broadphase cell settings and capacities are invalid.");
            return result;
        }

        std::map<cell_key, std::vector<uint32_t>> cells;
        const float expansion = settings.contact_thickness * 0.5f;
        for (std::size_t primitive_index = 0; primitive_index < primitives.size(); ++primitive_index)
        {
            const collision_aabb& bounds = primitives[primitive_index].bounds;
            if (!aabb_valid(bounds))
            {
                add_diagnostic(result, collision_error::invalid_geometry,
                    static_cast<uint32_t>(primitive_index), "Broadphase primitive AABB is invalid.");
                return result;
            }

            int64_t minimum[3] = {};
            int64_t maximum[3] = {};
            const float values_min[3] = {
                bounds.minimum.x - expansion,
                bounds.minimum.y - expansion,
                bounds.minimum.z - expansion
            };
            const float values_max[3] = {
                bounds.maximum.x + expansion,
                bounds.maximum.y + expansion,
                bounds.maximum.z + expansion
            };
            for (uint32_t axis = 0; axis < 3; ++axis)
            {
                if (!cell_coordinate(values_min[axis], settings.cell_size, minimum[axis]) ||
                    !cell_coordinate(values_max[axis], settings.cell_size, maximum[axis]) ||
                    maximum[axis] < minimum[axis])
                {
                    add_diagnostic(result, collision_error::invalid_geometry,
                        static_cast<uint32_t>(primitive_index), "AABB cell coordinate overflows its integer range.");
                    return result;
                }
            }

            std::size_t cell_count = 1;
            for (uint32_t axis = 0; axis < 3; ++axis)
            {
                const uint64_t extent = static_cast<uint64_t>(maximum[axis] - minimum[axis]) + 1u;
                if (extent > settings.max_cells_per_primitive ||
                    cell_count > settings.max_cells_per_primitive / static_cast<std::size_t>(extent))
                {
                    add_diagnostic(result, collision_error::cell_capacity_overflow,
                        static_cast<uint32_t>(primitive_index), "Primitive AABB exceeds max_cells_per_primitive.");
                    result.overflow = true;
                    return result;
                }
                cell_count *= static_cast<std::size_t>(extent);
            }

            if (result.required_cell_records >
                std::numeric_limits<std::size_t>::max() - cell_count)
            {
                add_diagnostic(result, collision_error::cell_capacity_overflow,
                    static_cast<uint32_t>(primitive_index), "Cell record count overflows size_t.");
                result.overflow = true;
                return result;
            }
            result.required_cell_records += cell_count;
            if (result.required_cell_records > settings.cell_record_capacity)
            {
                add_diagnostic(result, collision_error::cell_capacity_overflow,
                    static_cast<uint32_t>(primitive_index), "Broadphase cell record buffer capacity exceeded.");
                result.overflow = true;
                return result;
            }

            for (int64_t x = minimum[0]; x <= maximum[0]; ++x)
            {
                for (int64_t y = minimum[1]; y <= maximum[1]; ++y)
                {
                    for (int64_t z = minimum[2]; z <= maximum[2]; ++z)
                    {
                        cells[{ x, y, z }].push_back(static_cast<uint32_t>(primitive_index));
                    }
                }
            }
        }

        std::set<collision_candidate> unique_candidates;
        for (const auto& cell : cells)
        {
            const std::vector<uint32_t>& primitive_ids = cell.second;
            for (std::size_t first_index = 0; first_index < primitive_ids.size(); ++first_index)
            {
                for (std::size_t second_index = first_index + 1;
                    second_index < primitive_ids.size(); ++second_index)
                {
                    const uint32_t first_id = primitive_ids[first_index];
                    const uint32_t second_id = primitive_ids[second_index];
                    if (first_id == second_id)
                    {
                        continue;
                    }

                    const broadphase_primitive& first = primitives[first_id];
                    const broadphase_primitive& second = primitives[second_id];
                    if (!mesh_pair_allowed(first.mesh_id, second.mesh_id,
                        mesh_pairs, settings.self_collision_enabled))
                    {
                        continue;
                    }
                    if (!aabb_overlap(first.bounds, second.bounds, expansion))
                    {
                        continue;
                    }
                    unique_candidates.insert({
                        std::min(first_id, second_id),
                        std::max(first_id, second_id)
                    });
                }
            }
        }

        result.required_candidates = unique_candidates.size();
        if (result.required_candidates > settings.candidate_capacity)
        {
            add_diagnostic(result, collision_error::candidate_capacity_overflow,
                asset::invalid_index, "Broadphase candidate buffer capacity exceeded; no candidates were committed.");
            result.overflow = true;
            return result;
        }

        result.candidates.assign(unique_candidates.begin(), unique_candidates.end());
        result.success = true;
        return result;
    }

    capacity_check_result check_candidate_capacity(
        std::size_t required_count,
        std::size_t capacity)
    {
        capacity_check_result result;
        result.required_count = required_count;
        result.capacity = capacity;
        result.overflow = required_count > capacity;
        result.state_committable = !result.overflow;
        return result;
    }
}
