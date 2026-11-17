#include "skin_physics/physics/ccd.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace skin_physics::physics
{
    namespace
    {
        using asset::vec3;
        constexpr float minimum_length_squared = 1.0e-20f;

        vec3 add(const vec3& left, const vec3& right)
        {
            return { left.x + right.x, left.y + right.y, left.z + right.z };
        }

        vec3 subtract(const vec3& left, const vec3& right)
        {
            return { left.x - right.x, left.y - right.y, left.z - right.z };
        }

        vec3 multiply(const vec3& value, float scalar)
        {
            return { value.x * scalar, value.y * scalar, value.z * scalar };
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

        bool valid_settings(const ccd_settings& settings)
        {
            return std::isfinite(settings.safety) &&
                settings.safety > 0.0f && settings.safety <= 1.0f &&
                std::isfinite(settings.distance_tolerance) &&
                settings.distance_tolerance > 0.0f &&
                std::isfinite(settings.minimum_time_advance) &&
                settings.minimum_time_advance > 0.0f &&
                settings.maximum_iterations > 0;
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
    }

    point_triangle_ccd_result conservative_advance_point_triangle(
        const vec3& point_start,
        const vec3& point_end,
        const std::array<vec3, 3>& triangle,
        float combined_contact_radius,
        float interval_seconds,
        const ccd_settings& settings)
    {
        point_triangle_ccd_result result;
        if (!finite(point_start) || !finite(point_end) ||
            !finite(triangle[0]) || !finite(triangle[1]) || !finite(triangle[2]) ||
            !std::isfinite(combined_contact_radius) || combined_contact_radius < 0.0f ||
            !std::isfinite(interval_seconds) || interval_seconds <= 0.0f ||
            !valid_settings(settings))
        {
            result.diagnostic = "CCD input positions, radius, interval or iteration settings are invalid.";
            return result;
        }

        const vec3 displacement = subtract(point_end, point_start);
        const float speed = length(displacement) / interval_seconds;
        if (!std::isfinite(speed))
        {
            result.diagnostic = "CCD relative path speed is not finite.";
            return result;
        }

        float time = 0.0f;
        for (uint32_t iteration = 0; iteration < settings.maximum_iterations; ++iteration)
        {
            const float fraction = std::max(0.0f, std::min(1.0f, time / interval_seconds));
            const vec3 point = add(point_start, multiply(displacement, fraction));
            std::array<float, 3> barycentric = {};
            vec3 closest = {};
            if (!closest_point_triangle(point, triangle, barycentric, closest))
            {
                result.diagnostic = "CCD triangle is degenerate.";
                return result;
            }

            const vec3 separation = subtract(point, closest);
            const float distance = length(separation);
            if (!std::isfinite(distance))
            {
                result.diagnostic = "CCD distance evaluation produced a non-finite value.";
                return result;
            }

            result.iterations = iteration + 1;
            if (distance <= combined_contact_radius + settings.distance_tolerance)
            {
                result.status = ccd_status::hit;
                result.hit = true;
                result.toi_seconds = time;
                result.remaining_interval_seconds = std::max(0.0f, interval_seconds - time);
                result.contact_center = point;
                result.closest_point = closest;
                result.barycentric = barycentric;
                if (distance > 1.0e-10f)
                {
                    result.contact_normal = multiply(separation, 1.0f / distance);
                }
                else
                {
                    result.contact_normal = normalize(cross(
                        subtract(triangle[1], triangle[0]),
                        subtract(triangle[2], triangle[0])));
                }
                return result;
            }

            if (speed <= 1.0e-12f)
            {
                result.status = ccd_status::no_hit;
                result.diagnostic = "Relative path has zero speed and starts outside the contact radius.";
                return result;
            }

            const float gap = distance - combined_contact_radius;
            const float time_advance = settings.safety * gap / speed;
            if (!std::isfinite(time_advance) ||
                time_advance < settings.minimum_time_advance)
            {
                result.status = ccd_status::unresolved;
                result.diagnostic = "Conservative advancement reached its minimum time increment.";
                return result;
            }

            time += time_advance;
            if (time > interval_seconds)
            {
                const vec3 endpoint = point_end;
                std::array<float, 3> end_barycentric = {};
                vec3 end_closest = {};
                if (!closest_point_triangle(endpoint, triangle, end_barycentric, end_closest))
                {
                    result.diagnostic = "CCD endpoint triangle evaluation failed.";
                    return result;
                }
                const float endpoint_distance = length(subtract(endpoint, end_closest));
                result.status = endpoint_distance > combined_contact_radius
                    ? ccd_status::no_hit : ccd_status::unresolved;
                result.diagnostic = result.status == ccd_status::no_hit
                    ? "No contact occurs within the requested interval."
                    : "Conservative advancement crossed the interval without resolving a TOI.";
                return result;
            }
        }

        result.status = ccd_status::unresolved;
        result.diagnostic = "Conservative advancement reached its iteration limit.";
        return result;
    }

    toi_impulse_result resolve_toi_normal_impulse(
        const vec3& velocity_point,
        const vec3& velocity_triangle,
        const vec3& normal,
        float inverse_mass_point,
        float inverse_mass_triangle,
        float restitution,
        bool impact_already_resolved)
    {
        toi_impulse_result result;
        result.velocity_point_after = velocity_point;
        result.velocity_triangle_after = velocity_triangle;
        if (!finite(velocity_point) || !finite(velocity_triangle) ||
            !finite(normal) || length_squared(normal) <= minimum_length_squared ||
            !std::isfinite(inverse_mass_point) || inverse_mass_point < 0.0f ||
            !std::isfinite(inverse_mass_triangle) || inverse_mass_triangle < 0.0f ||
            !std::isfinite(restitution) || restitution < 0.0f || restitution > 1.0f)
        {
            return result;
        }
        if (impact_already_resolved)
        {
            result.impact_resolved = true;
            result.success = true;
            return result;
        }

        const vec3 unit_normal = normalize(normal);
        const vec3 relative_velocity = subtract(velocity_point, velocity_triangle);
        const float normal_velocity = dot(relative_velocity, unit_normal);
        const float denominator = inverse_mass_point + inverse_mass_triangle;
        if (normal_velocity < 0.0f && denominator > 1.0e-20f)
        {
            result.impulse_magnitude =
                -(1.0f + restitution) * normal_velocity / denominator;
            const vec3 impulse = multiply(unit_normal, result.impulse_magnitude);
            result.velocity_point_after = add(
                velocity_point, multiply(impulse, inverse_mass_point));
            result.velocity_triangle_after = subtract(
                velocity_triangle, multiply(impulse, inverse_mass_triangle));
            result.application_count = 1;
        }
        result.impact_resolved = true;
        result.success = finite(result.velocity_point_after) &&
            finite(result.velocity_triangle_after) &&
            std::isfinite(result.impulse_magnitude);
        return result;
    }

    initial_repair_result repair_initial_penetration(
        const std::vector<vec3>& initial_positions,
        const std::vector<uint32_t>& participant_vertex_ids,
        const vec3& direction,
        float requested_correction_distance_per_vertex,
        float maximum_correction_distance_per_vertex)
    {
        initial_repair_result result;
        result.positions = initial_positions;
        if (!finite(direction) || length_squared(direction) <= minimum_length_squared ||
            !std::isfinite(requested_correction_distance_per_vertex) ||
            requested_correction_distance_per_vertex < 0.0f ||
            !std::isfinite(maximum_correction_distance_per_vertex) ||
            maximum_correction_distance_per_vertex < 0.0f ||
            participant_vertex_ids.empty())
        {
            result.status = initial_repair_status::invalid_input;
            return result;
        }
        for (uint32_t vertex_id : participant_vertex_ids)
        {
            if (vertex_id >= initial_positions.size() || !finite(initial_positions[vertex_id]))
            {
                result.status = initial_repair_status::invalid_input;
                return result;
            }
        }

        result.maximum_requested_correction = requested_correction_distance_per_vertex;
        if (requested_correction_distance_per_vertex >
            maximum_correction_distance_per_vertex)
        {
            result.status = initial_repair_status::over_budget;
            result.state_committed = false;
            return result;
        }

        const vec3 correction = multiply(
            normalize(direction), requested_correction_distance_per_vertex);
        for (uint32_t vertex_id : participant_vertex_ids)
        {
            result.positions[vertex_id] = add(result.positions[vertex_id], correction);
        }
        result.status = initial_repair_status::success;
        result.state_committed = true;
        return result;
    }
}
