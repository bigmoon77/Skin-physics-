#include "skin_physics/physics/correction_ccd.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace skin_physics::physics
{
    namespace
    {
        using real = long double;
        using asset::vec3;

        constexpr real zero = 0.0L;
        constexpr real one = 1.0L;
        constexpr real interval_error_factor = 128.0L;

        struct vector3
        {
            real x = 0.0L;
            real y = 0.0L;
            real z = 0.0L;
        };

        struct interval
        {
            real lower = 0.0L;
            real upper = 0.0L;
            bool valid = true;
        };

        struct interval_vector3
        {
            interval x;
            interval y;
            interval z;
        };

        struct segment
        {
            real lower = 0.0L;
            real upper = 1.0L;
        };

        struct distance_sample
        {
            bool valid = false;
            real distance_lower = 0.0L;
            real distance_upper = 0.0L;
            vector3 point_a;
            vector3 point_b;
            std::array<real, 3> barycentric = {};
            real parameter_a = 0.0L;
            real parameter_b = 0.0L;
        };

        real negative_infinity()
        {
            return -std::numeric_limits<real>::infinity();
        }

        real positive_infinity()
        {
            return std::numeric_limits<real>::infinity();
        }

        real round_down(real value)
        {
            return std::nextafter(value, negative_infinity());
        }

        real round_up(real value)
        {
            return std::nextafter(value, positive_infinity());
        }

        real clamp_real(real value, real minimum, real maximum)
        {
            return std::max(minimum, std::min(maximum, value));
        }

        vector3 to_vector(const vec3& value)
        {
            return {
                static_cast<real>(value.x),
                static_cast<real>(value.y),
                static_cast<real>(value.z)
            };
        }

        vec3 to_asset_vector(const vector3& value)
        {
            return {
                static_cast<float>(value.x),
                static_cast<float>(value.y),
                static_cast<float>(value.z)
            };
        }

        vector3 add(const vector3& left, const vector3& right)
        {
            return { left.x + right.x, left.y + right.y, left.z + right.z };
        }

        vector3 subtract(const vector3& left, const vector3& right)
        {
            return { left.x - right.x, left.y - right.y, left.z - right.z };
        }

        vector3 multiply(const vector3& value, real scalar)
        {
            return { value.x * scalar, value.y * scalar, value.z * scalar };
        }

        real dot(const vector3& left, const vector3& right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }

        vector3 cross(const vector3& left, const vector3& right)
        {
            return {
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x
            };
        }

        real length_squared(const vector3& value)
        {
            return dot(value, value);
        }

        real length(const vector3& value)
        {
            return std::sqrt(std::max(zero, length_squared(value)));
        }

        bool finite(real value)
        {
            return std::isfinite(value) != 0;
        }

        bool finite(const vector3& value)
        {
            return finite(value.x) && finite(value.y) && finite(value.z);
        }

        bool finite(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) &&
                std::isfinite(value.z);
        }

        vector3 normalized(const vector3& value)
        {
            const real magnitude = length(value);
            if (!finite(magnitude) || magnitude <= 0.0L)
            {
                return {};
            }
            return multiply(value, one / magnitude);
        }

        real maximum_coordinate_magnitude(const vector3& a)
        {
            return std::max({
                1.0L,
                std::abs(a.x),
                std::abs(a.y),
                std::abs(a.z)
            });
        }

        real maximum_coordinate_magnitude(const vector3& a, const vector3& b)
        {
            return std::max(maximum_coordinate_magnitude(a),
                maximum_coordinate_magnitude(b));
        }

        real distance_error(const vector3& a, const vector3& b, real distance)
        {
            const real scale = std::max(maximum_coordinate_magnitude(a, b),
                std::abs(distance));
            return round_up(interval_error_factor *
                std::numeric_limits<real>::epsilon() * scale);
        }

        interval make_interval(real lower, real upper)
        {
            interval result;
            result.lower = lower;
            result.upper = upper;
            result.valid = finite(lower) && finite(upper) && lower <= upper;
            return result;
        }

        interval exact_interval(real value)
        {
            return make_interval(value, value);
        }

        interval interval_add(const interval& a, const interval& b)
        {
            if (!a.valid || !b.valid) return { 0.0L, 0.0L, false };
            return make_interval(
                round_down(a.lower + b.lower),
                round_up(a.upper + b.upper));
        }

        interval interval_subtract(const interval& a, const interval& b)
        {
            if (!a.valid || !b.valid) return { 0.0L, 0.0L, false };
            return make_interval(
                round_down(a.lower - b.upper),
                round_up(a.upper - b.lower));
        }

        interval interval_multiply(const interval& a, const interval& b)
        {
            if (!a.valid || !b.valid) return { 0.0L, 0.0L, false };
            const real products[4] = {
                a.lower * b.lower,
                a.lower * b.upper,
                a.upper * b.lower,
                a.upper * b.upper
            };
            const real minimum = *std::min_element(products, products + 4);
            const real maximum = *std::max_element(products, products + 4);
            if (!finite(minimum) || !finite(maximum))
            {
                return { 0.0L, 0.0L, false };
            }
            return make_interval(round_down(minimum), round_up(maximum));
        }

        interval interval_scale(const interval& value, real scalar)
        {
            return interval_multiply(value, exact_interval(scalar));
        }

        interval interval_divide_positive(
            const interval& numerator,
            const interval& denominator)
        {
            if (!numerator.valid || !denominator.valid ||
                denominator.lower <= 0.0L)
            {
                return { 0.0L, 0.0L, false };
            }
            const interval reciprocal = make_interval(
                round_down(one / denominator.upper),
                round_up(one / denominator.lower));
            return interval_multiply(numerator, reciprocal);
        }

        interval interval_square(const interval& value)
        {
            if (!value.valid) return { 0.0L, 0.0L, false };
            const real maximum = std::max(
                value.lower * value.lower, value.upper * value.upper);
            real minimum = 0.0L;
            if (value.lower > 0.0L || value.upper < 0.0L)
            {
                minimum = std::min(
                    value.lower * value.lower, value.upper * value.upper);
            }
            return make_interval(
                std::max(0.0L, round_down(minimum)),
                round_up(maximum));
        }

        interval interval_sqrt(const interval& value)
        {
            if (!value.valid || value.upper < 0.0L)
            {
                return { 0.0L, 0.0L, false };
            }
            const real lower_value = std::sqrt(std::max(0.0L, value.lower));
            const real upper_value = std::sqrt(std::max(0.0L, value.upper));
            return make_interval(
                std::max(0.0L, round_down(lower_value)),
                round_up(upper_value));
        }

        interval interval_dot(
            const interval_vector3& a,
            const vector3& b)
        {
            const interval x = interval_scale(a.x, b.x);
            const interval y = interval_scale(a.y, b.y);
            const interval z = interval_scale(a.z, b.z);
            return interval_add(interval_add(x, y), z);
        }

        interval interval_dot(
            const interval_vector3& a,
            const interval_vector3& b)
        {
            return interval_add(interval_add(
                interval_multiply(a.x, b.x),
                interval_multiply(a.y, b.y)),
                interval_multiply(a.z, b.z));
        }

        interval_vector3 interval_add(
            const interval_vector3& a,
            const interval_vector3& b)
        {
            return {
                interval_add(a.x, b.x),
                interval_add(a.y, b.y),
                interval_add(a.z, b.z)
            };
        }

        interval_vector3 interval_subtract(
            const interval_vector3& a,
            const interval_vector3& b)
        {
            return {
                interval_subtract(a.x, b.x),
                interval_subtract(a.y, b.y),
                interval_subtract(a.z, b.z)
            };
        }

        interval_vector3 interval_cross(
            const interval_vector3& a,
            const interval_vector3& b)
        {
            return {
                interval_subtract(
                    interval_multiply(a.y, b.z),
                    interval_multiply(a.z, b.y)),
                interval_subtract(
                    interval_multiply(a.z, b.x),
                    interval_multiply(a.x, b.z)),
                interval_subtract(
                    interval_multiply(a.x, b.y),
                    interval_multiply(a.y, b.x))
            };
        }

        interval interval_vector_length(const interval_vector3& value)
        {
            const interval squared = interval_add(
                interval_add(
                    interval_square(value.x),
                    interval_square(value.y)),
                interval_square(value.z));
            return interval_sqrt(squared);
        }

        interval interval_component(
            real before,
            real after,
            const segment& range)
        {
            const interval s = make_interval(
                round_down(range.lower), round_up(range.upper));
            const interval delta = interval_subtract(
                exact_interval(after), exact_interval(before));
            return interval_add(exact_interval(before),
                interval_multiply(delta, s));
        }

        interval_vector3 interval_lerp(
            const vector3& before,
            const vector3& after,
            const segment& range)
        {
            return {
                interval_component(before.x, after.x, range),
                interval_component(before.y, after.y, range),
                interval_component(before.z, after.z, range)
            };
        }

        vector3 lerp(const vector3& before, const vector3& after, real s)
        {
            return add(before, multiply(subtract(after, before), s));
        }

        bool valid_settings(const correction_ccd_settings& settings)
        {
            return std::isfinite(settings.contact_thickness) &&
                settings.contact_thickness >= 0.0f &&
                std::isfinite(settings.distance_tolerance) &&
                settings.distance_tolerance > 0.0f &&
                std::isfinite(settings.epsilon_length) &&
                settings.epsilon_length > 0.0f &&
                settings.maximum_interval_evaluations > 0 &&
                settings.maximum_interval_evaluations <= 32;
        }

        real maximum_displacement(
            const correction_vertex_motion& motion)
        {
            return length(subtract(
                to_vector(motion.after), to_vector(motion.before)));
        }

        real maximum_displacement(
            const std::array<correction_vertex_motion, 3>& motions)
        {
            return std::max({
                maximum_displacement(motions[0]),
                maximum_displacement(motions[1]),
                maximum_displacement(motions[2])
            });
        }

        real maximum_displacement(
            const std::array<correction_vertex_motion, 2>& motions)
        {
            return std::max(
                maximum_displacement(motions[0]),
                maximum_displacement(motions[1]));
        }

        real conservative_speed_bound(real value)
        {
            const real error = interval_error_factor *
                std::numeric_limits<real>::epsilon() * std::max(1.0L, value);
            return round_up(value + error);
        }

        bool valid_vt_motion(const vertex_triangle_correction_motion& motion)
        {
            if (!finite(motion.point.before) || !finite(motion.point.after) ||
                !finite(motion.segment_start_point))
            {
                return false;
            }
            for (std::size_t index = 0; index < 3; ++index)
            {
                if (!finite(motion.triangle[index].before) ||
                    !finite(motion.triangle[index].after) ||
                    !finite(motion.segment_start_triangle[index]))
                {
                    return false;
                }
            }
            return !motion.has_valid_persistent_contact_cache ||
                finite(motion.persistent_contact_normal);
        }

        bool valid_ee_motion(const edge_edge_correction_motion& motion)
        {
            for (std::size_t index = 0; index < 2; ++index)
            {
                if (!finite(motion.edge_a[index].before) ||
                    !finite(motion.edge_a[index].after) ||
                    !finite(motion.edge_b[index].before) ||
                    !finite(motion.edge_b[index].after) ||
                    !finite(motion.segment_start_edge_a[index]) ||
                    !finite(motion.segment_start_edge_b[index]))
                {
                    return false;
                }
            }
            return !motion.has_valid_persistent_contact_cache ||
                finite(motion.persistent_contact_normal);
        }

        real clamp_unit(real value)
        {
            return clamp_real(value, 0.0L, 1.0L);
        }

        vector3 closest_point_on_segment(
            const vector3& point,
            const vector3& start,
            const vector3& end,
            real& parameter)
        {
            const vector3 direction = subtract(end, start);
            const real denominator = length_squared(direction);
            parameter = denominator > 0.0L
                ? clamp_unit(dot(subtract(point, start), direction) / denominator)
                : 0.0L;
            return add(start, multiply(direction, parameter));
        }

        bool closest_point_triangle(
            const vector3& point,
            const std::array<vector3, 3>& triangle,
            real epsilon_length,
            vector3& closest,
            std::array<real, 3>& barycentric)
        {
            const vector3 ab = subtract(triangle[1], triangle[0]);
            const vector3 ac = subtract(triangle[2], triangle[0]);
            const vector3 normal = cross(ab, ac);
            const real area_squared = length_squared(normal);
            const real epsilon_area = epsilon_length * epsilon_length;
            if (!finite(area_squared) || area_squared <=
                epsilon_area * epsilon_area)
            {
                return false;
            }

            const vector3 relative = subtract(point, triangle[0]);
            const real d00 = dot(ab, ab);
            const real d01 = dot(ab, ac);
            const real d11 = dot(ac, ac);
            const real d20 = dot(relative, ab);
            const real d21 = dot(relative, ac);
            const real denominator = d00 * d11 - d01 * d01;
            if (!finite(denominator) || denominator <= epsilon_area * epsilon_area)
            {
                return false;
            }

            const real projected_v = (d11 * d20 - d01 * d21) / denominator;
            const real projected_w = (d00 * d21 - d01 * d20) / denominator;
            const real projected_u = 1.0L - projected_v - projected_w;
            if (projected_u >= 0.0L && projected_v >= 0.0L &&
                projected_w >= 0.0L)
            {
                barycentric = { projected_u, projected_v, projected_w };
                closest = add(triangle[0],
                    add(multiply(ab, projected_v), multiply(ac, projected_w)));
                return finite(closest);
            }

            real best_distance_squared = positive_infinity();
            const vector3 edge_starts[3] = {
                triangle[0], triangle[1], triangle[2]
            };
            const vector3 edge_ends[3] = {
                triangle[1], triangle[2], triangle[0]
            };
            const std::size_t edge_indices[3][2] = {
                { 0, 1 }, { 1, 2 }, { 2, 0 }
            };

            for (std::size_t edge = 0; edge < 3; ++edge)
            {
                real parameter = 0.0L;
                const vector3 candidate = closest_point_on_segment(
                    point, edge_starts[edge], edge_ends[edge], parameter);
                const real distance_squared =
                    length_squared(subtract(point, candidate));
                if (distance_squared < best_distance_squared)
                {
                    best_distance_squared = distance_squared;
                    closest = candidate;
                    const std::size_t first = edge_indices[edge][0];
                    const std::size_t second = edge_indices[edge][1];
                    barycentric = {};
                    barycentric[first] = 1.0L - parameter;
                    barycentric[second] = parameter;
                }
            }

            return finite(closest) && finite(best_distance_squared);
        }

        bool closest_points_segments(
            const vector3& a0,
            const vector3& a1,
            const vector3& b0,
            const vector3& b1,
            real epsilon_length,
            vector3& closest_a,
            vector3& closest_b,
            real& parameter_a,
            real& parameter_b)
        {
            const vector3 direction_a = subtract(a1, a0);
            const vector3 direction_b = subtract(b1, b0);
            const vector3 relative = subtract(a0, b0);
            const real length_a_squared = length_squared(direction_a);
            const real length_b_squared = length_squared(direction_b);
            const real epsilon_squared = epsilon_length * epsilon_length;
            if (!finite(length_a_squared) || !finite(length_b_squared) ||
                length_a_squared <= epsilon_squared ||
                length_b_squared <= epsilon_squared)
            {
                return false;
            }

            const real a = length_a_squared;
            const real e = length_b_squared;
            const real b = dot(direction_a, direction_b);
            const real c = dot(direction_a, relative);
            const real f = dot(direction_b, relative);
            const real denominator = a * e - b * b;

            parameter_a = denominator > epsilon_squared * epsilon_squared
                ? clamp_unit((b * f - c * e) / denominator)
                : 0.0L;
            parameter_b = (b * parameter_a + f) / e;

            if (parameter_b < 0.0L)
            {
                parameter_b = 0.0L;
                parameter_a = clamp_unit(-c / a);
            }
            else if (parameter_b > 1.0L)
            {
                parameter_b = 1.0L;
                parameter_a = clamp_unit((b - c) / a);
            }

            closest_a = add(a0, multiply(direction_a, parameter_a));
            closest_b = add(b0, multiply(direction_b, parameter_b));
            return finite(closest_a) && finite(closest_b);
        }

        real coordinate_scale(
            const vector3& a,
            const vector3& b,
            const vector3& c)
        {
            return std::max({
                maximum_coordinate_magnitude(a),
                maximum_coordinate_magnitude(b),
                maximum_coordinate_magnitude(c)
            });
        }

        real aabb_gap(
            real a_minimum,
            real a_maximum,
            real b_minimum,
            real b_maximum)
        {
            if (a_maximum < b_minimum) return b_minimum - a_maximum;
            if (b_maximum < a_minimum) return a_minimum - b_maximum;
            return 0.0L;
        }

        real distance_lower_from_aabb(
            const vector3& point,
            const std::array<vector3, 3>& triangle)
        {
            real minimum[3] = {
                triangle[0].x, triangle[0].y, triangle[0].z
            };
            real maximum[3] = {
                triangle[0].x, triangle[0].y, triangle[0].z
            };
            const real coordinates[3][3] = {
                { triangle[0].x, triangle[1].x, triangle[2].x },
                { triangle[0].y, triangle[1].y, triangle[2].y },
                { triangle[0].z, triangle[1].z, triangle[2].z }
            };
            const real point_coordinates[3] = { point.x, point.y, point.z };
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                minimum[axis] = *std::min_element(coordinates[axis],
                    coordinates[axis] + 3);
                maximum[axis] = *std::max_element(coordinates[axis],
                    coordinates[axis] + 3);
            }
            const real gap_x = aabb_gap(point_coordinates[0], point_coordinates[0],
                minimum[0], maximum[0]);
            const real gap_y = aabb_gap(point_coordinates[1], point_coordinates[1],
                minimum[1], maximum[1]);
            const real gap_z = aabb_gap(point_coordinates[2], point_coordinates[2],
                minimum[2], maximum[2]);
            const real distance = std::sqrt(
                gap_x * gap_x + gap_y * gap_y + gap_z * gap_z);
            const real error = interval_error_factor *
                std::numeric_limits<real>::epsilon() *
                std::max(coordinate_scale(point, triangle[0], triangle[1]),
                    maximum_coordinate_magnitude(triangle[2]));
            return std::max(0.0L, round_down(distance - error));
        }

        real distance_lower_from_aabb(
            const std::array<vector3, 2>& edge_a,
            const std::array<vector3, 2>& edge_b)
        {
            real minimum_a[3] = {
                std::min(edge_a[0].x, edge_a[1].x),
                std::min(edge_a[0].y, edge_a[1].y),
                std::min(edge_a[0].z, edge_a[1].z)
            };
            real maximum_a[3] = {
                std::max(edge_a[0].x, edge_a[1].x),
                std::max(edge_a[0].y, edge_a[1].y),
                std::max(edge_a[0].z, edge_a[1].z)
            };
            real minimum_b[3] = {
                std::min(edge_b[0].x, edge_b[1].x),
                std::min(edge_b[0].y, edge_b[1].y),
                std::min(edge_b[0].z, edge_b[1].z)
            };
            real maximum_b[3] = {
                std::max(edge_b[0].x, edge_b[1].x),
                std::max(edge_b[0].y, edge_b[1].y),
                std::max(edge_b[0].z, edge_b[1].z)
            };
            const real gap_x = aabb_gap(
                minimum_a[0], maximum_a[0], minimum_b[0], maximum_b[0]);
            const real gap_y = aabb_gap(
                minimum_a[1], maximum_a[1], minimum_b[1], maximum_b[1]);
            const real gap_z = aabb_gap(
                minimum_a[2], maximum_a[2], minimum_b[2], maximum_b[2]);
            const real distance = std::sqrt(
                gap_x * gap_x + gap_y * gap_y + gap_z * gap_z);
            const real scale = std::max({
                maximum_coordinate_magnitude(edge_a[0]),
                maximum_coordinate_magnitude(edge_a[1]),
                maximum_coordinate_magnitude(edge_b[0]),
                maximum_coordinate_magnitude(edge_b[1])
            });
            const real error = interval_error_factor *
                std::numeric_limits<real>::epsilon() * scale;
            return std::max(0.0L, round_down(distance - error));
        }

        distance_sample sample_vertex_triangle(
            const vector3& point,
            const std::array<vector3, 3>& triangle,
            real epsilon_length)
        {
            distance_sample result;
            vector3 closest;
            if (!closest_point_triangle(
                point, triangle, epsilon_length, closest, result.barycentric))
            {
                return result;
            }
            const real distance = length(subtract(point, closest));
            if (!finite(distance)) return result;
            const real error = distance_error(point, closest, distance);
            result.valid = true;
            result.distance_lower = std::max(
                0.0L, round_down(distance - error));
            result.distance_upper = round_up(distance + error);
            result.point_a = point;
            result.point_b = closest;
            return result;
        }

        distance_sample sample_edge_edge(
            const std::array<vector3, 2>& edge_a,
            const std::array<vector3, 2>& edge_b,
            real epsilon_length)
        {
            distance_sample result;
            vector3 closest_a;
            vector3 closest_b;
            if (!closest_points_segments(
                edge_a[0], edge_a[1], edge_b[0], edge_b[1],
                epsilon_length, closest_a, closest_b,
                result.parameter_a, result.parameter_b))
            {
                return result;
            }
            const real distance = length(subtract(closest_a, closest_b));
            if (!finite(distance)) return result;
            const real error = distance_error(closest_a, closest_b, distance);
            result.valid = true;
            result.distance_lower = std::max(
                0.0L, round_down(distance - error));
            result.distance_upper = round_up(distance + error);
            result.point_a = closest_a;
            result.point_b = closest_b;
            return result;
        }

        bool latch_reference_side(
            const vector3& initial_normal,
            const vector3& side_vector,
            bool has_cache,
            const vec3& cached_normal,
            real epsilon_length,
            vector3& reference_normal,
            int8_t& side_sign,
            int& orientation_sign)
        {
            const real initial_normal_length = length(initial_normal);
            if (!finite(initial_normal_length) ||
                initial_normal_length <= epsilon_length * epsilon_length)
            {
                return false;
            }

            if (has_cache)
            {
                if (!finite(cached_normal)) return false;
                reference_normal = normalized(to_vector(cached_normal));
                if (length_squared(reference_normal) <= epsilon_length * epsilon_length)
                {
                    return false;
                }
                const real orientation = dot(initial_normal, reference_normal);
                if (!finite(orientation) ||
                    std::abs(orientation) <= epsilon_length * epsilon_length)
                {
                    return false;
                }
                orientation_sign = orientation < 0.0L ? -1 : 1;
            }
            else
            {
                reference_normal = multiply(initial_normal,
                    one / initial_normal_length);
                orientation_sign = 1;
            }

            const real signed_distance = dot(reference_normal, side_vector);
            if (!finite(signed_distance) ||
                std::abs(signed_distance) <= epsilon_length)
            {
                return false;
            }
            side_sign = signed_distance < 0.0L ? -1 : 1;
            return true;
        }

        bool use_segment_side_latch(
            const vector3& initial_normal,
            const vector3& side_vector,
            const asset::vec3& latched_reference_normal,
            int8_t latched_side_sign,
            real epsilon_length,
            vector3& reference_normal,
            int8_t& side_sign,
            int& orientation_sign)
        {
            const real initial_normal_length = length(initial_normal);
            if (!finite(initial_normal_length) ||
                initial_normal_length <= epsilon_length * epsilon_length ||
                (latched_side_sign != -1 && latched_side_sign != 1) ||
                !finite(latched_reference_normal))
            {
                return false;
            }

            reference_normal = normalized(to_vector(latched_reference_normal));
            if (length_squared(reference_normal) <=
                epsilon_length * epsilon_length)
            {
                return false;
            }

            const real alignment = dot(initial_normal, reference_normal);
            if (!finite(alignment) ||
                std::abs(alignment) <= epsilon_length * epsilon_length)
            {
                return false;
            }
            orientation_sign = alignment < 0.0L ? -1 : 1;

            const real signed_distance = dot(reference_normal, side_vector);
            if (!finite(signed_distance) ||
                std::abs(signed_distance) <= epsilon_length ||
                (signed_distance < 0.0L ? -1 : 1) != latched_side_sign)
            {
                return false;
            }

            side_sign = latched_side_sign;
            return true;
        }

        struct candidate_search_result
        {
            bool hit = false;
            bool unresolved = false;
            distance_sample witness;
            real witness_s = 0.0L;
            uint32_t evaluations = 0;
        };

        template<typename SampleFunction, typename LowerBoundFunction>
        candidate_search_result search_correction_candidate(
            real speed_bound,
            real contact_threshold,
            uint32_t maximum_evaluations,
            SampleFunction sample_function,
            LowerBoundFunction lower_bound_function)
        {
            candidate_search_result result;
            std::vector<segment> pending;
            pending.push_back({ 0.0L, 1.0L });

            while (!pending.empty())
            {
                if (result.evaluations >= maximum_evaluations)
                {
                    result.unresolved = true;
                    return result;
                }

                const segment current = pending.back();
                pending.pop_back();
                const real midpoint = (current.lower + current.upper) * 0.5L;
                if (midpoint <= current.lower || midpoint >= current.upper)
                {
                    result.unresolved = true;
                    return result;
                }

                ++result.evaluations;
                const distance_sample sample = sample_function(midpoint);
                const real aabb_lower = lower_bound_function(midpoint);
                if (!sample.valid || !finite(aabb_lower))
                {
                    result.unresolved = true;
                    return result;
                }

                const real half_width = (current.upper - current.lower) * 0.5L;
                const real movement_bound = round_up(speed_bound * half_width);
                const real interval_distance_lower = std::max(
                    0.0L, round_down(aabb_lower - movement_bound));

                if (interval_distance_lower > contact_threshold)
                {
                    continue;
                }

                if (sample.distance_upper <= contact_threshold)
                {
                    result.hit = true;
                    result.witness = sample;
                    result.witness_s = midpoint;
                    return result;
                }

                const segment left = { current.lower, midpoint };
                const segment right = { midpoint, current.upper };
                pending.push_back(right);
                pending.push_back(left);
            }

            return result;
        }

        enum class side_proof_status : uint8_t
        {
            same_side,
            crossing,
            unresolved
        };

        template<typename SideIntervalFunction>
        side_proof_status prove_correction_side(
            uint32_t& evaluations,
            uint32_t maximum_evaluations,
            real minimum_normal_magnitude,
            SideIntervalFunction side_interval_function)
        {
            std::vector<segment> pending;
            pending.push_back({ 0.0L, 1.0L });

            while (!pending.empty())
            {
                if (evaluations >= maximum_evaluations)
                {
                    return side_proof_status::unresolved;
                }

                const segment current = pending.back();
                pending.pop_back();
                ++evaluations;
                const auto bounds = side_interval_function(current);

                if (!bounds.alignment.valid || !bounds.normal_length.valid ||
                    !bounds.phi.valid)
                {
                    return side_proof_status::unresolved;
                }

                if (bounds.alignment.upper <= minimum_normal_magnitude ||
                    bounds.normal_length.upper <= minimum_normal_magnitude)
                {
                    return side_proof_status::unresolved;
                }

                if (bounds.feature_mapping_required)
                {
                    if (!bounds.feature_parameter_a.valid ||
                        !bounds.feature_parameter_b.valid ||
                        bounds.feature_parameter_a.upper <=
                            bounds.feature_parameter_tolerance_a ||
                        bounds.feature_parameter_a.lower >=
                            one - bounds.feature_parameter_tolerance_a ||
                        bounds.feature_parameter_b.upper <=
                            bounds.feature_parameter_tolerance_b ||
                        bounds.feature_parameter_b.lower >=
                            one - bounds.feature_parameter_tolerance_b)
                    {
                        return side_proof_status::unresolved;
                    }
                    if (bounds.feature_parameter_a.lower <=
                            bounds.feature_parameter_tolerance_a ||
                        bounds.feature_parameter_a.upper >=
                            one - bounds.feature_parameter_tolerance_a ||
                        bounds.feature_parameter_b.lower <=
                            bounds.feature_parameter_tolerance_b ||
                        bounds.feature_parameter_b.upper >=
                            one - bounds.feature_parameter_tolerance_b)
                    {
                        const real midpoint =
                            (current.lower + current.upper) * 0.5L;
                        if (midpoint <= current.lower || midpoint >= current.upper)
                        {
                            return side_proof_status::unresolved;
                        }
                        pending.push_back({ current.lower, midpoint });
                        pending.push_back({ midpoint, current.upper });
                        continue;
                    }
                }

                // The signed separation inequalities are meaningful only while
                // the complete interval proves a non-degenerate Normal in the
                // reference Normal's positive hemisphere.
                if (bounds.alignment.lower > minimum_normal_magnitude &&
                    bounds.normal_length.lower > minimum_normal_magnitude)
                {
                    if (bounds.phi.upper < 0.0L)
                    {
                        return side_proof_status::crossing;
                    }
                    if (bounds.phi.lower >= 0.0L)
                    {
                        continue;
                    }
                }

                const real midpoint = (current.lower + current.upper) * 0.5L;
                if (midpoint <= current.lower || midpoint >= current.upper)
                {
                    return side_proof_status::unresolved;
                }
                // Visit the interval away from the Segment-start boundary first.
                // This allows a crossing exactly at an endpoint to be certified
                // from the open-side interval instead of exhausting the budget
                // forever on the subinterval that still contains that boundary.
                pending.push_back({ current.lower, midpoint });
                pending.push_back({ midpoint, current.upper });
            }

            return side_proof_status::same_side;
        }

        struct side_interval_bounds
        {
            interval alignment;
            interval normal_length;
            interval phi;
            interval feature_parameter_a;
            interval feature_parameter_b;
            real feature_parameter_tolerance_a = 0.0L;
            real feature_parameter_tolerance_b = 0.0L;
            bool feature_mapping_required = false;
        };

        side_interval_bounds make_side_interval(
            const interval_vector3& oriented_normal,
            const interval_vector3& separation_vector,
            const vector3& reference_normal,
            int8_t side_sign,
            real distance_tolerance)
        {
            side_interval_bounds result;
            result.alignment = interval_dot(oriented_normal, reference_normal);
            result.normal_length = interval_vector_length(oriented_normal);
            const interval signed_numerator = interval_scale(
                interval_dot(oriented_normal, separation_vector),
                static_cast<real>(side_sign));
            const interval tolerance_term = interval_scale(
                result.normal_length, distance_tolerance);
            result.phi = interval_add(signed_numerator, tolerance_term);
            return result;
        }

        side_interval_bounds make_ee_side_interval(
            const interval_vector3& edge_a0,
            const interval_vector3& edge_a1,
            const interval_vector3& edge_b0,
            const interval_vector3& edge_b1,
            const vector3& reference_normal,
            int8_t side_sign,
            int orientation_sign,
            real distance_tolerance,
            real epsilon_length)
        {
            side_interval_bounds result;
            result.feature_mapping_required = true;

            const interval_vector3 direction_a =
                interval_difference(edge_a1, edge_a0);
            const interval_vector3 direction_b =
                interval_difference(edge_b1, edge_b0);
            const interval_vector3 relative =
                interval_difference(edge_a0, edge_b0);

            const interval length_a_squared =
                interval_dot(direction_a, direction_a);
            const interval length_b_squared =
                interval_dot(direction_b, direction_b);
            const interval direction_dot =
                interval_dot(direction_a, direction_b);
            const interval relative_a =
                interval_dot(direction_a, relative);
            const interval relative_b =
                interval_dot(direction_b, relative);

            const interval denominator = interval_subtract(
                interval_multiply(length_a_squared, length_b_squared),
                interval_square(direction_dot));
            const real minimum_denominator =
                epsilon_length * epsilon_length *
                epsilon_length * epsilon_length;
            if (!denominator.valid ||
                denominator.lower <= minimum_denominator)
            {
                result.feature_parameter_a = { 0.0L, 0.0L, false };
                result.feature_parameter_b = { 0.0L, 0.0L, false };
                return result;
            }

            const interval numerator_a = interval_subtract(
                interval_multiply(direction_dot, relative_b),
                interval_multiply(relative_a, length_b_squared));
            const interval numerator_b = interval_subtract(
                interval_multiply(length_a_squared, relative_b),
                interval_multiply(direction_dot, relative_a));
            result.feature_parameter_a =
                interval_divide_positive(numerator_a, denominator);
            result.feature_parameter_b =
                interval_divide_positive(numerator_b, denominator);

            const interval edge_a_length = interval_sqrt(length_a_squared);
            const interval edge_b_length = interval_sqrt(length_b_squared);
            if (!edge_a_length.valid || !edge_b_length.valid ||
                edge_a_length.lower <= epsilon_length ||
                edge_b_length.lower <= epsilon_length)
            {
                result.feature_parameter_a.valid = false;
                result.feature_parameter_b.valid = false;
                return result;
            }
            result.feature_parameter_tolerance_a =
                epsilon_length / edge_a_length.lower;
            result.feature_parameter_tolerance_b =
                epsilon_length / edge_b_length.lower;
            if (result.feature_parameter_tolerance_a >= 0.5L ||
                result.feature_parameter_tolerance_b >= 0.5L)
            {
                result.feature_parameter_a.valid = false;
                result.feature_parameter_b.valid = false;
                return result;
            }

            const interval_vector3 normal = interval_scale_vector(
                interval_cross(direction_a, direction_b),
                static_cast<real>(orientation_sign));
            result.alignment = interval_dot(normal, reference_normal);
            result.normal_length = interval_vector_length(normal);
            const interval signed_numerator = interval_scale(
                interval_dot(normal, relative), static_cast<real>(side_sign));
            const interval tolerance_term = interval_scale(
                result.normal_length, distance_tolerance);
            result.phi = interval_add(signed_numerator, tolerance_term);
            return result;
        }

        correction_ccd_result make_unresolved(
            correction_ccd_result result,
            const std::string& diagnostic)
        {
            result.status = correction_ccd_status::unresolved;
            result.diagnostic = diagnostic;
            return result;
        }

        bool is_interior_edge_parameter(
            real parameter,
            real edge_length,
            real epsilon_length)
        {
            if (!finite(parameter) || !finite(edge_length) ||
                edge_length <= epsilon_length)
            {
                return false;
            }
            const real parameter_tolerance = epsilon_length / edge_length;
            return parameter_tolerance < 0.5L &&
                parameter > parameter_tolerance &&
                parameter < one - parameter_tolerance;
        }

        bool prepare_vt_reference(
            const vertex_triangle_correction_motion& motion,
            const correction_ccd_settings& settings,
            vector3& reference_normal,
            int8_t& side_sign,
            int& orientation_sign)
        {
            const vector3 point = to_vector(motion.segment_start_point);
            const vector3 a = to_vector(motion.segment_start_triangle[0]);
            const vector3 b = to_vector(motion.segment_start_triangle[1]);
            const vector3 c = to_vector(motion.segment_start_triangle[2]);
            const vector3 initial_normal = cross(subtract(b, a), subtract(c, a));
            if (motion.has_segment_side_latch)
            {
                return use_segment_side_latch(
                    initial_normal, subtract(point, a),
                    motion.segment_reference_normal,
                    motion.segment_allowed_side_sign,
                    settings.epsilon_length,
                    reference_normal, side_sign, orientation_sign);
            }
            return latch_reference_side(
                initial_normal, subtract(point, a),
                motion.has_valid_persistent_contact_cache,
                motion.persistent_contact_normal,
                settings.epsilon_length,
                reference_normal, side_sign, orientation_sign);
        }

        bool prepare_ee_reference(
            const edge_edge_correction_motion& motion,
            const correction_ccd_settings& settings,
            vector3& reference_normal,
            int8_t& side_sign,
            int& orientation_sign)
        {
            const vector3 a0 = to_vector(motion.segment_start_edge_a[0]);
            const vector3 a1 = to_vector(motion.segment_start_edge_a[1]);
            const vector3 b0 = to_vector(motion.segment_start_edge_b[0]);
            const vector3 b1 = to_vector(motion.segment_start_edge_b[1]);
            const vector3 direction_a = subtract(a1, a0);
            const vector3 direction_b = subtract(b1, b0);

            vector3 closest_a;
            vector3 closest_b;
            real parameter_a = 0.0L;
            real parameter_b = 0.0L;
            if (!closest_points_segments(
                a0, a1, b0, b1, settings.epsilon_length,
                closest_a, closest_b, parameter_a, parameter_b) ||
                !is_interior_edge_parameter(
                    parameter_a, length(direction_a), settings.epsilon_length) ||
                !is_interior_edge_parameter(
                    parameter_b, length(direction_b), settings.epsilon_length))
            {
                // Endpoint-clamped EE features require a separately certified
                // persistent/witness normal. The current CPU reference only
                // implements the Interior-Interior feature contract.
                return false;
            }

            const vector3 initial_normal = cross(direction_a, direction_b);
            if (motion.has_segment_side_latch)
            {
                return use_segment_side_latch(
                    initial_normal, subtract(closest_a, closest_b),
                    motion.segment_reference_normal,
                    motion.segment_allowed_side_sign,
                    settings.epsilon_length,
                    reference_normal, side_sign, orientation_sign);
            }
            return latch_reference_side(
                initial_normal, subtract(closest_a, closest_b),
                motion.has_valid_persistent_contact_cache,
                motion.persistent_contact_normal,
                settings.epsilon_length,
                reference_normal, side_sign, orientation_sign);
        }

        std::array<vector3, 3> evaluate_triangle(
            const std::array<correction_vertex_motion, 3>& motion,
            real s)
        {
            return {
                lerp(to_vector(motion[0].before), to_vector(motion[0].after), s),
                lerp(to_vector(motion[1].before), to_vector(motion[1].after), s),
                lerp(to_vector(motion[2].before), to_vector(motion[2].after), s)
            };
        }

        std::array<vector3, 2> evaluate_edge(
            const std::array<correction_vertex_motion, 2>& motion,
            real s)
        {
            return {
                lerp(to_vector(motion[0].before), to_vector(motion[0].after), s),
                lerp(to_vector(motion[1].before), to_vector(motion[1].after), s)
            };
        }

        interval_vector3 interval_motion(
            const correction_vertex_motion& motion,
            const segment& range)
        {
            return interval_lerp(
                to_vector(motion.before), to_vector(motion.after), range);
        }

        interval_vector3 interval_difference(
            const interval_vector3& a,
            const interval_vector3& b)
        {
            return interval_subtract(a, b);
        }

        interval_vector3 interval_scale_vector(
            const interval_vector3& value,
            real scalar)
        {
            return {
                interval_scale(value.x, scalar),
                interval_scale(value.y, scalar),
                interval_scale(value.z, scalar)
            };
        }
    }

    bool correction_candidate_pair_key::operator==(
        const correction_candidate_pair_key& other) const
    {
        return contact_type == other.contact_type &&
            stable_id_a == other.stable_id_a &&
            mesh_a == other.mesh_a &&
            primitive_a == other.primitive_a &&
            stable_id_b == other.stable_id_b &&
            mesh_b == other.mesh_b &&
            primitive_b == other.primitive_b;
    }

    bool correction_candidate_pair_key::operator<(
        const correction_candidate_pair_key& other) const
    {
        return std::tie(stable_id_a, mesh_a, primitive_a,
            stable_id_b, mesh_b, primitive_b, contact_type) <
            std::tie(other.stable_id_a, other.mesh_a, other.primitive_a,
                other.stable_id_b, other.mesh_b, other.primitive_b,
                other.contact_type);
    }

    bool correction_primitive_identity::operator==(
        const correction_primitive_identity& other) const
    {
        return stable_id == other.stable_id &&
            mesh_id == other.mesh_id &&
            primitive_id == other.primitive_id;
    }

    bool correction_primitive_identity::operator<(
        const correction_primitive_identity& other) const
    {
        return std::tie(stable_id, mesh_id, primitive_id) <
            std::tie(other.stable_id, other.mesh_id, other.primitive_id);
    }

    correction_candidate_pair_key make_canonical_correction_candidate_key(
        correction_contact_type contact_type,
        uint64_t stable_id_a,
        uint32_t mesh_a,
        uint32_t primitive_a,
        uint64_t stable_id_b,
        uint32_t mesh_b,
        uint32_t primitive_b)
    {
        const std::tuple<uint64_t, uint32_t, uint32_t> endpoint_a = {
            stable_id_a, mesh_a, primitive_a
        };
        const std::tuple<uint64_t, uint32_t, uint32_t> endpoint_b = {
            stable_id_b, mesh_b, primitive_b
        };
        if (endpoint_b < endpoint_a)
        {
            std::swap(stable_id_a, stable_id_b);
            std::swap(mesh_a, mesh_b);
            std::swap(primitive_a, primitive_b);
        }

        correction_candidate_pair_key result;
        result.contact_type = contact_type;
        result.stable_id_a = stable_id_a;
        result.mesh_a = mesh_a;
        result.primitive_a = primitive_a;
        result.stable_id_b = stable_id_b;
        result.mesh_b = mesh_b;
        result.primitive_b = primitive_b;
        return result;
    }

    correction_candidate_union_result merge_correction_candidate_union(
        const std::vector<correction_candidate_pair_key>& existing_candidates,
        const std::vector<correction_candidate_pair_key>& incoming_candidates,
        std::size_t capacity)
    {
        correction_candidate_union_result result;
        if (existing_candidates.size() >
            std::numeric_limits<std::size_t>::max() - incoming_candidates.size())
        {
            result.overflow = true;
            return result;
        }

        std::vector<correction_candidate_pair_key> merged;
        merged.reserve(existing_candidates.size() + incoming_candidates.size());
        for (const correction_candidate_pair_key& key : existing_candidates)
        {
            merged.push_back(make_canonical_correction_candidate_key(
                key.contact_type, key.stable_id_a, key.mesh_a, key.primitive_a,
                key.stable_id_b, key.mesh_b, key.primitive_b));
        }
        for (const correction_candidate_pair_key& key : incoming_candidates)
        {
            merged.push_back(make_canonical_correction_candidate_key(
                key.contact_type, key.stable_id_a, key.mesh_a, key.primitive_a,
                key.stable_id_b, key.mesh_b, key.primitive_b));
        }

        std::sort(merged.begin(), merged.end());
        merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
        result.required_count = merged.size();
        if (result.required_count > capacity)
        {
            result.overflow = true;
            return result;
        }

        result.candidates = std::move(merged);
        result.success = true;
        return result;
    }

    correction_segment_candidate_state_result create_correction_segment_candidate_state(
        uint64_t base_integration_segment_id,
        const std::vector<correction_candidate_pair_key>& physical_swept_candidates,
        std::size_t capacity)
    {
        correction_segment_candidate_state_result result;
        if (base_integration_segment_id == 0)
        {
            result.diagnostic =
                "Correction Candidate State requires a non-zero base_integration_segment_id.";
            return result;
        }

        const correction_candidate_union_result merged =
            merge_correction_candidate_union(
                {}, physical_swept_candidates, capacity);
        result.required_count = merged.required_count;
        result.capacity_overflow = merged.overflow;
        if (!merged.success)
        {
            result.diagnostic = merged.overflow
                ? "Physical Swept Candidate Set exceeds the Segment Candidate Capacity."
                : "Physical Swept Candidate Set could not be canonicalized.";
            return result;
        }

        result.state.base_integration_segment_id = base_integration_segment_id;
        result.state.candidates = merged.candidates;
        result.state.candidate_generation = 1u;
        result.state.contact_set_candidate_generation = 0u;
        result.state.vertex_contact_csr_candidate_generation = 0u;
        result.state.contact_set_current = false;
        result.state.vertex_contact_csr_current = false;
        result.success = true;
        return result;
    }

    correction_segment_candidate_update_result merge_correction_position_writer_epoch_candidates(
        correction_segment_candidate_state& state,
        uint64_t base_integration_segment_id,
        uint32_t position_writer_epoch_id,
        const std::vector<correction_candidate_pair_key>& epoch_candidates,
        std::size_t capacity)
    {
        correction_segment_candidate_update_result result;
        if (base_integration_segment_id == 0 ||
            state.base_integration_segment_id != base_integration_segment_id)
        {
            result.wrong_segment = true;
            result.diagnostic =
                "Position-Writer Epoch Candidate Set belongs to a different Integration Segment.";
            return result;
        }
        if (position_writer_epoch_id == 0 ||
            position_writer_epoch_id != state.last_position_writer_epoch_id)
        {
            result.invalid_epoch_order = true;
            result.diagnostic =
                "Candidate append result does not belong to the currently active Position-Writer Epoch.";
            return result;
        }

        const correction_candidate_union_result merged =
            merge_correction_candidate_union(
                state.candidates, epoch_candidates, capacity);
        result.required_count = merged.required_count;
        result.capacity_overflow = merged.overflow;
        if (!merged.success)
        {
            result.diagnostic = merged.overflow
                ? "Position-Writer Epoch Candidate Union exceeds the Segment Capacity."
                : "Position-Writer Epoch Candidate Union could not be canonicalized.";
            return result;
        }

        if (merged.candidates == state.candidates)
        {
            result.success = true;
            return result;
        }

        if (state.candidate_generation ==
            std::numeric_limits<uint64_t>::max())
        {
            result.generation_overflow = true;
            result.diagnostic =
                "Correction Candidate Generation overflow; Segment State was not mutated.";
            return result;
        }

        state.candidates = merged.candidates;
        ++state.candidate_generation;
        state.contact_set_current = false;
        state.vertex_contact_csr_current = false;
        result.changed = true;
        result.success = true;
        return result;
    }

    correction_segment_candidate_update_result begin_correction_position_writer_epoch(
        correction_segment_candidate_state& state,
        uint64_t base_integration_segment_id,
        uint32_t position_writer_epoch_id)
    {
        correction_segment_candidate_update_result result;
        if (base_integration_segment_id == 0 ||
            state.base_integration_segment_id != base_integration_segment_id)
        {
            result.wrong_segment = true;
            result.diagnostic =
                "Position-Writer Epoch belongs to a different Integration Segment.";
            return result;
        }
        if (position_writer_epoch_id == 0 ||
            position_writer_epoch_id <= state.last_position_writer_epoch_id)
        {
            result.invalid_epoch_order = true;
            result.diagnostic =
                "Position-Writer Epoch ID must be non-zero and strictly increasing within the Integration Segment.";
            return result;
        }
        if (state.position_writer_epoch_generation ==
            std::numeric_limits<uint64_t>::max())
        {
            result.epoch_generation_overflow = true;
            result.diagnostic =
                "Position-Writer Epoch Generation overflow; Segment State was not mutated.";
            return result;
        }

        state.last_position_writer_epoch_id = position_writer_epoch_id;
        ++state.position_writer_epoch_generation;
        state.contact_set_current = false;
        state.vertex_contact_csr_current = false;
        result.changed = true;
        result.success = true;
        return result;
    }

    correction_contact_set_refresh_result refresh_correction_segment_contact_set_and_csr(
        correction_segment_candidate_state& state,
        uint64_t candidate_generation,
        uint64_t position_writer_epoch_generation,
        const std::vector<correction_discrete_contact_input>& discrete_inputs,
        float epsilon_length)
    {
        correction_contact_set_refresh_result result;
        result.candidate_count = state.candidates.size();
        result.candidate_generation = candidate_generation;
        result.position_writer_epoch_generation = position_writer_epoch_generation;

        if (state.base_integration_segment_id == 0 ||
            candidate_generation != state.candidate_generation ||
            position_writer_epoch_generation !=
                state.position_writer_epoch_generation)
        {
            result.generation_mismatch = true;
            result.diagnostic =
                "Contact Set refresh does not match the current Segment Candidate and Position-Writer generations.";
            return result;
        }
        if (!std::isfinite(epsilon_length) || epsilon_length <= 0.0f ||
            discrete_inputs.size() != state.candidates.size())
        {
            result.invalid_input = true;
            result.diagnostic =
                "Contact Set refresh requires a positive finite epsilon and exactly one discrete geometry record per Candidate Pair.";
            return result;
        }

        std::vector<correction_discrete_contact_input> sorted_inputs =
            discrete_inputs;
        for (correction_discrete_contact_input& input : sorted_inputs)
        {
            input.key = make_canonical_correction_candidate_key(
                input.key.contact_type,
                input.key.stable_id_a, input.key.mesh_a, input.key.primitive_a,
                input.key.stable_id_b, input.key.mesh_b, input.key.primitive_b);
            if (!std::isfinite(input.contact_thickness) ||
                input.contact_thickness < 0.0f)
            {
                result.invalid_input = true;
                result.diagnostic =
                    "Contact Set input contains an invalid Contact Thickness.";
                return result;
            }

            std::array<uint64_t, 4> unique_vertex_ids = input.vertex_stable_ids;
            std::sort(unique_vertex_ids.begin(), unique_vertex_ids.end());
            if (std::adjacent_find(
                    unique_vertex_ids.begin(), unique_vertex_ids.end()) !=
                unique_vertex_ids.end())
            {
                result.invalid_input = true;
                result.diagnostic =
                    "A discrete Contact Candidate contains duplicate participant Vertex Stable IDs.";
                return result;
            }
            for (const asset::vec3& position : input.positions)
            {
                if (!finite(position))
                {
                    result.invalid_input = true;
                    result.diagnostic =
                        "Contact Set input contains a non-finite participant position.";
                    return result;
                }
            }
        }

        std::sort(sorted_inputs.begin(), sorted_inputs.end(),
            [](const correction_discrete_contact_input& left,
                const correction_discrete_contact_input& right)
            {
                return left.key < right.key;
            });
        for (std::size_t index = 1; index < sorted_inputs.size(); ++index)
        {
            if (sorted_inputs[index - 1].key == sorted_inputs[index].key)
            {
                result.invalid_input = true;
                result.diagnostic =
                    "Contact Set input contains duplicate Canonical Pair Keys.";
                return result;
            }
        }

        if (sorted_inputs.size() != state.candidates.size())
        {
            result.invalid_input = true;
            result.diagnostic =
                "Contact Set input does not cover the complete Segment Candidate Union.";
            return result;
        }
        for (std::size_t index = 0; index < state.candidates.size(); ++index)
        {
            if (!(sorted_inputs[index].key == state.candidates[index]))
            {
                result.invalid_input = true;
                result.diagnostic =
                    "Contact Set input is missing a Candidate Pair or contains a Pair outside the Segment Candidate Union.";
                return result;
            }
        }

        std::vector<correction_discrete_contact> next_contact_set;
        next_contact_set.reserve(sorted_inputs.size());
        std::vector<std::pair<uint64_t, uint32_t>> vertex_contact_entries;
        for (const correction_discrete_contact_input& input : sorted_inputs)
        {
            correction_discrete_contact contact;
            contact.key = input.key;

            if (input.key.contact_type ==
                correction_contact_type::vertex_triangle)
            {
                const std::array<asset::vec3, 3> triangle = {
                    input.positions[1], input.positions[2], input.positions[3]
                };
                contact.vertex_triangle = find_vertex_triangle_contact(
                    input.positions[0], triangle, input.contact_thickness);
                if (!contact.vertex_triangle.valid)
                {
                    result.invalid_input = true;
                    result.diagnostic =
                        "Final Discrete VT Narrowphase could not certify the Candidate geometry.";
                    return result;
                }
                contact.active = contact.vertex_triangle.hit;
            }
            else if (input.key.contact_type ==
                correction_contact_type::edge_edge)
            {
                const std::array<asset::vec3, 2> edge_a = {
                    input.positions[0], input.positions[1]
                };
                const std::array<asset::vec3, 2> edge_b = {
                    input.positions[2], input.positions[3]
                };
                contact.edge_edge = find_edge_edge_contact(
                    edge_a, edge_b, input.contact_thickness);
                if (!contact.edge_edge.valid)
                {
                    result.invalid_input = true;
                    result.diagnostic =
                        "Final Discrete EE Narrowphase could not certify the Candidate geometry.";
                    return result;
                }

                if (contact.edge_edge.distance <= epsilon_length)
                {
                    const vector3 direction_a = subtract(
                        to_vector(edge_a[1]), to_vector(edge_a[0]));
                    const vector3 direction_b = subtract(
                        to_vector(edge_b[1]), to_vector(edge_b[0]));
                    const vector3 edge_normal = cross(
                        direction_a, direction_b);
                    const real normal_length_squared =
                        length_squared(edge_normal);
                    const real minimum_area =
                        static_cast<real>(epsilon_length) *
                        static_cast<real>(epsilon_length);
                    if (!finite(normal_length_squared) ||
                        normal_length_squared <=
                            minimum_area * minimum_area)
                    {
                        result.invalid_input = true;
                        result.diagnostic =
                            "Final Discrete EE contact has zero separation and no stable non-parallel Edge Normal.";
                        return result;
                    }
                }
                contact.active = contact.edge_edge.hit;
            }
            else
            {
                result.invalid_input = true;
                result.diagnostic =
                    "Contact Set input contains an unknown Contact Type.";
                return result;
            }

            if (!contact.active)
            {
                continue;
            }
            if (next_contact_set.size() >=
                static_cast<std::size_t>(
                    std::numeric_limits<uint32_t>::max()))
            {
                result.invalid_input = true;
                result.diagnostic =
                    "Active Contact Set exceeds the Vertex-to-Contact CSR index range.";
                return result;
            }

            const uint32_t contact_index =
                static_cast<uint32_t>(next_contact_set.size());
            for (uint64_t vertex_id : input.vertex_stable_ids)
            {
                vertex_contact_entries.emplace_back(vertex_id, contact_index);
            }
            next_contact_set.push_back(std::move(contact));
        }

        std::sort(vertex_contact_entries.begin(), vertex_contact_entries.end());
        vertex_contact_entries.erase(
            std::unique(vertex_contact_entries.begin(),
                vertex_contact_entries.end()),
            vertex_contact_entries.end());
        if (vertex_contact_entries.size() >
            static_cast<std::size_t>(
                std::numeric_limits<uint32_t>::max()))
        {
            result.invalid_input = true;
            result.diagnostic =
                "Vertex-to-Contact CSR exceeds the 32-bit contact-index range.";
            return result;
        }

        correction_vertex_contact_csr next_csr;
        next_csr.candidate_generation = candidate_generation;
        next_csr.position_writer_epoch_generation =
            position_writer_epoch_generation;
        next_csr.offsets.push_back(0u);
        std::size_t entry_index = 0;
        while (entry_index < vertex_contact_entries.size())
        {
            const uint64_t vertex_id =
                vertex_contact_entries[entry_index].first;
            next_csr.vertex_stable_ids.push_back(vertex_id);
            while (entry_index < vertex_contact_entries.size() &&
                vertex_contact_entries[entry_index].first == vertex_id)
            {
                next_csr.contact_indices.push_back(
                    vertex_contact_entries[entry_index].second);
                ++entry_index;
            }
            next_csr.offsets.push_back(
                static_cast<uint32_t>(next_csr.contact_indices.size()));
        }
        next_csr.current = true;

        state.contact_set = std::move(next_contact_set);
        state.vertex_contact_csr = std::move(next_csr);
        state.contact_set_candidate_generation = candidate_generation;
        state.contact_set_position_writer_epoch_generation =
            position_writer_epoch_generation;
        state.vertex_contact_csr_candidate_generation = candidate_generation;
        state.vertex_contact_csr_position_writer_epoch_generation =
            position_writer_epoch_generation;
        state.contact_set_current = true;
        state.vertex_contact_csr_current = true;

        result.active_contact_count = state.contact_set.size();
        result.success = true;
        return result;
    }

    bool union_correction_envelope_aabb(
        const collision_aabb& first,
        const collision_aabb& second,
        collision_aabb& output_bounds)
    {
        output_bounds = {};
        const bool first_valid =
            finite(first.minimum) && finite(first.maximum) &&
            first.minimum.x <= first.maximum.x &&
            first.minimum.y <= first.maximum.y &&
            first.minimum.z <= first.maximum.z;
        const bool second_valid =
            finite(second.minimum) && finite(second.maximum) &&
            second.minimum.x <= second.maximum.x &&
            second.minimum.y <= second.maximum.y &&
            second.minimum.z <= second.maximum.z;
        if (!first_valid || !second_valid)
        {
            return false;
        }

        const float negative_infinity =
            -std::numeric_limits<float>::infinity();
        const float positive_infinity =
            std::numeric_limits<float>::infinity();
        const collision_aabb bounds = {
            {
                std::nextafter(std::min(first.minimum.x, second.minimum.x),
                    negative_infinity),
                std::nextafter(std::min(first.minimum.y, second.minimum.y),
                    negative_infinity),
                std::nextafter(std::min(first.minimum.z, second.minimum.z),
                    negative_infinity)
            },
            {
                std::nextafter(std::max(first.maximum.x, second.maximum.x),
                    positive_infinity),
                std::nextafter(std::max(first.maximum.y, second.maximum.y),
                    positive_infinity),
                std::nextafter(std::max(first.maximum.z, second.maximum.z),
                    positive_infinity)
            }
        };
        if (!finite(bounds.minimum) || !finite(bounds.maximum))
        {
            return false;
        }
        output_bounds = bounds;
        return true;
    }

    bool build_correction_swept_aabb(
        const std::vector<correction_vertex_motion>& vertex_motions,
        float contact_thickness,
        collision_aabb& output_bounds)
    {
        output_bounds = {};
        if (vertex_motions.empty() ||
            !std::isfinite(contact_thickness) ||
            contact_thickness < 0.0f)
        {
            return false;
        }

        const asset::vec3& first_before = vertex_motions[0].before;
        const asset::vec3& first_after = vertex_motions[0].after;
        if (!finite(first_before) || !finite(first_after))
        {
            return false;
        }

        float minimum_x = std::min(first_before.x, first_after.x);
        float minimum_y = std::min(first_before.y, first_after.y);
        float minimum_z = std::min(first_before.z, first_after.z);
        float maximum_x = std::max(first_before.x, first_after.x);
        float maximum_y = std::max(first_before.y, first_after.y);
        float maximum_z = std::max(first_before.z, first_after.z);

        for (std::size_t index = 1; index < vertex_motions.size(); ++index)
        {
            const correction_vertex_motion& motion = vertex_motions[index];
            if (!finite(motion.before) || !finite(motion.after))
            {
                return false;
            }

            minimum_x = std::min(minimum_x,
                std::min(motion.before.x, motion.after.x));
            minimum_y = std::min(minimum_y,
                std::min(motion.before.y, motion.after.y));
            minimum_z = std::min(minimum_z,
                std::min(motion.before.z, motion.after.z));
            maximum_x = std::max(maximum_x,
                std::max(motion.before.x, motion.after.x));
            maximum_y = std::max(maximum_y,
                std::max(motion.before.y, motion.after.y));
            maximum_z = std::max(maximum_z,
                std::max(motion.before.z, motion.after.z));
        }

        const float expansion = contact_thickness * 0.5f;
        if (!std::isfinite(expansion))
        {
            return false;
        }

        const float negative_infinity =
            -std::numeric_limits<float>::infinity();
        const float positive_infinity =
            std::numeric_limits<float>::infinity();
        const collision_aabb bounds = {
            {
                std::nextafter(minimum_x - expansion, negative_infinity),
                std::nextafter(minimum_y - expansion, negative_infinity),
                std::nextafter(minimum_z - expansion, negative_infinity)
            },
            {
                std::nextafter(maximum_x + expansion, positive_infinity),
                std::nextafter(maximum_y + expansion, positive_infinity),
                std::nextafter(maximum_z + expansion, positive_infinity)
            }
        };

        if (!finite(bounds.minimum) || !finite(bounds.maximum) ||
            bounds.minimum.x > bounds.maximum.x ||
            bounds.minimum.y > bounds.maximum.y ||
            bounds.minimum.z > bounds.maximum.z)
        {
            return false;
        }

        output_bounds = bounds;
        return true;
    }

    correction_broadphase_result generate_correction_broadphase_candidates(
        const std::vector<correction_broadphase_primitive>& primitives,
        const std::vector<broadphase_mesh_pair>& mesh_pairs,
        const broadphase_settings& settings)
    {
        correction_broadphase_result result;
        if (settings.contact_thickness != 0.0f)
        {
            result.diagnostics.push_back({
                collision_error::invalid_settings,
                asset::invalid_index,
                "Correction Broadphase requires pre-expanded per-primitive AABBs and contact_thickness == 0 to avoid double expansion."
            });
            return result;
        }

        std::vector<broadphase_primitive> broadphase_primitives;
        std::vector<correction_primitive_role> roles;
        broadphase_primitives.reserve(primitives.size());
        roles.reserve(primitives.size());

        for (std::size_t index = 0; index < primitives.size(); ++index)
        {
            const correction_broadphase_primitive& primitive = primitives[index];
            std::size_t expected_vertex_count = 0;
            switch (primitive.role)
            {
            case correction_primitive_role::vertex:
                expected_vertex_count = 1;
                break;
            case correction_primitive_role::triangle:
                expected_vertex_count = 3;
                break;
            case correction_primitive_role::edge:
                expected_vertex_count = 2;
                break;
            default:
                result.diagnostics.push_back({
                    collision_error::invalid_geometry,
                    static_cast<uint32_t>(index),
                    "Correction Broadphase primitive role is invalid."
                });
                return result;
            }

            if (primitive.vertex_stable_ids.size() != expected_vertex_count)
            {
                result.diagnostics.push_back({
                    collision_error::invalid_geometry,
                    static_cast<uint32_t>(index),
                    "Correction Broadphase primitive must provide one unique Stable Vertex ID per endpoint."
                });
                return result;
            }
            std::vector<uint64_t> vertex_ids = primitive.vertex_stable_ids;
            std::sort(vertex_ids.begin(), vertex_ids.end());
            if (std::adjacent_find(vertex_ids.begin(), vertex_ids.end()) !=
                vertex_ids.end())
            {
                result.diagnostics.push_back({
                    collision_error::invalid_geometry,
                    static_cast<uint32_t>(index),
                    "Correction Broadphase primitive contains duplicate Stable Vertex IDs."
                });
                return result;
            }

            std::vector<correction_primitive_identity> exclusions =
                primitive.excluded_primitive_ids;
            std::sort(exclusions.begin(), exclusions.end());
            if (std::adjacent_find(exclusions.begin(), exclusions.end()) !=
                exclusions.end())
            {
                result.diagnostics.push_back({
                    collision_error::invalid_geometry,
                    static_cast<uint32_t>(index),
                    "Correction Broadphase primitive contains duplicate topology exclusions."
                });
                return result;
            }

            broadphase_primitive broadphase_primitive;
            broadphase_primitive.mesh_id = primitive.mesh_id;
            broadphase_primitive.bounds = primitive.envelope_bounds;
            broadphase_primitives.push_back(broadphase_primitive);
            roles.push_back(primitive.role);
        }

        const broadphase_result broadphase = generate_broadphase_candidates(
            broadphase_primitives, mesh_pairs, settings);
        result.diagnostics = broadphase.diagnostics;
        result.required_cell_records = broadphase.required_cell_records;
        result.broadphase_candidate_count = broadphase.required_candidates;
        result.overflow = broadphase.overflow;
        if (!broadphase.success)
        {
            return result;
        }

        std::vector<correction_candidate_pair_key> typed_candidates;
        typed_candidates.reserve(broadphase.candidates.size());
        for (const collision_candidate& candidate : broadphase.candidates)
        {
            if (candidate.primitive_a >= primitives.size() ||
                candidate.primitive_b >= primitives.size())
            {
                result.diagnostics.push_back({
                    collision_error::invalid_geometry,
                    asset::invalid_index,
                    "Broadphase returned a primitive index outside the correction input."
                });
                return result;
            }

            const correction_broadphase_primitive& primitive_a =
                primitives[candidate.primitive_a];
            const correction_broadphase_primitive& primitive_b =
                primitives[candidate.primitive_b];
            const correction_primitive_role role_a =
                roles[candidate.primitive_a];
            const correction_primitive_role role_b =
                roles[candidate.primitive_b];

            const std::vector<uint64_t>& vertex_ids_a =
                primitive_a.vertex_stable_ids;
            const std::vector<uint64_t>& vertex_ids_b =
                primitive_b.vertex_stable_ids;
            bool shares_topological_vertex = false;
            for (uint64_t vertex_id_a : vertex_ids_a)
            {
                if (std::find(vertex_ids_b.begin(), vertex_ids_b.end(),
                        vertex_id_a) != vertex_ids_b.end())
                {
                    shares_topological_vertex = true;
                    break;
                }
            }
            if (shares_topological_vertex)
            {
                continue;
            }

            const correction_primitive_identity identity_a = {
                primitive_a.stable_id, primitive_a.mesh_id,
                primitive_a.primitive_id
            };
            const correction_primitive_identity identity_b = {
                primitive_b.stable_id, primitive_b.mesh_id,
                primitive_b.primitive_id
            };
            const bool explicitly_excluded =
                std::find(
                    primitive_a.excluded_primitive_ids.begin(),
                    primitive_a.excluded_primitive_ids.end(),
                    identity_b) != primitive_a.excluded_primitive_ids.end() ||
                std::find(
                    primitive_b.excluded_primitive_ids.begin(),
                    primitive_b.excluded_primitive_ids.end(),
                    identity_a) != primitive_b.excluded_primitive_ids.end();
            if (explicitly_excluded)
            {
                continue;
            }

            correction_contact_type contact_type;
            if ((role_a == correction_primitive_role::vertex &&
                    role_b == correction_primitive_role::triangle) ||
                (role_a == correction_primitive_role::triangle &&
                    role_b == correction_primitive_role::vertex))
            {
                contact_type = correction_contact_type::vertex_triangle;
            }
            else if (role_a == correction_primitive_role::edge &&
                role_b == correction_primitive_role::edge)
            {
                contact_type = correction_contact_type::edge_edge;
            }
            else
            {
                continue;
            }

            typed_candidates.push_back(make_canonical_correction_candidate_key(
                contact_type,
                primitive_a.stable_id, primitive_a.mesh_id, primitive_a.primitive_id,
                primitive_b.stable_id, primitive_b.mesh_id, primitive_b.primitive_id));
        }

        const correction_candidate_union_result unique_candidates =
            merge_correction_candidate_union({}, typed_candidates,
                settings.candidate_capacity);
        result.required_candidates = unique_candidates.required_count;
        result.overflow = result.overflow || unique_candidates.overflow;
        if (!unique_candidates.success)
        {
            if (unique_candidates.overflow)
            {
                result.diagnostics.push_back({
                    collision_error::candidate_capacity_overflow,
                    asset::invalid_index,
                    "Correction Candidate Union capacity exceeded; no partial Candidate Set was returned."
                });
            }
            return result;
        }

        result.candidates = unique_candidates.candidates;
        result.success = true;
        return result;
    }

    correction_writer_epoch_result process_correction_position_writer_epoch(
        correction_segment_candidate_state& state,
        uint64_t base_integration_segment_id,
        uint32_t position_writer_epoch_id,
        const std::vector<correction_primitive_epoch_input>& primitives,
        const std::vector<broadphase_mesh_pair>& mesh_pairs,
        const broadphase_settings& broadphase_settings,
        const correction_ccd_settings& ccd_settings)
    {
        correction_writer_epoch_result result;
        result.candidate_generation = state.candidate_generation;
        result.position_writer_epoch_generation =
            state.position_writer_epoch_generation;

        if (base_integration_segment_id == 0 ||
            state.base_integration_segment_id != base_integration_segment_id)
        {
            result.status = correction_writer_epoch_status::rejected;
            result.diagnostic =
                "Position-Writer Epoch belongs to a different Integration Segment.";
            return result;
        }
        if (position_writer_epoch_id == 0 ||
            position_writer_epoch_id <= state.last_position_writer_epoch_id)
        {
            result.status = correction_writer_epoch_status::rejected;
            result.diagnostic =
                "Position-Writer Epoch ID must be non-zero and strictly increasing.";
            return result;
        }
        if (!valid_settings(ccd_settings))
        {
            result.status = correction_writer_epoch_status::rejected;
            result.diagnostic =
                "Position-Writer Epoch has invalid Correction CCD settings.";
            return result;
        }
        if (broadphase_settings.contact_thickness != 0.0f)
        {
            result.status = correction_writer_epoch_status::rejected;
            result.diagnostic =
                "Position-Writer Epoch Broadphase requires pre-expanded AABBs and zero secondary Contact Thickness expansion.";
            return result;
        }

        using primitive_key = std::tuple<uint64_t, uint32_t, uint32_t>;
        std::map<primitive_key, std::size_t> primitive_lookup;
        std::vector<correction_broadphase_primitive> correction_primitives;
        std::vector<correction_broadphase_primitive> final_discrete_primitives;
        correction_primitives.reserve(primitives.size());
        final_discrete_primitives.reserve(primitives.size());

        for (std::size_t index = 0; index < primitives.size(); ++index)
        {
            const correction_primitive_epoch_input& primitive = primitives[index];
            std::size_t expected_vertex_count = 0;
            switch (primitive.role)
            {
            case correction_primitive_role::vertex:
                expected_vertex_count = 1;
                break;
            case correction_primitive_role::triangle:
                expected_vertex_count = 3;
                break;
            case correction_primitive_role::edge:
                expected_vertex_count = 2;
                break;
            default:
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Position-Writer Epoch contains an unknown Primitive Role.";
                return result;
            }

            if (primitive.vertex_motions.size() != expected_vertex_count ||
                primitive.vertex_stable_ids.size() != expected_vertex_count ||
                primitive.segment_start_positions.size() != expected_vertex_count)
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Position-Writer Epoch Primitive vertex arrays do not match its declared role.";
                return result;
            }

            std::vector<uint64_t> unique_vertex_ids =
                primitive.vertex_stable_ids;
            std::sort(unique_vertex_ids.begin(), unique_vertex_ids.end());
            if (std::adjacent_find(
                    unique_vertex_ids.begin(), unique_vertex_ids.end()) !=
                unique_vertex_ids.end())
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Position-Writer Epoch Primitive contains duplicate Stable Vertex IDs.";
                return result;
            }

            std::vector<correction_primitive_identity> sorted_exclusions =
                primitive.excluded_primitive_ids;
            std::sort(sorted_exclusions.begin(), sorted_exclusions.end());
            if (std::adjacent_find(
                    sorted_exclusions.begin(), sorted_exclusions.end()) !=
                sorted_exclusions.end())
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Position-Writer Epoch Primitive contains duplicate topology exclusions.";
                return result;
            }

            const primitive_key key = {
                primitive.stable_id, primitive.mesh_id, primitive.primitive_id
            };
            if (!primitive_lookup.emplace(key, index).second)
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Position-Writer Epoch contains duplicate Stable / Mesh / Primitive identity.";
                return result;
            }

            collision_aabb correction_bounds;
            if (!build_correction_swept_aabb(
                primitive.vertex_motions,
                ccd_settings.contact_thickness,
                correction_bounds))
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Correction Swept AABB could not conservatively enclose a Position-Writer Epoch Primitive.";
                result.diagnostics.push_back({
                    collision_error::invalid_geometry,
                    static_cast<uint32_t>(index),
                    result.diagnostic
                });
                return result;
            }

            std::vector<correction_vertex_motion> final_position_motions;
            final_position_motions.reserve(expected_vertex_count);
            for (const correction_vertex_motion& motion : primitive.vertex_motions)
            {
                if (!finite(motion.before) || !finite(motion.after))
                {
                    result.status = correction_writer_epoch_status::rejected;
                    result.diagnostic =
                        "Position-Writer Epoch contains a non-finite before / after position.";
                    return result;
                }
                final_position_motions.push_back({ motion.after, motion.after });
            }
            for (const asset::vec3& segment_start_position :
                primitive.segment_start_positions)
            {
                if (!finite(segment_start_position))
                {
                    result.status = correction_writer_epoch_status::rejected;
                    result.diagnostic =
                        "Position-Writer Epoch contains a non-finite Integration Segment-start position.";
                    return result;
                }
            }

            collision_aabb final_discrete_bounds;
            if (!build_correction_swept_aabb(
                final_position_motions,
                ccd_settings.contact_thickness,
                final_discrete_bounds))
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Final Discrete AABB could not conservatively enclose the Position-Writer Epoch endpoint geometry.";
                return result;
            }

            correction_broadphase_primitive correction_primitive;
            correction_primitive.stable_id = primitive.stable_id;
            correction_primitive.mesh_id = primitive.mesh_id;
            correction_primitive.primitive_id = primitive.primitive_id;
            correction_primitive.role = primitive.role;
            correction_primitive.envelope_bounds = correction_bounds;
            correction_primitive.vertex_stable_ids = primitive.vertex_stable_ids;
            correction_primitive.excluded_primitive_ids =
                primitive.excluded_primitive_ids;
            correction_primitives.push_back(correction_primitive);

            correction_broadphase_primitive final_primitive;
            final_primitive.stable_id = primitive.stable_id;
            final_primitive.mesh_id = primitive.mesh_id;
            final_primitive.primitive_id = primitive.primitive_id;
            final_primitive.role = primitive.role;
            final_primitive.envelope_bounds = final_discrete_bounds;
            final_primitive.vertex_stable_ids = primitive.vertex_stable_ids;
            final_primitive.excluded_primitive_ids =
                primitive.excluded_primitive_ids;
            final_discrete_primitives.push_back(final_primitive);
        }

        const correction_broadphase_result correction_broadphase =
            generate_correction_broadphase_candidates(
                correction_primitives, mesh_pairs, broadphase_settings);
        result.required_cell_records = correction_broadphase.required_cell_records;
        result.capacity_overflow = correction_broadphase.overflow;
        result.diagnostics = correction_broadphase.diagnostics;
        if (!correction_broadphase.success)
        {
            result.status = correction_broadphase.overflow
                ? correction_writer_epoch_status::blocked
                : correction_writer_epoch_status::rejected;
            result.diagnostic = correction_broadphase.overflow
                ? "Correction Broadphase Capacity is insufficient; no partial Epoch state was committed."
                : "Correction Broadphase could not prove a complete Candidate Set.";
            return result;
        }

        const correction_broadphase_result final_discrete_broadphase =
            generate_correction_broadphase_candidates(
                final_discrete_primitives, mesh_pairs, broadphase_settings);
        result.diagnostics.insert(
            result.diagnostics.end(),
            final_discrete_broadphase.diagnostics.begin(),
            final_discrete_broadphase.diagnostics.end());
        result.capacity_overflow = result.capacity_overflow ||
            final_discrete_broadphase.overflow;
        result.required_cell_records = std::max(
            result.required_cell_records,
            final_discrete_broadphase.required_cell_records);
        if (!final_discrete_broadphase.success)
        {
            result.status = final_discrete_broadphase.overflow
                ? correction_writer_epoch_status::blocked
                : correction_writer_epoch_status::rejected;
            result.diagnostic = final_discrete_broadphase.overflow
                ? "Final Discrete Broadphase Capacity is insufficient; no partial Epoch state was committed."
                : "Final Discrete Broadphase could not prove a complete endpoint Candidate Set.";
            return result;
        }

        correction_segment_candidate_state next_state = state;
        const correction_segment_candidate_update_result begin_result =
            begin_correction_position_writer_epoch(
                next_state, base_integration_segment_id,
                position_writer_epoch_id);
        if (!begin_result.success)
        {
            result.status = correction_writer_epoch_status::rejected;
            result.diagnostic = begin_result.diagnostic;
            return result;
        }

        const correction_segment_candidate_update_result correction_merge =
            merge_correction_position_writer_epoch_candidates(
                next_state,
                base_integration_segment_id,
                position_writer_epoch_id,
                correction_broadphase.candidates,
                broadphase_settings.candidate_capacity);
        if (!correction_merge.success)
        {
            result.status = correction_merge.capacity_overflow
                ? correction_writer_epoch_status::blocked
                : correction_writer_epoch_status::rejected;
            result.capacity_overflow = correction_merge.capacity_overflow;
            result.required_candidates = correction_merge.required_count;
            result.diagnostic = correction_merge.diagnostic;
            return result;
        }

        const correction_segment_candidate_update_result final_merge =
            merge_correction_position_writer_epoch_candidates(
                next_state,
                base_integration_segment_id,
                position_writer_epoch_id,
                final_discrete_broadphase.candidates,
                broadphase_settings.candidate_capacity);
        if (!final_merge.success)
        {
            result.status = final_merge.capacity_overflow
                ? correction_writer_epoch_status::blocked
                : correction_writer_epoch_status::rejected;
            result.capacity_overflow = final_merge.capacity_overflow;
            result.required_candidates = final_merge.required_count;
            result.diagnostic = final_merge.diagnostic;
            return result;
        }

        const auto find_primitive = [&primitive_lookup, &primitives](
            uint64_t stable_id,
            uint32_t mesh_id,
            uint32_t primitive_id) -> const correction_primitive_epoch_input*
        {
            const primitive_key key = { stable_id, mesh_id, primitive_id };
            const auto found = primitive_lookup.find(key);
            return found == primitive_lookup.end()
                ? nullptr : &primitives[found->second];
        };

        const auto find_side_latch = [&next_state](
            const correction_candidate_pair_key& candidate)
            -> const correction_segment_side_latch*
        {
            const auto found = std::find_if(
                next_state.side_latches.begin(),
                next_state.side_latches.end(),
                [&candidate](const correction_segment_side_latch& latch)
                {
                    return latch.key == candidate;
                });
            return found == next_state.side_latches.end()
                ? nullptr : &(*found);
        };

        const auto resolve_vt = [](
            const correction_primitive_epoch_input* first,
            const correction_primitive_epoch_input* second,
            const correction_primitive_epoch_input*& point,
            const correction_primitive_epoch_input*& triangle) -> bool
        {
            if (first == nullptr || second == nullptr)
            {
                return false;
            }
            if (first->role == correction_primitive_role::vertex &&
                second->role == correction_primitive_role::triangle)
            {
                point = first;
                triangle = second;
                return true;
            }
            if (second->role == correction_primitive_role::vertex &&
                first->role == correction_primitive_role::triangle)
            {
                point = second;
                triangle = first;
                return true;
            }
            return false;
        };

        for (const correction_candidate_pair_key& candidate :
            correction_broadphase.candidates)
        {
            const correction_primitive_epoch_input* first = find_primitive(
                candidate.stable_id_a, candidate.mesh_a, candidate.primitive_a);
            const correction_primitive_epoch_input* second = find_primitive(
                candidate.stable_id_b, candidate.mesh_b, candidate.primitive_b);

            correction_writer_epoch_ccd_record record;
            record.candidate = candidate;
            const correction_segment_side_latch* side_latch =
                find_side_latch(candidate);
            if (candidate.contact_type == correction_contact_type::vertex_triangle)
            {
                const correction_primitive_epoch_input* point = nullptr;
                const correction_primitive_epoch_input* triangle = nullptr;
                if (!resolve_vt(first, second, point, triangle))
                {
                    result.status = correction_writer_epoch_status::rejected;
                    result.diagnostic =
                        "Correction Broadphase returned a VT candidate with mismatched primitive roles.";
                    return result;
                }

                vertex_triangle_correction_motion motion;
                if (side_latch != nullptr)
                {
                    motion.has_segment_side_latch = true;
                    motion.segment_reference_normal =
                        side_latch->reference_normal;
                    motion.segment_allowed_side_sign =
                        side_latch->allowed_side_sign;
                }
                motion.point = point->vertex_motions[0];
                motion.segment_start_point = point->segment_start_positions[0];
                for (std::size_t index = 0; index < 3; ++index)
                {
                    motion.triangle[index] = triangle->vertex_motions[index];
                    motion.segment_start_triangle[index] =
                        triangle->segment_start_positions[index];
                }
                record.result = detect_vertex_triangle_correction_motion(
                    motion, ccd_settings);
            }
            else if (candidate.contact_type == correction_contact_type::edge_edge)
            {
                if (first == nullptr || second == nullptr ||
                    first->role != correction_primitive_role::edge ||
                    second->role != correction_primitive_role::edge)
                {
                    result.status = correction_writer_epoch_status::rejected;
                    result.diagnostic =
                        "Correction Broadphase returned an EE candidate with mismatched Edge roles.";
                    return result;
                }

                edge_edge_correction_motion motion;
                if (side_latch != nullptr)
                {
                    motion.has_segment_side_latch = true;
                    motion.segment_reference_normal =
                        side_latch->reference_normal;
                    motion.segment_allowed_side_sign =
                        side_latch->allowed_side_sign;
                }
                for (std::size_t endpoint = 0; endpoint < 2; ++endpoint)
                {
                    motion.edge_a[endpoint] = first->vertex_motions[endpoint];
                    motion.edge_b[endpoint] = second->vertex_motions[endpoint];
                    motion.segment_start_edge_a[endpoint] =
                        first->segment_start_positions[endpoint];
                    motion.segment_start_edge_b[endpoint] =
                        second->segment_start_positions[endpoint];
                }
                record.result = detect_edge_edge_correction_motion(
                    motion, ccd_settings);
            }
            else
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Correction Broadphase returned an unsupported Contact Type.";
                return result;
            }

            if (record.result.status == correction_ccd_status::no_hit ||
                record.result.status == correction_ccd_status::hit)
            {
                if ((record.result.allowed_side_sign != -1 &&
                        record.result.allowed_side_sign != 1) ||
                    !finite(record.result.reference_normal))
                {
                    result.status = correction_writer_epoch_status::rejected;
                    result.diagnostic =
                        "Correction Motion CCD returned a missing Segment Side Latch.";
                    return result;
                }
                if (side_latch == nullptr)
                {
                    correction_segment_side_latch new_latch;
                    new_latch.key = candidate;
                    new_latch.reference_normal =
                        record.result.reference_normal;
                    new_latch.allowed_side_sign =
                        record.result.allowed_side_sign;
                    next_state.side_latches.push_back(new_latch);
                    std::sort(next_state.side_latches.begin(),
                        next_state.side_latches.end(),
                        [](const correction_segment_side_latch& left,
                            const correction_segment_side_latch& right)
                        {
                            return left.key < right.key;
                        });
                }
            }

            result.correction_ccd_results.push_back(record);
            if (record.result.status == correction_ccd_status::path_crossing)
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic = record.result.diagnostic;
                return result;
            }
            if (record.result.status == correction_ccd_status::unresolved ||
                record.result.status == correction_ccd_status::invalid_input)
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic = record.result.diagnostic;
                return result;
            }
        }

        std::vector<correction_discrete_contact_input> discrete_inputs;
        discrete_inputs.reserve(next_state.candidates.size());
        for (const correction_candidate_pair_key& candidate :
            next_state.candidates)
        {
            const correction_primitive_epoch_input* first = find_primitive(
                candidate.stable_id_a, candidate.mesh_a, candidate.primitive_a);
            const correction_primitive_epoch_input* second = find_primitive(
                candidate.stable_id_b, candidate.mesh_b, candidate.primitive_b);
            if (first == nullptr || second == nullptr)
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Segment Candidate Union references a primitive absent from the Position-Writer Epoch snapshot.";
                return result;
            }

            correction_discrete_contact_input input;
            input.key = candidate;
            input.contact_thickness = ccd_settings.contact_thickness;

            if (candidate.contact_type == correction_contact_type::vertex_triangle)
            {
                const correction_primitive_epoch_input* point = nullptr;
                const correction_primitive_epoch_input* triangle = nullptr;
                if (!resolve_vt(first, second, point, triangle))
                {
                    result.status = correction_writer_epoch_status::rejected;
                    result.diagnostic =
                        "Final Discrete VT candidate has mismatched primitive roles.";
                    return result;
                }

                input.vertex_stable_ids[0] = point->vertex_stable_ids[0];
                input.positions[0] = point->vertex_motions[0].after;
                for (std::size_t index = 0; index < 3; ++index)
                {
                    input.vertex_stable_ids[index + 1] =
                        triangle->vertex_stable_ids[index];
                    input.positions[index + 1] =
                        triangle->vertex_motions[index].after;
                }
            }
            else if (candidate.contact_type == correction_contact_type::edge_edge)
            {
                if (first->role != correction_primitive_role::edge ||
                    second->role != correction_primitive_role::edge)
                {
                    result.status = correction_writer_epoch_status::rejected;
                    result.diagnostic =
                        "Final Discrete EE candidate has mismatched Edge roles.";
                    return result;
                }
                for (std::size_t endpoint = 0; endpoint < 2; ++endpoint)
                {
                    input.vertex_stable_ids[endpoint] =
                        first->vertex_stable_ids[endpoint];
                    input.vertex_stable_ids[endpoint + 2] =
                        second->vertex_stable_ids[endpoint];
                    input.positions[endpoint] =
                        first->vertex_motions[endpoint].after;
                    input.positions[endpoint + 2] =
                        second->vertex_motions[endpoint].after;
                }
            }
            else
            {
                result.status = correction_writer_epoch_status::rejected;
                result.diagnostic =
                    "Final Discrete Candidate Union contains an unsupported Contact Type.";
                return result;
            }
            discrete_inputs.push_back(input);
        }

        const correction_contact_set_refresh_result refresh =
            refresh_correction_segment_contact_set_and_csr(
                next_state,
                next_state.candidate_generation,
                next_state.position_writer_epoch_generation,
                discrete_inputs,
                ccd_settings.epsilon_length);
        if (!refresh.success)
        {
            result.status = correction_writer_epoch_status::rejected;
            result.diagnostic = refresh.diagnostic;
            return result;
        }

        result.active_contacts.reserve(next_state.contact_set.size());
        for (const correction_discrete_contact& contact : next_state.contact_set)
        {
            result.active_contacts.push_back(contact.key);
        }
        result.candidate_generation = next_state.candidate_generation;
        result.position_writer_epoch_generation =
            next_state.position_writer_epoch_generation;
        result.required_candidates = next_state.candidates.size();

        state = std::move(next_state);
        result.status = correction_writer_epoch_status::accepted;
        result.state_committed = true;
        return result;
    }

    correction_ccd_result detect_vertex_triangle_correction_motion(
        const vertex_triangle_correction_motion& motion,
        const correction_ccd_settings& settings)
    {
        correction_ccd_result result;
        if (!valid_settings(settings) || !valid_vt_motion(motion))
        {
            result.status = correction_ccd_status::invalid_input;
            result.diagnostic = "Correction VT input or settings are invalid or non-finite.";
            return result;
        }

        vector3 reference_normal;
        int8_t side_sign = 0;
        int orientation_sign = 1;
        if (!prepare_vt_reference(motion, settings,
            reference_normal, side_sign, orientation_sign))
        {
            return make_unresolved(result,
                "Correction VT Segment-start normal or allowed side cannot be established.");
        }

        result.reference_normal = to_asset_vector(reference_normal);
        result.allowed_side_sign = side_sign;
        const real movement_bound = conservative_speed_bound(
            length(subtract(to_vector(motion.point.after), to_vector(motion.point.before))) +
            maximum_displacement(motion.triangle));

        if (!finite(movement_bound))
        {
            return make_unresolved(result,
                "Correction VT displacement bound is not finite.");
        }

        const real threshold = round_up(static_cast<real>(settings.contact_thickness) +
            static_cast<real>(settings.distance_tolerance));
        const candidate_search_result candidate = search_correction_candidate(
            movement_bound,
            threshold,
            settings.maximum_interval_evaluations,
            [&motion, &settings](real s)
            {
                return sample_vertex_triangle(
                    lerp(to_vector(motion.point.before),
                        to_vector(motion.point.after), s),
                    evaluate_triangle(motion.triangle, s),
                    settings.epsilon_length);
            },
            [&motion](real s)
            {
                return distance_lower_from_aabb(
                    lerp(to_vector(motion.point.before),
                        to_vector(motion.point.after), s),
                    evaluate_triangle(motion.triangle, s));
            });
        result.interval_evaluations = candidate.evaluations;

        if (candidate.unresolved)
        {
            return make_unresolved(result,
                "Correction VT distance interval could not be certified within the interval budget.");
        }
        if (!candidate.hit)
        {
            result.status = correction_ccd_status::no_hit;
            result.diagnostic = "Every correction VT interval is certified outside the contact thickness.";
            return result;
        }

        result.hit_witness_valid = true;
        result.witness_s = static_cast<float>(candidate.witness_s);
        result.witness_distance_lower = static_cast<float>(
            candidate.witness.distance_lower);
        result.witness_distance_upper = static_cast<float>(
            candidate.witness.distance_upper);
        result.witness_point_a = to_asset_vector(candidate.witness.point_a);
        result.witness_point_b = to_asset_vector(candidate.witness.point_b);
        result.witness_barycentric = {
            static_cast<float>(candidate.witness.barycentric[0]),
            static_cast<float>(candidate.witness.barycentric[1]),
            static_cast<float>(candidate.witness.barycentric[2])
        };
        const vector3 witness_separation = subtract(
            candidate.witness.point_a, candidate.witness.point_b);
        const real witness_distance = length(witness_separation);
        result.witness_normal = to_asset_vector(
            witness_distance > settings.epsilon_length
                ? multiply(witness_separation, one / witness_distance)
                : multiply(reference_normal, static_cast<real>(side_sign)));

        const side_proof_status side_status = prove_correction_side(
            result.interval_evaluations,
            settings.maximum_interval_evaluations,
            static_cast<real>(settings.epsilon_length) *
                static_cast<real>(settings.epsilon_length),
            [&motion, &reference_normal, side_sign, orientation_sign,
                &settings](const segment& range)
            {
                const interval_vector3 point =
                    interval_motion(motion.point, range);
                const interval_vector3 a =
                    interval_motion(motion.triangle[0], range);
                const interval_vector3 b =
                    interval_motion(motion.triangle[1], range);
                const interval_vector3 c =
                    interval_motion(motion.triangle[2], range);
                const interval_vector3 u = interval_difference(b, a);
                const interval_vector3 v = interval_difference(c, a);
                const interval_vector3 normal = interval_scale_vector(
                    interval_cross(u, v),
                    static_cast<real>(orientation_sign));
                const interval_vector3 separation = interval_difference(point, a);
                return make_side_interval(
                    normal, separation, reference_normal, side_sign,
                    settings.distance_tolerance);
            });

        if (side_status == side_proof_status::crossing)
        {
            result.status = correction_ccd_status::path_crossing;
            result.diagnostic = "Correction VT path crossed the Segment-latched allowed side.";
            return result;
        }
        if (side_status == side_proof_status::unresolved)
        {
            return make_unresolved(result,
                "Correction VT normal hemisphere or signed-side interval is unresolved.");
        }

        result.status = correction_ccd_status::hit;
        result.diagnostic = "Correction VT hit witness is valid and the full path remains on the Segment-latched side.";
        return result;
    }

    correction_ccd_result detect_edge_edge_correction_motion(
        const edge_edge_correction_motion& motion,
        const correction_ccd_settings& settings)
    {
        correction_ccd_result result;
        if (!valid_settings(settings) || !valid_ee_motion(motion))
        {
            result.status = correction_ccd_status::invalid_input;
            result.diagnostic = "Correction EE input or settings are invalid or non-finite.";
            return result;
        }

        vector3 reference_normal;
        int8_t side_sign = 0;
        int orientation_sign = 1;
        if (!prepare_ee_reference(motion, settings,
            reference_normal, side_sign, orientation_sign))
        {
            return make_unresolved(result,
                "Correction EE Segment-start normal or allowed side cannot be established.");
        }

        const vector3 start_a0 = to_vector(motion.segment_start_edge_a[0]);
        const vector3 start_a1 = to_vector(motion.segment_start_edge_a[1]);
        const vector3 start_b0 = to_vector(motion.segment_start_edge_b[0]);
        const vector3 start_b1 = to_vector(motion.segment_start_edge_b[1]);
        if (length(subtract(start_a1, start_a0)) <= settings.epsilon_length ||
            length(subtract(start_b1, start_b0)) <= settings.epsilon_length)
        {
            return make_unresolved(result,
                "Correction EE contains a degenerate Segment-start edge.");
        }

        result.reference_normal = to_asset_vector(reference_normal);
        result.allowed_side_sign = side_sign;
        const real movement_bound = conservative_speed_bound(
            maximum_displacement(motion.edge_a) +
            maximum_displacement(motion.edge_b));
        if (!finite(movement_bound))
        {
            return make_unresolved(result,
                "Correction EE displacement bound is not finite.");
        }

        const real threshold = static_cast<real>(settings.contact_thickness) +
            static_cast<real>(settings.distance_tolerance);
        const candidate_search_result candidate = search_correction_candidate(
            movement_bound,
            threshold,
            settings.maximum_interval_evaluations,
            [&motion, &settings](real s)
            {
                return sample_edge_edge(
                    evaluate_edge(motion.edge_a, s),
                    evaluate_edge(motion.edge_b, s),
                    settings.epsilon_length);
            },
            [&motion](real s)
            {
                return distance_lower_from_aabb(
                    evaluate_edge(motion.edge_a, s),
                    evaluate_edge(motion.edge_b, s));
            });
        result.interval_evaluations = candidate.evaluations;

        if (candidate.unresolved)
        {
            return make_unresolved(result,
                "Correction EE distance interval could not be certified within the interval budget.");
        }
        if (!candidate.hit)
        {
            result.status = correction_ccd_status::no_hit;
            result.diagnostic = "Every correction EE interval is certified outside the contact thickness.";
            return result;
        }

        const std::array<vector3, 2> witness_edge_a =
            evaluate_edge(motion.edge_a, candidate.witness_s);
        const std::array<vector3, 2> witness_edge_b =
            evaluate_edge(motion.edge_b, candidate.witness_s);
        const real witness_edge_a_length = length(subtract(
            witness_edge_a[1], witness_edge_a[0]));
        const real witness_edge_b_length = length(subtract(
            witness_edge_b[1], witness_edge_b[0]));
        if (!is_interior_edge_parameter(
                candidate.witness.parameter_a,
                witness_edge_a_length, settings.epsilon_length) ||
            !is_interior_edge_parameter(
                candidate.witness.parameter_b,
                witness_edge_b_length, settings.epsilon_length))
        {
            return make_unresolved(result,
                "Correction EE witness is endpoint-clamped; a certified persistent feature normal is required.");
        }

        result.hit_witness_valid = true;
        result.witness_s = static_cast<float>(candidate.witness_s);
        result.witness_distance_lower = static_cast<float>(
            candidate.witness.distance_lower);
        result.witness_distance_upper = static_cast<float>(
            candidate.witness.distance_upper);
        result.witness_point_a = to_asset_vector(candidate.witness.point_a);
        result.witness_point_b = to_asset_vector(candidate.witness.point_b);
        result.witness_parameter_a = static_cast<float>(
            candidate.witness.parameter_a);
        result.witness_parameter_b = static_cast<float>(
            candidate.witness.parameter_b);
        const vector3 witness_separation = subtract(
            candidate.witness.point_a, candidate.witness.point_b);
        const real witness_distance = length(witness_separation);
        result.witness_normal = to_asset_vector(
            witness_distance > settings.epsilon_length
                ? multiply(witness_separation, one / witness_distance)
                : multiply(reference_normal, static_cast<real>(side_sign)));

        const side_proof_status side_status = prove_correction_side(
            result.interval_evaluations,
            settings.maximum_interval_evaluations,
            static_cast<real>(settings.epsilon_length) *
                static_cast<real>(settings.epsilon_length),
            [&motion, &reference_normal, side_sign, orientation_sign,
                &settings](const segment& range)
            {
                const interval_vector3 a0 =
                    interval_motion(motion.edge_a[0], range);
                const interval_vector3 a1 =
                    interval_motion(motion.edge_a[1], range);
                const interval_vector3 b0 =
                    interval_motion(motion.edge_b[0], range);
                const interval_vector3 b1 =
                    interval_motion(motion.edge_b[1], range);
                const side_interval_bounds bounds = make_ee_side_interval(
                    a0, a1, b0, b1, reference_normal, side_sign,
                    orientation_sign, settings.distance_tolerance,
                    settings.epsilon_length);
                return bounds;
            });

        if (side_status == side_proof_status::crossing)
        {
            result.status = correction_ccd_status::path_crossing;
            result.diagnostic = "Correction EE path crossed the Segment-latched allowed side.";
            return result;
        }
        if (side_status == side_proof_status::unresolved)
        {
            return make_unresolved(result,
                "Correction EE normal hemisphere or signed-side interval is unresolved.");
        }

        result.status = correction_ccd_status::hit;
        result.diagnostic = "Correction EE hit witness is valid and the full path remains on the Segment-latched side.";
        return result;
    }
}
