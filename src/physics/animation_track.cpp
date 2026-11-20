#include "skin_physics/physics/animation_track.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace skin_physics::physics
{
    namespace
    {
        using asset::vec3;
        constexpr float quaternion_epsilon = 1.0e-12f;

        vec3 lerp(const vec3& start, const vec3& end, float interpolation)
        {
            return {
                start.x + (end.x - start.x) * interpolation,
                start.y + (end.y - start.y) * interpolation,
                start.z + (end.z - start.z) * interpolation
            };
        }

        bool finite(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool finite(const quaternion& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) &&
                std::isfinite(value.z) && std::isfinite(value.w);
        }

        float dot(const quaternion& left, const quaternion& right)
        {
            return left.x * right.x + left.y * right.y +
                left.z * right.z + left.w * right.w;
        }

        quaternion negate(const quaternion& value)
        {
            return { -value.x, -value.y, -value.z, -value.w };
        }

        quaternion lerp(const quaternion& start, const quaternion& end, float interpolation)
        {
            return normalize_quaternion({
                start.x + (end.x - start.x) * interpolation,
                start.y + (end.y - start.y) * interpolation,
                start.z + (end.z - start.z) * interpolation,
                start.w + (end.w - start.w) * interpolation
            });
        }

        animation_sample_result make_error(animation_error error, const std::string& message)
        {
            animation_sample_result result;
            result.error = error;
            result.message = message;
            return result;
        }
    }

    quaternion normalize_quaternion(const quaternion& value)
    {
        const float squared = dot(value, value);
        if (!std::isfinite(squared) || squared <= quaternion_epsilon)
        {
            return {};
        }
        const float reciprocal = 1.0f / std::sqrt(squared);
        return {
            value.x * reciprocal,
            value.y * reciprocal,
            value.z * reciprocal,
            value.w * reciprocal
        };
    }

    quaternion slerp_quaternion(
        const quaternion& start_value,
        const quaternion& end_value,
        float interpolation)
    {
        const quaternion start = normalize_quaternion(start_value);
        quaternion end = normalize_quaternion(end_value);
        const float t = std::max(0.0f, std::min(1.0f, interpolation));
        float cosine = dot(start, end);
        if (cosine < 0.0f)
        {
            end = negate(end);
            cosine = -cosine;
        }
        cosine = std::max(-1.0f, std::min(1.0f, cosine));
        if (cosine > 0.9995f)
        {
            return lerp(start, end, t);
        }

        const float angle = std::acos(cosine);
        const float sine = std::sin(angle);
        if (std::abs(sine) <= quaternion_epsilon)
        {
            return start;
        }
        const float start_weight = std::sin((1.0f - t) * angle) / sine;
        const float end_weight = std::sin(t * angle) / sine;
        return normalize_quaternion({
            start.x * start_weight + end.x * end_weight,
            start.y * start_weight + end.y * end_weight,
            start.z * start_weight + end.z * end_weight,
            start.w * start_weight + end.w * end_weight
        });
    }

    animation_sample_result sample_animation_track(
        const animation_track& track,
        double sample_time,
        const animation_sample_options& options)
    {
        if (track.keyframes.empty())
        {
            return make_error(animation_error::empty_track, "Animation track contains no keyframes.");
        }
        if (!std::isfinite(sample_time))
        {
            return make_error(animation_error::invalid_time, "Animation sample time must be finite.");
        }
        if (options.teleport && !finite(options.teleport_translation))
        {
            return make_error(animation_error::invalid_keyframe,
                "Teleport translation must contain finite values.");
        }

        for (std::size_t index = 0; index < track.keyframes.size(); ++index)
        {
            const animation_keyframe& keyframe = track.keyframes[index];
            if (!std::isfinite(keyframe.time) || !finite(keyframe.translation) ||
                !finite(keyframe.scale) || !finite(keyframe.rotation))
            {
                return make_error(animation_error::invalid_keyframe,
                    "Animation keyframe contains a non-finite field.");
            }
            const float quaternion_squared = dot(keyframe.rotation, keyframe.rotation);
            if (quaternion_squared <= quaternion_epsilon)
            {
                return make_error(animation_error::invalid_quaternion,
                    "Animation keyframe quaternion must have non-zero length.");
            }
            if (index > 0 && keyframe.time <= track.keyframes[index - 1].time)
            {
                return make_error(animation_error::non_monotonic_time,
                    "Animation keyframe times must be strictly increasing.");
            }
        }

        animation_sample_result result;
        const animation_keyframe& first = track.keyframes.front();
        const animation_keyframe& last = track.keyframes.back();
        if (sample_time <= first.time)
        {
            result.value.translation = first.translation;
            result.value.rotation = normalize_quaternion(first.rotation);
            result.value.scale = first.scale;
        }
        else if (sample_time >= last.time)
        {
            result.value.translation = last.translation;
            result.value.rotation = normalize_quaternion(last.rotation);
            result.value.scale = last.scale;
        }
        else
        {
            std::size_t upper_index = 1;
            while (upper_index < track.keyframes.size() &&
                sample_time > track.keyframes[upper_index].time)
            {
                ++upper_index;
            }
            const animation_keyframe& start = track.keyframes[upper_index - 1];
            const animation_keyframe& end = track.keyframes[upper_index];
            const double duration = end.time - start.time;
            const float interpolation = static_cast<float>(
                (sample_time - start.time) / duration);
            result.value.translation = lerp(start.translation, end.translation, interpolation);
            result.value.rotation = slerp_quaternion(
                start.rotation, end.rotation, interpolation);
            result.value.scale = lerp(start.scale, end.scale, interpolation);
            result.value.translation_velocity = {
                static_cast<float>((end.translation.x - start.translation.x) / duration),
                static_cast<float>((end.translation.y - start.translation.y) / duration),
                static_cast<float>((end.translation.z - start.translation.z) / duration)
            };
        }

        if (options.teleport)
        {
            result.value.translation = options.teleport_translation;
            if (options.zero_velocity_on_teleport)
            {
                result.value.translation_velocity = {};
            }
        }

        result.success = true;
        return result;
    }

    namespace
    {
        using animation_component_array = std::array<double, 4>;

        bool finite_component_array(const std::vector<float>& values, std::size_t expected)
        {
            return values.size() == expected &&
                std::all_of(values.begin(), values.end(),
                    [](float value) { return std::isfinite(value); });
        }

        double component_value(const asset::vec4& value, std::size_t component)
        {
            switch (component)
            {
            case 0u: return value.x;
            case 1u: return value.y;
            case 2u: return value.z;
            default: return value.w;
            }
        }

        void set_component_value(
            asset::vec4& value,
            std::size_t component,
            float component_value_input)
        {
            switch (component)
            {
            case 0u: value.x = component_value_input; break;
            case 1u: value.y = component_value_input; break;
            case 2u: value.z = component_value_input; break;
            default: value.w = component_value_input; break;
            }
        }

        asset::vec4 read_key_value(
            const asset::animation_key& key,
            std::size_t component_count)
        {
            asset::vec4 result;
            for (std::size_t component = 0u; component < component_count; ++component)
            {
                set_component_value(
                    result, component, key.value[component]);
            }
            return result;
        }

        bool validate_asset_animation_track(
            const asset::animation_track& track,
            std::size_t& out_component_count,
            animation_channel_span_error& out_error,
            std::string& out_message)
        {
            out_component_count =
                track.path == asset::animation_path::rotation ? 4u : 3u;
            if (track.node_index == asset::invalid_index ||
                track.component_count != out_component_count ||
                track.keys.empty())
            {
                out_error = animation_channel_span_error::invalid_track;
                out_message =
                    "Animation channel requires a valid Node Index, component count and at least one key.";
                return false;
            }

            if (track.path != asset::animation_path::translation &&
                track.path != asset::animation_path::rotation &&
                track.path != asset::animation_path::scale)
            {
                out_error = animation_channel_span_error::invalid_track;
                out_message = "Animation channel path is unsupported.";
                return false;
            }
            if (track.mode != asset::interpolation::step &&
                track.mode != asset::interpolation::linear &&
                track.mode != asset::interpolation::cubic_spline)
            {
                out_error = animation_channel_span_error::invalid_track;
                out_message = "Animation channel interpolation mode is unsupported.";
                return false;
            }

            for (std::size_t index = 0u; index < track.keys.size(); ++index)
            {
                const asset::animation_key& key = track.keys[index];
                if (!std::isfinite(key.time) || key.time < 0.0f ||
                    !finite_component_array(key.value, out_component_count))
                {
                    out_error = animation_channel_span_error::malformed_key;
                    out_message =
                        "Animation key contains an invalid time or value component count / value.";
                    return false;
                }
                if (index > 0u && key.time <= track.keys[index - 1u].time)
                {
                    out_error = animation_channel_span_error::non_monotonic_time;
                    out_message =
                        "Animation key times must be strictly increasing for Span coefficient construction.";
                    return false;
                }
                if (track.mode == asset::interpolation::cubic_spline)
                {
                    if (!finite_component_array(key.in_tangent, out_component_count) ||
                        !finite_component_array(key.out_tangent, out_component_count))
                    {
                        out_error = animation_channel_span_error::malformed_key;
                        out_message =
                            "CUBICSPLINE keys require finite in / out tangent arrays matching the channel component count.";
                        return false;
                    }
                }
                else if ((!key.in_tangent.empty() &&
                            !finite_component_array(key.in_tangent, out_component_count)) ||
                    (!key.out_tangent.empty() &&
                        !finite_component_array(key.out_tangent, out_component_count)))
                {
                    out_error = animation_channel_span_error::malformed_key;
                    out_message =
                        "Non-CUBICSPLINE key tangent arrays must be empty or match the channel component count.";
                    return false;
                }

                if (track.path == asset::animation_path::rotation)
                {
                    double squared = 0.0;
                    for (double value : key.value)
                    {
                        squared += value * value;
                    }
                    if (!std::isfinite(squared) || squared <= 1.0e-24)
                    {
                        out_error = animation_channel_span_error::invalid_quaternion;
                        out_message =
                            "Animation rotation key quaternion must have finite non-zero length.";
                        return false;
                    }
                }
            }
            return true;
        }

        void write_constant_coefficients(
            animation_channel_span& span,
            const asset::animation_key& key,
            std::size_t component_count)
        {
            span.interpolation_mode = asset::interpolation::step;
            span.coefficients = {};
            span.coefficients[0] = read_key_value(key, component_count);
        }

        void build_scalar_channel_coefficients(
            animation_channel_span& span,
            const asset::animation_key& start,
            const asset::animation_key& end,
            std::size_t component_count,
            double normalized_start,
            double normalized_delta)
        {
            const double interval_duration =
                static_cast<double>(end.time) - start.time;
            const double normalized_start_squared =
                normalized_start * normalized_start;
            const double normalized_start_cubed =
                normalized_start_squared * normalized_start;
            const double normalized_delta_squared =
                normalized_delta * normalized_delta;
            const double normalized_delta_cubed =
                normalized_delta_squared * normalized_delta;

            for (std::size_t component = 0u;
                component < component_count; ++component)
            {
                const double p0 = start.value[component];
                const double p1 = end.value[component];
                double a0 = p0;
                double a1 = p1 - p0;
                double a2 = 0.0;
                double a3 = 0.0;

                if (span.interpolation_mode == asset::interpolation::cubic_spline)
                {
                    const double m0 =
                        static_cast<double>(start.out_tangent[component]) *
                        interval_duration;
                    const double m1 =
                        static_cast<double>(end.in_tangent[component]) *
                        interval_duration;
                    a1 = m0;
                    a2 = -3.0 * p0 + 3.0 * p1 - 2.0 * m0 - m1;
                    a3 = 2.0 * p0 - 2.0 * p1 + m0 + m1;
                }

                double c0 = 0.0;
                double c1 = 0.0;
                double c2 = 0.0;
                double c3 = 0.0;
                if (span.interpolation_mode == asset::interpolation::linear)
                {
                    c0 = p0 + a1 * normalized_start;
                    c1 = a1 * normalized_delta;
                }
                else
                {
                    c0 = a0 + a1 * normalized_start +
                        a2 * normalized_start_squared +
                        a3 * normalized_start_cubed;
                    c1 = normalized_delta *
                        (a1 + 2.0 * a2 * normalized_start +
                            3.0 * a3 * normalized_start_squared);
                    c2 = normalized_delta_squared *
                        (a2 + 3.0 * a3 * normalized_start);
                    c3 = normalized_delta_cubed * a3;
                }

                set_component_value(
                    span.coefficients[0], component, static_cast<float>(c0));
                set_component_value(
                    span.coefficients[1], component, static_cast<float>(c1));
                set_component_value(
                    span.coefficients[2], component, static_cast<float>(c2));
                set_component_value(
                    span.coefficients[3], component, static_cast<float>(c3));
            }
        }

        bool finite_asset_vec4(const asset::vec4& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) &&
                std::isfinite(value.z) && std::isfinite(value.w);
        }

        double dot_quaternion(const animation_component_array& left,
            const animation_component_array& right)
        {
            double result = 0.0;
            for (std::size_t component = 0u; component < 4u; ++component)
            {
                result += left[component] * right[component];
            }
            return result;
        }

        bool normalize_quaternion_double(
            const animation_component_array& input,
            animation_component_array& output,
            double& out_length)
        {
            const double squared = dot_quaternion(input, input);
            if (!std::isfinite(squared) || squared <= 1.0e-24)
            {
                return false;
            }
            out_length = std::sqrt(squared);
            if (!std::isfinite(out_length) || out_length <= 1.0e-12)
            {
                return false;
            }
            for (std::size_t component = 0u; component < 4u; ++component)
            {
                output[component] = input[component] / out_length;
                if (!std::isfinite(output[component]))
                {
                    return false;
                }
            }
            return true;
        }

        bool interpolate_rotation_key_values(
            const asset::animation_key& start,
            const asset::animation_key& end,
            double interpolation,
            asset::vec4& out_value)
        {
            animation_component_array start_value = {
                start.value[0], start.value[1], start.value[2], start.value[3]
            };
            animation_component_array end_value = {
                end.value[0], end.value[1], end.value[2], end.value[3]
            };
            animation_component_array normalized_start = {};
            animation_component_array normalized_end = {};
            double start_length = 0.0;
            double end_length = 0.0;
            if (!normalize_quaternion_double(
                    start_value, normalized_start, start_length) ||
                !normalize_quaternion_double(
                    end_value, normalized_end, end_length))
            {
                return false;
            }

            double cosine = dot_quaternion(normalized_start, normalized_end);
            if (cosine < 0.0)
            {
                cosine = -cosine;
                for (double& component : normalized_end)
                {
                    component = -component;
                }
            }
            cosine = std::max(-1.0, std::min(1.0, cosine));
            const double t = std::max(0.0, std::min(1.0, interpolation));
            animation_component_array value = {};
            if (cosine > 0.9995)
            {
                animation_component_array raw = {};
                for (std::size_t component = 0u; component < 4u; ++component)
                {
                    raw[component] =
                        normalized_start[component] * (1.0 - t) +
                        normalized_end[component] * t;
                }
                double length = 0.0;
                if (!normalize_quaternion_double(raw, value, length))
                {
                    return false;
                }
            }
            else
            {
                const double angle = std::acos(cosine);
                const double sine = std::sin(angle);
                if (!std::isfinite(sine) || std::abs(sine) <= 1.0e-12)
                {
                    return false;
                }
                for (std::size_t component = 0u; component < 4u; ++component)
                {
                    const double tangent =
                        (normalized_end[component] -
                            normalized_start[component] * cosine) / sine;
                    value[component] =
                        normalized_start[component] * std::cos(angle * t) +
                        tangent * std::sin(angle * t);
                }
            }

            out_value = {
                static_cast<float>(value[0]),
                static_cast<float>(value[1]),
                static_cast<float>(value[2]),
                static_cast<float>(value[3])
            };
            return finite_asset_vec4(out_value);
        }

        animation_channel_span_sample_result make_span_sample_error(
            animation_channel_span_error error,
            const std::string& message)
        {
            animation_channel_span_sample_result result;
            result.error = error;
            result.message = message;
            return result;
        }
    }

    animation_channel_span_build_result build_animation_channel_span(
        const asset::animation_track& track,
        double clip_time_start,
        double clip_time_delta)
    {
        animation_channel_span_build_result result;
        std::size_t component_count = 0u;
        if (!std::isfinite(clip_time_start) ||
            !std::isfinite(clip_time_delta) ||
            !std::isfinite(clip_time_start + clip_time_delta) ||
            (clip_time_delta != 0.0 &&
                clip_time_start + clip_time_delta == clip_time_start))
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Span Clip-Time start / delta must be finite and advance at the requested double-precision magnitude.";
            return result;
        }
        if (!validate_asset_animation_track(
                track, component_count, result.error, result.message))
        {
            return result;
        }

        animation_channel_span candidate;
        candidate.node_index = track.node_index;
        candidate.path = track.path;
        candidate.interpolation_mode = track.mode;
        candidate.clip_time_start = clip_time_start;
        candidate.clip_time_delta = clip_time_delta;

        const double clip_time_end = clip_time_start + clip_time_delta;
        const double minimum_time = std::min(clip_time_start, clip_time_end);
        const double maximum_time = std::max(clip_time_start, clip_time_end);
        const asset::animation_key& first = track.keys.front();
        const asset::animation_key& last = track.keys.back();

        if (track.keys.size() == 1u)
        {
            // A one-key glTF sampler is constant for its entire Clip-Time
            // domain. Its sole key is not an interior Span boundary.
            write_constant_coefficients(candidate, first, component_count);
            result.value = candidate;
            result.success = true;
            return result;
        }

        if (clip_time_delta == 0.0)
        {
            if (clip_time_start <= first.time)
            {
                write_constant_coefficients(candidate, first, component_count);
            }
            else if (clip_time_start >= last.time)
            {
                write_constant_coefficients(candidate, last, component_count);
            }
            else
            {
                const auto exact_key = std::lower_bound(
                    track.keys.begin(),
                    track.keys.end(),
                    clip_time_start,
                    [](const asset::animation_key& key, double time)
                    {
                        return static_cast<double>(key.time) < time;
                    });
                if (exact_key != track.keys.end() &&
                    static_cast<double>(exact_key->time) == clip_time_start)
                {
                    write_constant_coefficients(candidate, *exact_key, component_count);
                }
                else
                {
                    std::size_t upper_index = 1u;
                    while (upper_index < track.keys.size() &&
                        clip_time_start > track.keys[upper_index].time)
                    {
                        ++upper_index;
                    }
                    const asset::animation_key& start = track.keys[upper_index - 1u];
                    const asset::animation_key& end = track.keys[upper_index];
                    const double interval_duration =
                        static_cast<double>(end.time) - start.time;
                    const double normalized =
                        (clip_time_start - start.time) / interval_duration;

                    if (track.mode == asset::interpolation::step)
                    {
                        write_constant_coefficients(candidate, start, component_count);
                    }
                    else if (track.path == asset::animation_path::rotation &&
                        track.mode == asset::interpolation::linear)
                    {
                        asset::vec4 paused_rotation;
                        if (!interpolate_rotation_key_values(
                                start, end, normalized, paused_rotation))
                        {
                            result.error = animation_channel_span_error::invalid_quaternion;
                            result.message =
                                "Paused LINEAR rotation Span could not normalize its sampled quaternion.";
                            return result;
                        }
                        candidate.coefficients[0] = paused_rotation;
                        candidate.coefficients[1] = paused_rotation;
                    }
                    else
                    {
                        build_scalar_channel_coefficients(
                            candidate, start, end, component_count, normalized, 0.0);
                    }
                }
            }
        }
        else if (maximum_time <= first.time)
        {
            write_constant_coefficients(candidate, first, component_count);
        }
        else if (minimum_time >= last.time)
        {
            write_constant_coefficients(candidate, last, component_count);
        }
        else
        {
            // An animation Span is bounded by key times. Reject a range that
            // crosses a key, rather than silently approximating two segments
            // with one polynomial.
            for (const asset::animation_key& key : track.keys)
            {
                if (static_cast<double>(key.time) > minimum_time &&
                    static_cast<double>(key.time) < maximum_time)
                {
                    result.error =
                        animation_channel_span_error::span_crosses_key_boundary;
                    result.message =
                        "Animation Span crosses an interior Key boundary and must be split by the CPU Scheduler.";
                    return result;
                }
            }

            const double midpoint =
                clip_time_start + clip_time_delta * 0.5;
            std::size_t upper_index = 1u;
            while (upper_index < track.keys.size() &&
                midpoint >= track.keys[upper_index].time)
            {
                ++upper_index;
            }
            if (upper_index >= track.keys.size())
            {
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Span could not resolve one key interval for its midpoint.";
                return result;
            }

            const asset::animation_key& start = track.keys[upper_index - 1u];
            const asset::animation_key& end = track.keys[upper_index];
            const double interval_duration =
                static_cast<double>(end.time) - start.time;
            const double normalized_start =
                (clip_time_start - start.time) / interval_duration;
            const double normalized_delta =
                clip_time_delta / interval_duration;
            if (!std::isfinite(normalized_start) ||
                !std::isfinite(normalized_delta))
            {
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Span normalized Clip-Time coefficients are not finite.";
                return result;
            }

            if (track.mode == asset::interpolation::step)
            {
                write_constant_coefficients(candidate, start, component_count);
            }
            else if (track.path == asset::animation_path::rotation &&
                track.mode == asset::interpolation::linear)
            {
                const double normalized_end = normalized_start + normalized_delta;
                asset::vec4 rotation_start;
                asset::vec4 rotation_end;
                if (!interpolate_rotation_key_values(
                        start, end, normalized_start, rotation_start) ||
                    !interpolate_rotation_key_values(
                        start, end, normalized_end, rotation_end))
                {
                    result.error = animation_channel_span_error::invalid_quaternion;
                    result.message =
                        "LINEAR rotation Span could not normalize its Clip-Time endpoint quaternions.";
                    return result;
                }
                candidate.coefficients[0] = rotation_start;
                candidate.coefficients[1] = rotation_end;
            }
            else
            {
                build_scalar_channel_coefficients(
                    candidate,
                    start,
                    end,
                    component_count,
                    normalized_start,
                    normalized_delta);
            }
        }

        for (const asset::vec4& coefficient : candidate.coefficients)
        {
            if (!finite_asset_vec4(coefficient))
            {
                result.error = animation_channel_span_error::malformed_key;
                result.message =
                    "Animation Span coefficient conversion exceeded finite float range.";
                return result;
            }
        }

        result.value = candidate;
        result.success = true;
        return result;
    }

    animation_channel_spans_build_result build_animation_channel_spans(
        const asset::animation_track& track,
        double clip_time_start,
        double clip_time_delta)
    {
        animation_channel_spans_build_result result;
        std::size_t component_count = 0u;
        if (!std::isfinite(clip_time_start) ||
            !std::isfinite(clip_time_delta) ||
            !std::isfinite(clip_time_start + clip_time_delta) ||
            (clip_time_delta != 0.0 &&
                clip_time_start + clip_time_delta == clip_time_start))
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Span schedule Clip-Time start / delta must be finite and advance at the requested double-precision magnitude.";
            return result;
        }
        if (!validate_asset_animation_track(
                track, component_count, result.error, result.message))
        {
            return result;
        }

        if (track.keys.size() == 1u)
        {
            animation_channel_span_build_result only_span =
                build_animation_channel_span(
                    track, clip_time_start, clip_time_delta);
            if (!only_span.success)
            {
                result.error = only_span.error;
                result.message = only_span.message;
                return result;
            }
            result.values.push_back(std::move(only_span.value));
            result.success = true;
            return result;
        }

        const double clip_time_end = clip_time_start + clip_time_delta;
        std::vector<double> boundaries;
        boundaries.reserve(track.keys.size() + 2u);
        boundaries.push_back(clip_time_start);

        if (clip_time_delta != 0.0)
        {
            const double minimum_time =
                std::min(clip_time_start, clip_time_end);
            const double maximum_time =
                std::max(clip_time_start, clip_time_end);
            for (const asset::animation_key& key : track.keys)
            {
                const double key_time = static_cast<double>(key.time);
                if (key_time > minimum_time && key_time < maximum_time)
                {
                    boundaries.push_back(key_time);
                }
            }
            if (clip_time_delta < 0.0)
            {
                std::sort(
                    boundaries.begin() + 1,
                    boundaries.end(),
                    [](double left, double right)
                    {
                        return left > right;
                    });
            }
            else
            {
                std::sort(boundaries.begin() + 1, boundaries.end());
            }
        }

        boundaries.push_back(clip_time_end);
        result.values.reserve(boundaries.size() - 1u);
        for (std::size_t index = 0u;
            index + 1u < boundaries.size(); ++index)
        {
            const double span_start = boundaries[index];
            const double span_delta =
                boundaries[index + 1u] - boundaries[index];
            animation_channel_span_build_result span_result =
                build_animation_channel_span(track, span_start, span_delta);
            if (!span_result.success)
            {
                result.values.clear();
                result.error = span_result.error;
                result.message = span_result.message;
                return result;
            }
            result.values.push_back(std::move(span_result.value));
        }

        result.success = true;
        return result;
    }

    animation_channel_span_schedule_result schedule_animation_channel_spans(
        const asset::animation_track& track,
        double clip_time_start,
        double clip_time_delta,
        double world_time_start,
        double world_time_duration)
    {
        animation_channel_span_schedule_result result;
        if (!std::isfinite(world_time_start) ||
            !std::isfinite(world_time_duration) ||
            world_time_duration <= 0.0 ||
            !std::isfinite(world_time_start + world_time_duration) ||
            world_time_start + world_time_duration == world_time_start)
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Span scheduling requires finite World-Time start and a positive representable Tick duration.";
            return result;
        }

        animation_channel_spans_build_result channel_spans =
            build_animation_channel_spans(
                track, clip_time_start, clip_time_delta);
        if (!channel_spans.success)
        {
            result.error = channel_spans.error;
            result.message = channel_spans.message;
            return result;
        }

        result.values.reserve(channel_spans.values.size());
        for (const animation_channel_span& channel_span : channel_spans.values)
        {
            double world_start_fraction = 0.0;
            double world_duration_fraction = 1.0;
            if (clip_time_delta != 0.0)
            {
                world_start_fraction =
                    (channel_span.clip_time_start - clip_time_start) /
                    clip_time_delta;
                world_duration_fraction =
                    channel_span.clip_time_delta / clip_time_delta;
            }

            constexpr double fraction_tolerance =
                std::numeric_limits<double>::epsilon() * 32.0;
            if (!std::isfinite(world_start_fraction) ||
                !std::isfinite(world_duration_fraction) ||
                world_start_fraction < -fraction_tolerance ||
                world_start_fraction > 1.0 + fraction_tolerance ||
                world_duration_fraction <= 0.0 ||
                world_duration_fraction > 1.0 + fraction_tolerance)
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Span Clip-Time partition could not be mapped into a monotonic World-Time interval.";
                return result;
            }

            world_start_fraction =
                std::max(0.0, std::min(1.0, world_start_fraction));
            world_duration_fraction =
                std::max(0.0, std::min(1.0, world_duration_fraction));
            scheduled_animation_channel_span scheduled;
            scheduled.channel = channel_span;
            scheduled.world_time_start =
                world_time_start +
                world_start_fraction * world_time_duration;
            scheduled.world_time_duration =
                world_duration_fraction * world_time_duration;
            if (!std::isfinite(scheduled.world_time_start) ||
                !std::isfinite(scheduled.world_time_duration) ||
                scheduled.world_time_duration <= 0.0 ||
                !std::isfinite(
                    scheduled.world_time_start +
                    scheduled.world_time_duration))
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Span World-Time partition is not representable as positive finite intervals.";
                return result;
            }
            result.values.push_back(std::move(scheduled));
        }

        result.success = !result.values.empty();
        if (!result.success)
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Span Scheduler produced an empty World-Time partition.";
        }
        return result;
    }

    animation_clip_spans_build_result build_animation_clip_spans(
        const asset::animation_clip& clip,
        uint32_t node_count,
        double clip_time_start,
        double clip_time_delta)
    {
        animation_clip_spans_build_result result;
        if (node_count == 0u || clip.tracks.empty() ||
            clip.tracks.size() >
                static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))
        {
            result.error = animation_channel_span_error::invalid_track;
            result.message =
                "Animation Clip Span scheduling requires at least one Node and Track count representable by the GPU ABI.";
            return result;
        }
        if (!std::isfinite(static_cast<double>(clip.duration)) ||
            clip.duration < 0.0f ||
            !std::isfinite(clip_time_start) ||
            !std::isfinite(clip_time_delta) ||
            !std::isfinite(clip_time_start + clip_time_delta) ||
            (clip_time_delta != 0.0 &&
                clip_time_start + clip_time_delta == clip_time_start))
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Clip duration and requested Clip-Time interval must be finite and representable.";
            return result;
        }

        const double clip_time_end = clip_time_start + clip_time_delta;
        const double duration = static_cast<double>(clip.duration);
        if (clip_time_start < 0.0 || clip_time_start > duration ||
            clip_time_end < 0.0 || clip_time_end > duration)
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Clip Span interval must remain inside one already-unwrapped Clip duration.";
            return result;
        }

        std::size_t component_count = 0u;
        for (std::size_t track_index = 0u;
            track_index < clip.tracks.size(); ++track_index)
        {
            const asset::animation_track& track = clip.tracks[track_index];
            if (!validate_asset_animation_track(
                    track, component_count, result.error, result.message))
            {
                return result;
            }
            if (track.node_index >= node_count ||
                static_cast<double>(track.keys.back().time) > duration)
            {
                result.error = animation_channel_span_error::invalid_track;
                result.message =
                    "Animation Clip Track references a Node outside the Model or a Key beyond the Clip duration.";
                return result;
            }

            for (std::size_t previous_index = 0u;
                previous_index < track_index; ++previous_index)
            {
                const asset::animation_track& previous =
                    clip.tracks[previous_index];
                if (previous.node_index == track.node_index &&
                    previous.path == track.path)
                {
                    result.error = animation_channel_span_error::invalid_track;
                    result.message =
                        "Animation Clip contains more than one Track for the same Node / Path channel.";
                    return result;
                }
            }
        }

        const double minimum_time =
            std::min(clip_time_start, clip_time_end);
        const double maximum_time =
            std::max(clip_time_start, clip_time_end);
        std::size_t boundary_capacity = 2u;
        for (const asset::animation_track& track : clip.tracks)
        {
            if (track.keys.size() >
                static_cast<std::size_t>(
                    std::numeric_limits<uint32_t>::max()) - boundary_capacity)
            {
                result.error = animation_channel_span_error::invalid_track;
                result.message =
                    "Animation Clip contains more Key boundaries than the uint32 Span Index ABI can represent.";
                return result;
            }
            boundary_capacity += track.keys.size();
        }
        std::vector<double> boundaries;
        boundaries.reserve(boundary_capacity);
        boundaries.push_back(clip_time_start);
        if (clip_time_delta != 0.0)
        {
            for (const asset::animation_track& track : clip.tracks)
            {
                // A one-key sampler is constant over the entire Clip and
                // therefore contributes no interpolation Span boundary.
                if (track.keys.size() <= 1u)
                {
                    continue;
                }
                for (const asset::animation_key& key : track.keys)
                {
                    const double key_time = static_cast<double>(key.time);
                    if (key_time > minimum_time && key_time < maximum_time)
                    {
                        boundaries.push_back(key_time);
                    }
                }
            }

            if (clip_time_delta < 0.0)
            {
                std::sort(
                    boundaries.begin() + 1,
                    boundaries.end(),
                    [](double left, double right)
                    {
                        return left > right;
                    });
            }
            else
            {
                std::sort(boundaries.begin() + 1, boundaries.end());
            }
            boundaries.erase(
                std::unique(boundaries.begin() + 1, boundaries.end()),
                boundaries.end());
        }
        boundaries.push_back(clip_time_end);
        if (boundaries.size() <= 1u ||
            boundaries.size() - 1u >
                static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))
        {
            result.error = animation_channel_span_error::invalid_track;
            result.message =
                "Animation Clip Span count cannot be represented by the uint32 Span Index ABI.";
            return result;
        }

        result.values.reserve(boundaries.size() - 1u);
        for (std::size_t span_index = 0u;
            span_index + 1u < boundaries.size(); ++span_index)
        {
            animation_clip_span clip_span;
            clip_span.span_index = static_cast<uint32_t>(span_index);
            clip_span.clip_time_start = boundaries[span_index];
            clip_span.clip_time_delta =
                boundaries[span_index + 1u] - boundaries[span_index];
            clip_span.channels.reserve(clip.tracks.size());

            for (std::size_t track_index = 0u;
                track_index < clip.tracks.size(); ++track_index)
            {
                animation_channel_span_build_result channel_result =
                    build_animation_channel_span(
                        clip.tracks[track_index],
                        clip_span.clip_time_start,
                        clip_span.clip_time_delta);
                if (!channel_result.success)
                {
                    result.values.clear();
                    result.error = channel_result.error;
                    result.message = channel_result.message;
                    return result;
                }
                channel_result.value.track_index =
                    static_cast<uint32_t>(track_index);
                clip_span.channels.push_back(
                    std::move(channel_result.value));
            }

            result.values.push_back(std::move(clip_span));
        }

        result.success = true;
        return result;
    }

    animation_clip_span_schedule_result schedule_animation_clip_spans(
        const asset::animation_clip& clip,
        uint32_t node_count,
        double clip_time_start,
        double clip_time_delta,
        double world_time_start,
        double world_time_duration)
    {
        animation_clip_span_schedule_result result;
        const double world_time_end =
            world_time_start + world_time_duration;
        if (!std::isfinite(world_time_start) ||
            !std::isfinite(world_time_duration) ||
            world_time_duration <= 0.0 ||
            !std::isfinite(world_time_end) ||
            world_time_end == world_time_start)
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Clip scheduling requires a finite positive representable Logical Tick interval.";
            return result;
        }

        animation_clip_spans_build_result clip_spans =
            build_animation_clip_spans(
                clip, node_count, clip_time_start, clip_time_delta);
        if (!clip_spans.success)
        {
            result.error = clip_spans.error;
            result.message = clip_spans.message;
            return result;
        }

        result.values.reserve(clip_spans.values.size());
        for (const animation_clip_span& clip_span : clip_spans.values)
        {
            double world_start_fraction = 0.0;
            double world_duration_fraction = 1.0;
            if (clip_time_delta != 0.0)
            {
                world_start_fraction =
                    (clip_span.clip_time_start - clip_time_start) /
                    clip_time_delta;
                world_duration_fraction =
                    clip_span.clip_time_delta / clip_time_delta;
            }

            constexpr double fraction_tolerance =
                std::numeric_limits<double>::epsilon() * 32.0;
            if (!std::isfinite(world_start_fraction) ||
                !std::isfinite(world_duration_fraction) ||
                world_start_fraction < -fraction_tolerance ||
                world_start_fraction > 1.0 + fraction_tolerance ||
                world_duration_fraction <= 0.0 ||
                world_duration_fraction > 1.0 + fraction_tolerance)
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Synchronized Clip Span partition could not be mapped into monotonic World-Time intervals.";
                return result;
            }

            world_start_fraction =
                std::max(0.0, std::min(1.0, world_start_fraction));
            world_duration_fraction =
                std::max(0.0, std::min(1.0, world_duration_fraction));
            scheduled_animation_clip_span scheduled;
            scheduled.clip = clip_span;
            scheduled.world_time_start =
                world_time_start +
                world_start_fraction * world_time_duration;
            scheduled.world_time_duration =
                world_duration_fraction * world_time_duration;
            if (!std::isfinite(scheduled.world_time_start) ||
                !std::isfinite(scheduled.world_time_duration) ||
                scheduled.world_time_duration <= 0.0 ||
                !std::isfinite(
                    scheduled.world_time_start +
                    scheduled.world_time_duration))
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Synchronized Clip Span World-Time partition is not representable as positive finite intervals.";
                return result;
            }
            result.values.push_back(std::move(scheduled));
        }

        result.success = !result.values.empty();
        if (!result.success)
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Clip Scheduler produced an empty World-Time partition.";
        }
        return result;
    }

    animation_clip_span_schedule_result schedule_animation_clip_playback_spans(
        const asset::animation_clip& clip,
        uint32_t node_count,
        double clip_time_start,
        double playback_speed,
        bool loop,
        double world_time_start,
        double world_time_duration,
        uint32_t maximum_span_count)
    {
        animation_clip_span_schedule_result result;
        const double clip_duration = static_cast<double>(clip.duration);
        const double world_time_end =
            world_time_start + world_time_duration;
        if (!std::isfinite(clip_duration) || clip_duration < 0.0 ||
            !std::isfinite(clip_time_start) ||
            !std::isfinite(playback_speed) ||
            !std::isfinite(world_time_start) ||
            !std::isfinite(world_time_duration) ||
            world_time_duration <= 0.0 ||
            !std::isfinite(world_time_end) ||
            world_time_end == world_time_start ||
            maximum_span_count == 0u ||
            clip_time_start < 0.0 || clip_time_start > clip_duration)
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Playback scheduling requires finite Clip / World times, a valid Clip start, finite Speed and a positive Span limit.";
            return result;
        }
        if (clip.tracks.empty() || node_count == 0u)
        {
            result.error = animation_channel_span_error::invalid_track;
            result.message =
                "Animation Playback scheduling requires a Clip with Node-bound Tracks.";
            return result;
        }
        if (loop && clip_duration <= 0.0 && playback_speed != 0.0)
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "A non-zero-speed looping Track requires a positive Clip duration.";
            return result;
        }

        const auto append_world_chunk =
            [&clip, node_count, world_time_start, world_time_duration,
                maximum_span_count, &result](
                double world_offset,
                double world_chunk_duration,
                double chunk_clip_start,
                double chunk_clip_delta) -> bool
        {
            const double chunk_world_start =
                world_time_start + world_offset;
            const double chunk_world_end =
                chunk_world_start + world_chunk_duration;
            if (!std::isfinite(world_offset) ||
                world_offset < 0.0 ||
                !std::isfinite(world_chunk_duration) ||
                world_chunk_duration <= 0.0 ||
                !std::isfinite(chunk_world_start) ||
                !std::isfinite(chunk_world_end) ||
                chunk_world_end <= chunk_world_start ||
                !std::isfinite(chunk_clip_start) ||
                !std::isfinite(chunk_clip_delta) ||
                !std::isfinite(chunk_clip_start + chunk_clip_delta) ||
                (chunk_clip_delta != 0.0 &&
                    chunk_clip_start + chunk_clip_delta == chunk_clip_start))
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Playback chunk cannot advance its absolute World / Clip time at the requested double-precision magnitude.";
                return false;
            }

            animation_clip_span_schedule_result chunk =
                schedule_animation_clip_spans(
                    clip,
                    node_count,
                    chunk_clip_start,
                    chunk_clip_delta,
                    chunk_world_start,
                    world_chunk_duration);
            if (!chunk.success)
            {
                result.values.clear();
                result.error = chunk.error;
                result.message = chunk.message;
                return false;
            }
            if (chunk.values.size() >
                static_cast<std::size_t>(maximum_span_count) -
                    std::min(
                        static_cast<std::size_t>(maximum_span_count),
                        result.values.size()))
            {
                result.values.clear();
                result.error = animation_channel_span_error::capacity_exceeded;
                result.message =
                    "Animation Playback scheduling exceeded the configured immutable Clip Span count limit.";
                return false;
            }

            result.values.reserve(result.values.size() + chunk.values.size());
            for (scheduled_animation_clip_span& scheduled : chunk.values)
            {
                if (result.values.size() >=
                    static_cast<std::size_t>(maximum_span_count) ||
                    result.values.size() >=
                    static_cast<std::size_t>(
                        std::numeric_limits<uint32_t>::max()))
                {
                    result.values.clear();
                    result.error = animation_channel_span_error::capacity_exceeded;
                    result.message =
                        "Animation Playback Span Index exceeds its configured or uint32 capacity.";
                    return false;
                }
                scheduled.clip.span_index =
                    static_cast<uint32_t>(result.values.size());
                result.values.push_back(std::move(scheduled));
            }
            return true;
        };

        double remaining_world_time = world_time_duration;
        double world_offset = 0.0;
        double current_clip_time = clip_time_start;
        uint32_t wrap_count = 0u;

        if (playback_speed == 0.0 || clip_duration == 0.0)
        {
            if (!append_world_chunk(
                    0.0, world_time_duration, current_clip_time, 0.0))
            {
                return result;
            }
            result.success = true;
            return result;
        }

        while (remaining_world_time > 0.0)
        {
            if (loop)
            {
                if (playback_speed > 0.0 &&
                    current_clip_time >= clip_duration)
                {
                    current_clip_time = 0.0;
                    if (wrap_count == std::numeric_limits<uint32_t>::max())
                    {
                        result.values.clear();
                        result.error = animation_channel_span_error::capacity_exceeded;
                        result.message =
                            "Animation Playback Loop wrap count exceeds uint32 capacity.";
                        return result;
                    }
                    ++wrap_count;
                }
                else if (playback_speed < 0.0 && current_clip_time <= 0.0)
                {
                    current_clip_time = clip_duration;
                    if (wrap_count == std::numeric_limits<uint32_t>::max())
                    {
                        result.values.clear();
                        result.error = animation_channel_span_error::capacity_exceeded;
                        result.message =
                            "Animation Playback Loop wrap count exceeds uint32 capacity.";
                        return result;
                    }
                    ++wrap_count;
                }
            }

            const bool moving_forward = playback_speed > 0.0;
            const double distance_to_boundary =
                moving_forward
                    ? clip_duration - current_clip_time
                    : current_clip_time;
            if (distance_to_boundary < 0.0 ||
                !std::isfinite(distance_to_boundary))
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Playback cursor moved outside its Clip boundary range.";
                return result;
            }

            if (distance_to_boundary == 0.0)
            {
                if (loop)
                {
                    current_clip_time =
                        moving_forward ? 0.0 : clip_duration;
                    if (wrap_count == std::numeric_limits<uint32_t>::max())
                    {
                        result.values.clear();
                        result.error = animation_channel_span_error::capacity_exceeded;
                        result.message =
                            "Animation Playback Loop wrap count exceeds uint32 capacity.";
                        return result;
                    }
                    ++wrap_count;
                    continue;
                }

                if (!append_world_chunk(
                        world_offset,
                        remaining_world_time,
                        current_clip_time,
                        0.0))
                {
                    return result;
                }
                world_offset += remaining_world_time;
                remaining_world_time = 0.0;
                break;
            }

            const double absolute_speed = std::abs(playback_speed);
            const double world_time_to_boundary =
                distance_to_boundary / absolute_speed;
            if (std::isnan(world_time_to_boundary) ||
                world_time_to_boundary <= 0.0)
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Playback cannot derive a positive representable World-Time duration to its next Clip boundary.";
                return result;
            }

            const bool reaches_boundary =
                world_time_to_boundary <= remaining_world_time;
            const double chunk_world_duration =
                reaches_boundary
                    ? world_time_to_boundary
                    : remaining_world_time;
            const double chunk_clip_delta =
                playback_speed * chunk_world_duration;
            if (!std::isfinite(chunk_clip_delta) ||
                chunk_clip_delta == 0.0 ||
                !std::isfinite(current_clip_time + chunk_clip_delta) ||
                current_clip_time + chunk_clip_delta == current_clip_time)
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Playback Clip-Time increment is not representable at the requested Speed and World-Time interval.";
                return result;
            }

            const double next_clip_time =
                reaches_boundary
                    ? (moving_forward ? clip_duration : 0.0)
                    : current_clip_time + chunk_clip_delta;
            if (next_clip_time < 0.0 || next_clip_time > clip_duration ||
                !append_world_chunk(
                    world_offset,
                    chunk_world_duration,
                    current_clip_time,
                    next_clip_time - current_clip_time))
            {
                if (result.error == animation_channel_span_error::none)
                {
                    result.error = animation_channel_span_error::invalid_time;
                    result.message =
                        "Animation Playback chunk crossed a Clip boundary without an explicit wrap record.";
                }
                result.values.clear();
                return result;
            }

            world_offset += chunk_world_duration;
            remaining_world_time -= chunk_world_duration;
            if (remaining_world_time < 0.0 &&
                remaining_world_time >=
                    -std::numeric_limits<double>::epsilon() *
                        world_time_duration * 8.0)
            {
                remaining_world_time = 0.0;
            }
            current_clip_time = next_clip_time;

            if (reaches_boundary && remaining_world_time > 0.0 && !loop)
            {
                if (!append_world_chunk(
                        world_offset,
                        remaining_world_time,
                        current_clip_time,
                        0.0))
                {
                    return result;
                }
                world_offset += remaining_world_time;
                remaining_world_time = 0.0;
            }
            else if (reaches_boundary && remaining_world_time > 0.0 && loop)
            {
                if (wrap_count == std::numeric_limits<uint32_t>::max())
                {
                    result.values.clear();
                    result.error = animation_channel_span_error::capacity_exceeded;
                    result.message =
                        "Animation Playback Loop wrap count exceeds uint32 capacity.";
                    return result;
                }
                ++wrap_count;
                current_clip_time =
                    moving_forward ? 0.0 : clip_duration;
            }

            if (world_offset > world_time_duration ||
                !std::isfinite(world_time_start + world_offset))
            {
                result.values.clear();
                result.error = animation_channel_span_error::invalid_time;
                result.message =
                    "Animation Playback World-Time cursor exceeded the Logical Tick.";
                return result;
            }
        }

        result.success = !result.values.empty();
        if (!result.success)
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Playback Scheduler produced no Clip Spans for the Logical Tick.";
        }
        return result;
    }

    animation_channel_span_sample_result evaluate_animation_channel_span(
        const animation_channel_span& span,
        double span_local_u,
        double span_duration_seconds)
    {
        if (span.node_index == asset::invalid_index ||
            (span.path != asset::animation_path::translation &&
                span.path != asset::animation_path::rotation &&
                span.path != asset::animation_path::scale) ||
            (span.interpolation_mode != asset::interpolation::step &&
                span.interpolation_mode != asset::interpolation::linear &&
                span.interpolation_mode != asset::interpolation::cubic_spline))
        {
            return make_span_sample_error(
                animation_channel_span_error::invalid_track,
                "Animation Span metadata contains an invalid Node Index, channel path or interpolation mode.");
        }
        if (!std::isfinite(span.clip_time_start) ||
            !std::isfinite(span.clip_time_delta) ||
            !std::isfinite(span.clip_time_start + span.clip_time_delta) ||
            !std::isfinite(span_local_u) || span_local_u < 0.0 ||
            span_local_u > 1.0 ||
            !std::isfinite(span_duration_seconds) ||
            span_duration_seconds <= 0.0 ||
            !std::isfinite(1.0 / span_duration_seconds))
        {
            return make_span_sample_error(
                animation_channel_span_error::invalid_time,
                "Animation Span evaluation requires finite Clip-Time metadata, u in [0, 1] and a finite positive World-Time duration with a representable reciprocal.");
        }
        for (const asset::vec4& coefficient : span.coefficients)
        {
            if (!finite_asset_vec4(coefficient))
            {
                return make_span_sample_error(
                    animation_channel_span_error::malformed_key,
                    "Animation Span contains a non-finite coefficient.");
            }
        }

        animation_channel_span_sample_result result;
        const double u = span_local_u;
        const double inverse_world_duration = 1.0 / span_duration_seconds;
        const std::size_t component_count =
            span.path == asset::animation_path::rotation ? 4u : 3u;

        if (span.path == asset::animation_path::rotation)
        {
            animation_component_array value = {};
            animation_component_array derivative_u = {};
            if (span.interpolation_mode == asset::interpolation::linear)
            {
                animation_component_array start = {
                    span.coefficients[0].x,
                    span.coefficients[0].y,
                    span.coefficients[0].z,
                    span.coefficients[0].w
                };
                animation_component_array end = {
                    span.coefficients[1].x,
                    span.coefficients[1].y,
                    span.coefficients[1].z,
                    span.coefficients[1].w
                };
                double start_length = 0.0;
                double end_length = 0.0;
                animation_component_array normalized_start = {};
                animation_component_array normalized_end = {};
                if (!normalize_quaternion_double(
                        start, normalized_start, start_length) ||
                    !normalize_quaternion_double(
                        end, normalized_end, end_length))
                {
                    return make_span_sample_error(
                        animation_channel_span_error::invalid_quaternion,
                        "LINEAR rotation Span contains a zero or non-finite endpoint quaternion.");
                }
                double cosine = dot_quaternion(normalized_start, normalized_end);
                if (cosine < 0.0)
                {
                    cosine = -cosine;
                    for (double& component : normalized_end)
                    {
                        component = -component;
                    }
                }
                cosine = std::max(-1.0, std::min(1.0, cosine));
                if (cosine > 0.9995)
                {
                    animation_component_array raw = {};
                    animation_component_array raw_derivative = {};
                    for (std::size_t component = 0u; component < 4u; ++component)
                    {
                        raw[component] =
                            normalized_start[component] * (1.0 - u) +
                            normalized_end[component] * u;
                        raw_derivative[component] =
                            normalized_end[component] - normalized_start[component];
                    }
                    double raw_length = 0.0;
                    if (!normalize_quaternion_double(raw, value, raw_length))
                    {
                        return make_span_sample_error(
                            animation_channel_span_error::invalid_quaternion,
                            "LINEAR rotation Span normalization failed.");
                    }
                    const double projection =
                        dot_quaternion(value, raw_derivative);
                    for (std::size_t component = 0u; component < 4u; ++component)
                    {
                        derivative_u[component] =
                            (raw_derivative[component] -
                                value[component] * projection) / raw_length;
                    }
                }
                else
                {
                    const double angle = std::acos(cosine);
                    const double sine = std::sin(angle);
                    if (!std::isfinite(sine) || std::abs(sine) <= 1.0e-12)
                    {
                        return make_span_sample_error(
                            animation_channel_span_error::invalid_quaternion,
                            "LINEAR rotation Span Slerp denominator is unresolved.");
                    }
                    animation_component_array tangent = {};
                    for (std::size_t component = 0u; component < 4u; ++component)
                    {
                        tangent[component] =
                            (normalized_end[component] -
                                normalized_start[component] * cosine) / sine;
                        value[component] =
                            normalized_start[component] * std::cos(angle * u) +
                            tangent[component] * std::sin(angle * u);
                        derivative_u[component] =
                            angle *
                            (-normalized_start[component] * std::sin(angle * u) +
                                tangent[component] * std::cos(angle * u));
                    }
                }
            }
            else
            {
                animation_component_array raw = {};
                animation_component_array raw_derivative = {};
                const double u_squared = u * u;
                for (std::size_t component = 0u; component < 4u; ++component)
                {
                    const double c0 = component_value(span.coefficients[0], component);
                    const double c1 = component_value(span.coefficients[1], component);
                    const double c2 = component_value(span.coefficients[2], component);
                    const double c3 = component_value(span.coefficients[3], component);
                    raw[component] =
                        c0 + c1 * u + c2 * u_squared + c3 * u_squared * u;
                    raw_derivative[component] =
                        c1 + 2.0 * c2 * u + 3.0 * c3 * u_squared;
                }

                double raw_length = 0.0;
                if (!normalize_quaternion_double(raw, value, raw_length))
                {
                    return make_span_sample_error(
                        animation_channel_span_error::invalid_quaternion,
                        "CUBICSPLINE / STEP rotation Span produced a zero or non-finite quaternion.");
                }
                const double projection =
                    dot_quaternion(value, raw_derivative);
                for (std::size_t component = 0u; component < 4u; ++component)
                {
                    derivative_u[component] =
                        (raw_derivative[component] -
                            value[component] * projection) / raw_length;
                }
            }

            for (std::size_t component = 0u; component < 4u; ++component)
            {
                const double world_derivative =
                    derivative_u[component] * inverse_world_duration;
                if (!std::isfinite(value[component]) ||
                    !std::isfinite(world_derivative) ||
                    std::abs(value[component]) >
                        static_cast<double>(std::numeric_limits<float>::max()) ||
                    std::abs(world_derivative) >
                        static_cast<double>(std::numeric_limits<float>::max()))
                {
                    return make_span_sample_error(
                        animation_channel_span_error::malformed_key,
                        "Animation rotation Span evaluation exceeded finite float range.");
                }
                set_component_value(
                    result.value.value, component,
                    static_cast<float>(value[component]));
                set_component_value(
                    result.value.world_time_derivative, component,
                    static_cast<float>(world_derivative));
            }
        }
        else
        {
            const double u_squared = u * u;
            for (std::size_t component = 0u; component < component_count; ++component)
            {
                const double c0 = component_value(span.coefficients[0], component);
                const double c1 = component_value(span.coefficients[1], component);
                const double c2 = component_value(span.coefficients[2], component);
                const double c3 = component_value(span.coefficients[3], component);
                const double value =
                    c0 + c1 * u + c2 * u_squared + c3 * u_squared * u;
                const double derivative_u =
                    c1 + 2.0 * c2 * u + 3.0 * c3 * u_squared;
                const double world_derivative =
                    derivative_u * inverse_world_duration;
                if (!std::isfinite(value) || !std::isfinite(world_derivative) ||
                    std::abs(value) >
                        static_cast<double>(std::numeric_limits<float>::max()) ||
                    std::abs(world_derivative) >
                        static_cast<double>(std::numeric_limits<float>::max()))
                {
                    return make_span_sample_error(
                        animation_channel_span_error::malformed_key,
                        "Animation channel Span evaluation exceeded finite float range.");
                }
                set_component_value(
                    result.value.value, component, static_cast<float>(value));
                set_component_value(
                    result.value.world_time_derivative, component,
                    static_cast<float>(world_derivative));
            }
        }

        result.success = true;
        return result;
    }


    namespace
    {
        animation_blend_weight_span_build_result make_blend_span_error(
            animation_blend_weight_error error,
            const std::string& message)
        {
            animation_blend_weight_span_build_result result;
            result.error = error;
            result.message = message;
            return result;
        }

        animation_blend_weight_spans_build_result make_blend_spans_error(
            animation_blend_weight_error error,
            const std::string& message)
        {
            animation_blend_weight_spans_build_result result;
            result.error = error;
            result.message = message;
            return result;
        }

        animation_blend_weight_sample_result make_blend_sample_error(
            animation_blend_weight_error error,
            const std::string& message)
        {
            animation_blend_weight_sample_result result;
            result.error = error;
            result.message = message;
            return result;
        }

        animation_component_array blend_curve_coefficients(
            const animation_blend_weight_curve_desc& curve)
        {
            const double duration =
                curve.blend_end_time - curve.blend_start_time;
            const double weight_start = curve.weight_start;
            const double weight_end = curve.weight_end;
            const double tangent_start =
                static_cast<double>(curve.tangent_start) * duration;
            const double tangent_end =
                static_cast<double>(curve.tangent_end) * duration;
            return {
                weight_start,
                tangent_start,
                -3.0 * weight_start + 3.0 * weight_end -
                    2.0 * tangent_start - tangent_end,
                2.0 * weight_start - 2.0 * weight_end +
                    tangent_start + tangent_end
            };
        }

        double evaluate_blend_curve_value(
            const animation_component_array& coefficients,
            double u)
        {
            return ((coefficients[3] * u + coefficients[2]) * u +
                coefficients[1]) * u + coefficients[0];
        }

        bool blend_curve_value_is_valid(
            const animation_component_array& coefficients,
            double u)
        {
            const double value = evaluate_blend_curve_value(coefficients, u);
            return std::isfinite(value) && value >= 0.0 && value <= 1.0;
        }
    }

    bool valid_animation_blend_weight_curve(
        const animation_blend_weight_curve_desc& curve)
    {
        if (!std::isfinite(curve.blend_start_time) ||
            !std::isfinite(curve.blend_end_time) ||
            curve.blend_end_time <= curve.blend_start_time ||
            !std::isfinite(curve.weight_start) ||
            !std::isfinite(curve.weight_end) ||
            curve.weight_start < 0.0f || curve.weight_start > 1.0f ||
            curve.weight_end < 0.0f || curve.weight_end > 1.0f ||
            !std::isfinite(curve.tangent_start) ||
            !std::isfinite(curve.tangent_end))
        {
            return false;
        }

        const animation_component_array coefficients =
            blend_curve_coefficients(curve);
        if (!std::all_of(
                coefficients.begin(), coefficients.end(),
                [](double value) { return std::isfinite(value); }) ||
            !blend_curve_value_is_valid(coefficients, 0.0) ||
            !blend_curve_value_is_valid(coefficients, 1.0))
        {
            return false;
        }

        const double quadratic_a = 3.0 * coefficients[3];
        const double quadratic_b = 2.0 * coefficients[2];
        const double quadratic_c = coefficients[1];
        const double coefficient_scale = std::max(
            1.0,
            std::max(std::abs(quadratic_a),
                std::max(std::abs(quadratic_b), std::abs(quadratic_c))));
        const double linear_epsilon =
            std::numeric_limits<double>::epsilon() *
            coefficient_scale * 64.0;

        if (std::abs(quadratic_a) <= linear_epsilon)
        {
            if (std::abs(quadratic_b) <= linear_epsilon)
            {
                return true;
            }
            const double root = -quadratic_c / quadratic_b;
            return !(root > 0.0 && root < 1.0) ||
                blend_curve_value_is_valid(coefficients, root);
        }

        double discriminant =
            quadratic_b * quadratic_b -
            4.0 * quadratic_a * quadratic_c;
        const double discriminant_scale = std::max(
            1.0,
            quadratic_b * quadratic_b +
                std::abs(4.0 * quadratic_a * quadratic_c));
        const double discriminant_epsilon =
            std::numeric_limits<double>::epsilon() *
            discriminant_scale * 64.0;
        if (discriminant < -discriminant_epsilon)
        {
            return true;
        }
        discriminant = std::max(0.0, discriminant);
        const double square_root = std::sqrt(discriminant);
        const double q = -0.5 *
            (quadratic_b + std::copysign(square_root, quadratic_b));
        if (q == 0.0)
        {
            const double root = -quadratic_b / (2.0 * quadratic_a);
            return !(root > 0.0 && root < 1.0) ||
                blend_curve_value_is_valid(coefficients, root);
        }

        const double root0 = q / quadratic_a;
        const double root1 = quadratic_c / q;
        return (!(root0 > 0.0 && root0 < 1.0) ||
                    blend_curve_value_is_valid(coefficients, root0)) &&
            (!(root1 > 0.0 && root1 < 1.0) ||
                blend_curve_value_is_valid(coefficients, root1));
    }

    animation_blend_weight_span_build_result build_animation_blend_weight_span(
        const animation_blend_weight_curve_desc& curve,
        double world_time_start,
        double world_time_duration)
    {
        if (!valid_animation_blend_weight_curve(curve))
        {
            return make_blend_span_error(
                animation_blend_weight_error::invalid_curve,
                "Animation Blend Weight Curve is invalid or leaves [0, 1] over its complete domain.");
        }
        const double world_time_end =
            world_time_start + world_time_duration;
        if (!std::isfinite(world_time_start) ||
            !std::isfinite(world_time_duration) ||
            world_time_duration <= 0.0 ||
            !std::isfinite(world_time_end) ||
            world_time_end == world_time_start)
        {
            return make_blend_span_error(
                animation_blend_weight_error::invalid_time,
                "Animation Blend Weight Span requires a finite World-Time start and positive representable duration.");
        }

        animation_blend_weight_span candidate;
        candidate.world_time_start = world_time_start;
        candidate.world_time_duration = world_time_duration;
        if (world_time_end <= curve.blend_start_time)
        {
            candidate.coefficients = {
                static_cast<double>(curve.weight_start), 0.0, 0.0, 0.0
            };
        }
        else if (world_time_start >= curve.blend_end_time)
        {
            candidate.coefficients = {
                static_cast<double>(curve.weight_end), 0.0, 0.0, 0.0
            };
        }
        else if (world_time_start < curve.blend_start_time ||
            world_time_end > curve.blend_end_time)
        {
            return make_blend_span_error(
                animation_blend_weight_error::span_crosses_curve_boundary,
                "Animation Blend Weight Span crosses blend_start_time or blend_end_time and must be split by the World Scheduler.");
        }
        else
        {
            const animation_component_array curve_coefficients =
                blend_curve_coefficients(curve);
            const double curve_duration =
                curve.blend_end_time - curve.blend_start_time;
            const double normalized_start =
                (world_time_start - curve.blend_start_time) /
                curve_duration;
            const double normalized_delta =
                world_time_duration / curve_duration;
            const double normalized_start_squared =
                normalized_start * normalized_start;
            const double normalized_delta_squared =
                normalized_delta * normalized_delta;
            const double normalized_delta_cubed =
                normalized_delta_squared * normalized_delta;

            candidate.coefficients[0] =
                evaluate_blend_curve_value(
                    curve_coefficients, normalized_start);
            candidate.coefficients[1] = normalized_delta *
                (curve_coefficients[1] +
                    2.0 * curve_coefficients[2] * normalized_start +
                    3.0 * curve_coefficients[3] * normalized_start_squared);
            candidate.coefficients[2] = normalized_delta_squared *
                (curve_coefficients[2] +
                    3.0 * curve_coefficients[3] * normalized_start);
            candidate.coefficients[3] =
                normalized_delta_cubed * curve_coefficients[3];
        }

        if (!std::all_of(
                candidate.coefficients.begin(),
                candidate.coefficients.end(),
                [](double value) { return std::isfinite(value); }))
        {
            return make_blend_span_error(
                animation_blend_weight_error::invalid_curve,
                "Animation Blend Weight Span coefficients exceed finite double range.");
        }

        animation_blend_weight_span_build_result result;
        result.value = candidate;
        result.success = true;
        return result;
    }

    animation_blend_weight_spans_build_result build_animation_blend_weight_spans(
        const animation_blend_weight_curve_desc& curve,
        double world_time_start,
        double world_time_duration)
    {
        if (!valid_animation_blend_weight_curve(curve))
        {
            return make_blend_spans_error(
                animation_blend_weight_error::invalid_curve,
                "Animation Blend Weight Curve is invalid or leaves [0, 1] over its complete domain.");
        }
        const double world_time_end =
            world_time_start + world_time_duration;
        if (!std::isfinite(world_time_start) ||
            !std::isfinite(world_time_duration) ||
            world_time_duration <= 0.0 ||
            !std::isfinite(world_time_end) ||
            world_time_end == world_time_start)
        {
            return make_blend_spans_error(
                animation_blend_weight_error::invalid_time,
                "Animation Blend Weight scheduling requires a finite positive Logical Tick interval.");
        }

        std::vector<double> boundaries;
        boundaries.reserve(4u);
        boundaries.push_back(world_time_start);
        if (curve.blend_start_time > world_time_start &&
            curve.blend_start_time < world_time_end)
        {
            boundaries.push_back(curve.blend_start_time);
        }
        if (curve.blend_end_time > world_time_start &&
            curve.blend_end_time < world_time_end)
        {
            boundaries.push_back(curve.blend_end_time);
        }
        boundaries.push_back(world_time_end);
        std::sort(boundaries.begin(), boundaries.end());

        animation_blend_weight_spans_build_result result;
        result.values.reserve(boundaries.size() - 1u);
        for (std::size_t index = 0u;
            index + 1u < boundaries.size(); ++index)
        {
            const double span_start = boundaries[index];
            const double span_duration = boundaries[index + 1u] - span_start;
            animation_blend_weight_span_build_result span_result =
                build_animation_blend_weight_span(
                    curve, span_start, span_duration);
            if (!span_result.success)
            {
                result.values.clear();
                result.error = span_result.error;
                result.message = span_result.message;
                return result;
            }
            result.values.push_back(std::move(span_result.value));
        }
        result.success = !result.values.empty();
        if (!result.success)
        {
            result.error = animation_blend_weight_error::invalid_time;
            result.message =
                "Animation Blend Weight Scheduler produced an empty World-Time partition.";
        }
        return result;
    }

    animation_blend_weight_sample_result evaluate_animation_blend_weight_span(
        const animation_blend_weight_span& span,
        double span_local_u)
    {
        if (!std::isfinite(span.world_time_start) ||
            !std::isfinite(span.world_time_duration) ||
            span.world_time_duration <= 0.0 ||
            !std::isfinite(span.world_time_start + span.world_time_duration) ||
            !std::isfinite(span_local_u) ||
            span_local_u < 0.0 || span_local_u > 1.0)
        {
            return make_blend_sample_error(
                animation_blend_weight_error::invalid_time,
                "Animation Blend Weight evaluation requires a finite World-Time interval and u in [0, 1].");
        }
        if (!std::all_of(
                span.coefficients.begin(),
                span.coefficients.end(),
                [](double value) { return std::isfinite(value); }))
        {
            return make_blend_sample_error(
                animation_blend_weight_error::invalid_curve,
                "Animation Blend Weight Span contains a non-finite coefficient.");
        }

        const double u = span_local_u;
        const double u_squared = u * u;
        const double weight =
            ((span.coefficients[3] * u + span.coefficients[2]) * u +
                span.coefficients[1]) * u + span.coefficients[0];
        const double derivative_u =
            span.coefficients[1] +
            2.0 * span.coefficients[2] * u +
            3.0 * span.coefficients[3] * u_squared;
        const double world_derivative =
            derivative_u / span.world_time_duration;
        if (!std::isfinite(weight) || weight < -1.0e-9 || weight > 1.0 + 1.0e-9 ||
            !std::isfinite(world_derivative))
        {
            return make_blend_sample_error(
                animation_blend_weight_error::invalid_curve,
                "Animation Blend Weight evaluation left [0, 1] or produced a non-finite derivative.");
        }

        animation_blend_weight_sample_result result;
        result.value.weight = std::max(0.0, std::min(1.0, weight));
        result.value.world_time_derivative = world_derivative;
        result.success = true;
        return result;
    }

    animation_blend_clip_span_schedule_result schedule_animation_blend_clip_spans(
        const asset::animation_clip& source_clip,
        uint32_t source_node_count,
        double source_clip_time_start,
        double source_playback_speed,
        bool source_loop,
        const asset::animation_clip& target_clip,
        uint32_t target_node_count,
        double target_clip_time_start,
        double target_playback_speed,
        bool target_loop,
        const animation_blend_weight_curve_desc& blend_curve,
        double world_time_start,
        double world_time_duration,
        uint32_t maximum_span_count)
    {
        animation_blend_clip_span_schedule_result result;
        if (!std::isfinite(world_time_start) ||
            !std::isfinite(world_time_duration) ||
            world_time_duration <= 0.0 ||
            !std::isfinite(world_time_start + world_time_duration) ||
            world_time_start + world_time_duration == world_time_start ||
            maximum_span_count == 0u)
        {
            result.error = animation_blend_clip_span_error::invalid_time;
            result.message =
                "Animation Blend Clip scheduling requires one finite positive Logical Tick and a non-zero Span limit.";
            return result;
        }

        animation_clip_span_schedule_result source_schedule =
            schedule_animation_clip_playback_spans(
                source_clip,
                source_node_count,
                source_clip_time_start,
                source_playback_speed,
                source_loop,
                world_time_start,
                world_time_duration,
                maximum_span_count);
        if (!source_schedule.success)
        {
            result.error =
                source_schedule.error ==
                    animation_channel_span_error::capacity_exceeded
                    ? animation_blend_clip_span_error::capacity_exceeded
                    : animation_blend_clip_span_error::invalid_source_clip;
            result.message =
                "Source Animation Clip could not be scheduled: " +
                source_schedule.message;
            return result;
        }

        animation_clip_span_schedule_result target_schedule =
            schedule_animation_clip_playback_spans(
                target_clip,
                target_node_count,
                target_clip_time_start,
                target_playback_speed,
                target_loop,
                world_time_start,
                world_time_duration,
                maximum_span_count);
        if (!target_schedule.success)
        {
            result.error =
                target_schedule.error ==
                    animation_channel_span_error::capacity_exceeded
                    ? animation_blend_clip_span_error::capacity_exceeded
                    : animation_blend_clip_span_error::invalid_target_clip;
            result.message =
                "Target Animation Clip could not be scheduled: " +
                target_schedule.message;
            return result;
        }

        animation_blend_weight_spans_build_result weight_schedule =
            build_animation_blend_weight_spans(
                blend_curve, world_time_start, world_time_duration);
        if (!weight_schedule.success)
        {
            result.error =
                weight_schedule.error == animation_blend_weight_error::invalid_time
                    ? animation_blend_clip_span_error::invalid_time
                    : animation_blend_clip_span_error::invalid_blend_curve;
            result.message =
                "Animation Blend Weight Curve could not be scheduled: " +
                weight_schedule.message;
            return result;
        }

        std::vector<double> boundaries;
        boundaries.reserve(
            2u +
            source_schedule.values.size() * 2u +
            target_schedule.values.size() * 2u +
            weight_schedule.values.size() * 2u);
        const double world_time_end =
            world_time_start + world_time_duration;
        boundaries.push_back(world_time_start);
        boundaries.push_back(world_time_end);

        const auto append_clip_boundaries =
            [&boundaries](const animation_clip_span_schedule_result& schedule)
        {
            for (const scheduled_animation_clip_span& span : schedule.values)
            {
                boundaries.push_back(span.world_time_start);
                boundaries.push_back(
                    span.world_time_start + span.world_time_duration);
            }
        };
        append_clip_boundaries(source_schedule);
        append_clip_boundaries(target_schedule);
        for (const animation_blend_weight_span& span : weight_schedule.values)
        {
            boundaries.push_back(span.world_time_start);
            boundaries.push_back(span.world_time_start + span.world_time_duration);
        }

        std::sort(boundaries.begin(), boundaries.end());
        boundaries.erase(
            std::unique(boundaries.begin(), boundaries.end()),
            boundaries.end());
        if (boundaries.size() < 2u ||
            boundaries.front() != world_time_start ||
            boundaries.back() != world_time_end)
        {
            result.error = animation_blend_clip_span_error::invalid_time;
            result.message =
                "Merged Animation Blend Clip boundaries do not exactly cover the Logical Tick.";
            return result;
        }

        if (boundaries.size() - 1u >
            static_cast<std::size_t>(maximum_span_count) ||
            boundaries.size() - 1u >
            static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))
        {
            result.error = animation_blend_clip_span_error::capacity_exceeded;
            result.message =
                "Merged Animation Blend Clip Span count exceeds the configured or uint32 capacity.";
            return result;
        }

        const auto find_covering_clip_span =
            [](const animation_clip_span_schedule_result& schedule,
                double interval_start,
                double interval_end,
                const scheduled_animation_clip_span*& out_span) -> bool
        {
            out_span = nullptr;
            uint32_t match_count = 0u;
            for (const scheduled_animation_clip_span& candidate : schedule.values)
            {
                const double candidate_end =
                    candidate.world_time_start + candidate.world_time_duration;
                if (candidate.world_time_start <= interval_start &&
                    candidate_end >= interval_end)
                {
                    out_span = &candidate;
                    ++match_count;
                }
            }
            return match_count == 1u;
        };

        const auto find_covering_weight_span =
            [&weight_schedule](
                double interval_start,
                double interval_end,
                const animation_blend_weight_span*& out_span) -> bool
        {
            out_span = nullptr;
            uint32_t match_count = 0u;
            for (const animation_blend_weight_span& candidate :
                weight_schedule.values)
            {
                const double candidate_end =
                    candidate.world_time_start + candidate.world_time_duration;
                if (candidate.world_time_start <= interval_start &&
                    candidate_end >= interval_end)
                {
                    out_span = &candidate;
                    ++match_count;
                }
            }
            return match_count == 1u;
        };

        result.values.reserve(boundaries.size() - 1u);
        for (std::size_t span_index = 0u;
            span_index + 1u < boundaries.size(); ++span_index)
        {
            const double interval_start = boundaries[span_index];
            const double interval_end = boundaries[span_index + 1u];
            const double interval_duration =
                interval_end - interval_start;
            if (!std::isfinite(interval_start) ||
                !std::isfinite(interval_end) ||
                !std::isfinite(interval_duration) ||
                interval_duration <= 0.0 ||
                interval_start < world_time_start ||
                interval_end > world_time_end)
            {
                result.values.clear();
                result.error = animation_blend_clip_span_error::invalid_time;
                result.message =
                    "Merged Animation Blend Clip interval is not a positive ordered subinterval of the Logical Tick.";
                return result;
            }

            const scheduled_animation_clip_span* source_span = nullptr;
            const scheduled_animation_clip_span* target_span = nullptr;
            const animation_blend_weight_span* weight_span = nullptr;
            if (!find_covering_clip_span(
                    source_schedule, interval_start, interval_end, source_span) ||
                !find_covering_clip_span(
                    target_schedule, interval_start, interval_end, target_span) ||
                !find_covering_weight_span(
                    interval_start, interval_end, weight_span) ||
                source_span == nullptr || target_span == nullptr ||
                weight_span == nullptr)
            {
                result.values.clear();
                result.error = animation_blend_clip_span_error::invalid_time;
                result.message =
                    "A merged Animation Blend interval is not covered by exactly one Source, Target and Blend Weight Span.";
                return result;
            }

            const auto rebuild_clip_interval =
                [interval_start, interval_end, interval_duration](
                    const scheduled_animation_clip_span& source,
                    const asset::animation_clip& clip,
                    uint32_t node_count,
                    scheduled_animation_clip_span& output,
                    std::string& diagnostic) -> bool
            {
                const double source_duration = source.world_time_duration;
                if (!std::isfinite(source_duration) || source_duration <= 0.0)
                {
                    diagnostic =
                        "Source scheduled Clip Span has a non-positive World-Time duration.";
                    return false;
                }
                double u0 =
                    (interval_start - source.world_time_start) / source_duration;
                double u1 =
                    (interval_end - source.world_time_start) / source_duration;
                constexpr double local_fraction_tolerance =
                    std::numeric_limits<double>::epsilon() * 64.0;
                if (!std::isfinite(u0) || !std::isfinite(u1) ||
                    u0 < -local_fraction_tolerance ||
                    u0 > 1.0 + local_fraction_tolerance ||
                    u1 < -local_fraction_tolerance ||
                    u1 > 1.0 + local_fraction_tolerance)
                {
                    diagnostic =
                        "Merged Blend interval cannot be mapped into its source Clip Span local coordinate. u0=" +
                        std::to_string(u0) + " u1=" + std::to_string(u1);
                    return false;
                }
                u0 = std::max(0.0, std::min(1.0, u0));
                u1 = std::max(0.0, std::min(1.0, u1));
                if (u1 <= u0)
                {
                    diagnostic =
                        "Merged Blend interval collapsed after local-coordinate rounding. u0=" +
                        std::to_string(u0) + " u1=" + std::to_string(u1);
                    return false;
                }
                const double clip_time_start =
                    source.clip.clip_time_start +
                    source.clip.clip_time_delta * u0;
                const double clip_time_delta =
                    source.clip.clip_time_delta * (u1 - u0);
                if (!std::isfinite(clip_time_start) ||
                    !std::isfinite(clip_time_delta) ||
                    !std::isfinite(clip_time_start + clip_time_delta) ||
                    (source.clip.clip_time_delta != 0.0 &&
                        clip_time_delta == 0.0))
                {
                    diagnostic =
                        "Merged Blend interval cannot represent its source Clip-Time partition.";
                    return false;
                }

                animation_clip_spans_build_result rebuilt =
                    build_animation_clip_spans(
                        clip, node_count, clip_time_start, clip_time_delta);
                if (!rebuilt.success || rebuilt.values.size() != 1u)
                {
                    diagnostic =
                        "Merged Blend interval could not rebuild one key-bounded Clip Span: " +
                        rebuilt.message;
                    return false;
                }

                output.clip = std::move(rebuilt.values.front());
                output.world_time_start = interval_start;
                output.world_time_duration = interval_duration;
                return true;
            };

            animation_blend_clip_span scheduled;
            scheduled.span_index = static_cast<uint32_t>(span_index);
            scheduled.world_time_start = interval_start;
            scheduled.world_time_duration = interval_duration;
            std::string diagnostic;
            if (!rebuild_clip_interval(
                    *source_span, source_clip, source_node_count,
                    scheduled.source, diagnostic))
            {
                result.values.clear();
                result.error = animation_blend_clip_span_error::invalid_source_clip;
                result.message =
                    "Source Clip could not be synchronized to the merged Blend interval: " +
                    diagnostic;
                return result;
            }
            if (!rebuild_clip_interval(
                    *target_span, target_clip, target_node_count,
                    scheduled.target, diagnostic))
            {
                result.values.clear();
                result.error = animation_blend_clip_span_error::invalid_target_clip;
                result.message =
                    "Target Clip could not be synchronized to the merged Blend interval: " +
                    diagnostic;
                return result;
            }
            scheduled.source.clip.span_index = scheduled.span_index;
            scheduled.target.clip.span_index = scheduled.span_index;

            const double weight_source_duration =
                weight_span->world_time_duration;
            double weight_u0 =
                (interval_start - weight_span->world_time_start) /
                weight_source_duration;
            double weight_u1 =
                (interval_end - weight_span->world_time_start) /
                weight_source_duration;
            const double local_fraction_tolerance =
                std::numeric_limits<double>::epsilon() * 64.0;
            if (!std::isfinite(weight_u0) ||
                !std::isfinite(weight_u1) ||
                weight_u0 < -local_fraction_tolerance ||
                weight_u0 > 1.0 + local_fraction_tolerance ||
                weight_u1 < -local_fraction_tolerance ||
                weight_u1 > 1.0 + local_fraction_tolerance)
            {
                result.values.clear();
                result.error = animation_blend_clip_span_error::invalid_time;
                result.message =
                    "Merged Blend interval cannot be mapped into its Hermite Weight Span local coordinate.";
                return result;
            }
            weight_u0 = std::max(0.0, std::min(1.0, weight_u0));
            weight_u1 = std::max(0.0, std::min(1.0, weight_u1));
            const double weight_u_delta = weight_u1 - weight_u0;
            if (!std::isfinite(weight_u_delta) || weight_u_delta <= 0.0)
            {
                result.values.clear();
                result.error = animation_blend_clip_span_error::invalid_time;
                result.message =
                    "Merged Blend interval collapsed after Hermite Weight Span local-coordinate rounding.";
                return result;
            }

            const double u0_squared = weight_u0 * weight_u0;
            const double u_delta_squared = weight_u_delta * weight_u_delta;
            const double c0 = weight_span->coefficients[0];
            const double c1 = weight_span->coefficients[1];
            const double c2 = weight_span->coefficients[2];
            const double c3 = weight_span->coefficients[3];
            scheduled.blend_weight.world_time_start = interval_start;
            scheduled.blend_weight.world_time_duration = interval_duration;
            scheduled.blend_weight.coefficients = {
                ((c3 * weight_u0 + c2) * weight_u0 + c1) *
                    weight_u0 + c0,
                weight_u_delta *
                    (c1 + 2.0 * c2 * weight_u0 +
                        3.0 * c3 * u0_squared),
                u_delta_squared *
                    (c2 + 3.0 * c3 * weight_u0),
                u_delta_squared * weight_u_delta * c3
            };
            if (!std::all_of(
                    scheduled.blend_weight.coefficients.begin(),
                    scheduled.blend_weight.coefficients.end(),
                    [](double value) { return std::isfinite(value); }))
            {
                result.values.clear();
                result.error = animation_blend_clip_span_error::invalid_blend_curve;
                result.message =
                    "Merged Blend Weight coefficient reparameterization produced a non-finite value.";
                return result;
            }

            result.values.push_back(std::move(scheduled));
        }

        result.success = !result.values.empty();
        if (!result.success)
        {
            result.error = animation_blend_clip_span_error::invalid_time;
            result.message =
                "Animation Blend Clip Scheduler produced an empty synchronized partition.";
        }
        return result;
    }


    namespace
    {
        asset::mat4 zero_matrix()
        {
            asset::mat4 result;
            result.value.fill(0.0f);
            return result;
        }

        bool finite_matrix(const asset::mat4& matrix)
        {
            return std::all_of(
                matrix.value.begin(), matrix.value.end(),
                [](float value) { return std::isfinite(value); });
        }

        bool multiply_matrix(
            const asset::mat4& left,
            const asset::mat4& right,
            asset::mat4& output)
        {
            output = zero_matrix();
            for (std::size_t column = 0u; column < 4u; ++column)
            {
                for (std::size_t row = 0u; row < 4u; ++row)
                {
                    double value = 0.0;
                    for (std::size_t k = 0u; k < 4u; ++k)
                    {
                        value +=
                            static_cast<double>(left.value[k * 4u + row]) *
                            static_cast<double>(right.value[column * 4u + k]);
                    }
                    if (!std::isfinite(value) ||
                        std::abs(value) >
                            static_cast<double>(std::numeric_limits<float>::max()))
                    {
                        return false;
                    }
                    output.value[column * 4u + row] =
                        static_cast<float>(value);
                }
            }
            return finite_matrix(output);
        }

        bool add_matrix(
            const asset::mat4& left,
            const asset::mat4& right,
            asset::mat4& output)
        {
            output = zero_matrix();
            for (std::size_t index = 0u; index < 16u; ++index)
            {
                const double value =
                    static_cast<double>(left.value[index]) +
                    static_cast<double>(right.value[index]);
                if (!std::isfinite(value) ||
                    std::abs(value) >
                        static_cast<double>(std::numeric_limits<float>::max()))
                {
                    return false;
                }
                output.value[index] = static_cast<float>(value);
            }
            return true;
        }

        bool is_affine_matrix(const asset::mat4& matrix)
        {
            constexpr float affine_tolerance = 1.0e-5f;
            return finite_matrix(matrix) &&
                std::abs(matrix.value[3]) <= affine_tolerance &&
                std::abs(matrix.value[7]) <= affine_tolerance &&
                std::abs(matrix.value[11]) <= affine_tolerance &&
                std::abs(matrix.value[15] - 1.0f) <= affine_tolerance;
        }

        bool invert_affine_matrix(
            const asset::mat4& input,
            asset::mat4& output)
        {
            if (!is_affine_matrix(input))
            {
                return false;
            }

            const double a = input.value[0];
            const double b = input.value[4];
            const double c = input.value[8];
            const double d = input.value[1];
            const double e = input.value[5];
            const double f = input.value[9];
            const double g = input.value[2];
            const double h = input.value[6];
            const double i = input.value[10];
            const double determinant =
                a * (e * i - f * h) -
                b * (d * i - f * g) +
                c * (d * h - e * g);
            const double scale = std::max({
                std::abs(a), std::abs(b), std::abs(c),
                std::abs(d), std::abs(e), std::abs(f),
                std::abs(g), std::abs(h), std::abs(i)
            });
            const double determinant_tolerance =
                std::numeric_limits<double>::epsilon() *
                std::max(1.0, scale * scale * scale) * 128.0;
            if (!std::isfinite(determinant) ||
                !std::isfinite(scale) ||
                scale <= 0.0 ||
                std::abs(determinant) <= determinant_tolerance)
            {
                return false;
            }

            const double inverse_determinant = 1.0 / determinant;
            const double inverse_rows[3][3] = {
                {
                    (e * i - f * h) * inverse_determinant,
                    (c * h - b * i) * inverse_determinant,
                    (b * f - c * e) * inverse_determinant
                },
                {
                    (f * g - d * i) * inverse_determinant,
                    (a * i - c * g) * inverse_determinant,
                    (c * d - a * f) * inverse_determinant
                },
                {
                    (d * h - e * g) * inverse_determinant,
                    (b * g - a * h) * inverse_determinant,
                    (a * e - b * d) * inverse_determinant
                }
            };

            output = asset::mat4{};
            for (std::size_t row = 0u; row < 3u; ++row)
            {
                for (std::size_t column = 0u; column < 3u; ++column)
                {
                    const double value = inverse_rows[row][column];
                    if (!std::isfinite(value) ||
                        std::abs(value) >
                            static_cast<double>(std::numeric_limits<float>::max()))
                    {
                        return false;
                    }
                    output.value[column * 4u + row] =
                        static_cast<float>(value);
                }
            }

            for (std::size_t row = 0u; row < 3u; ++row)
            {
                const double inverse_translation =
                    -(inverse_rows[row][0] * input.value[12] +
                        inverse_rows[row][1] * input.value[13] +
                        inverse_rows[row][2] * input.value[14]);
                if (!std::isfinite(inverse_translation) ||
                    std::abs(inverse_translation) >
                        static_cast<double>(std::numeric_limits<float>::max()))
                {
                    return false;
                }
                output.value[12u + row] =
                    static_cast<float>(inverse_translation);
            }
            return finite_matrix(output);
        }

        animation_hierarchy_sample_result make_hierarchy_error(
            animation_hierarchy_error error,
            const std::string& message)
        {
            animation_hierarchy_sample_result result;
            result.error = error;
            result.message = message;
            return result;
        }

        struct hierarchy_node_trs
        {
            asset::vec3 translation;
            asset::vec3 scale = { 1.0f, 1.0f, 1.0f };
            asset::vec4 rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
            asset::vec3 translation_velocity;
            asset::vec3 scale_velocity;
            asset::vec4 rotation_velocity;
            bool has_translation_channel = false;
            bool has_rotation_channel = false;
            bool has_scale_channel = false;
            bool uses_matrix = false;
            asset::mat4 static_matrix;
        };

        bool finite_vec3(const asset::vec3& value)
        {
            return std::isfinite(value.x) &&
                std::isfinite(value.y) &&
                std::isfinite(value.z);
        }

        bool finite_vec4(const asset::vec4& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) &&
                std::isfinite(value.z) && std::isfinite(value.w);
        }

        bool build_trs_matrix_pair(
            const hierarchy_node_trs& trs,
            asset::mat4& out_local,
            asset::mat4& out_local_derivative)
        {
            if (!finite_vec3(trs.translation) ||
                !finite_vec3(trs.translation_velocity) ||
                !finite_vec3(trs.scale) ||
                !finite_vec3(trs.scale_velocity) ||
                !finite_vec4(trs.rotation) ||
                !finite_vec4(trs.rotation_velocity) ||
                trs.scale.x <= 0.0f ||
                trs.scale.y <= 0.0f ||
                trs.scale.z <= 0.0f)
            {
                return false;
            }

            animation_component_array raw_rotation = {
                trs.rotation.x, trs.rotation.y,
                trs.rotation.z, trs.rotation.w
            };
            animation_component_array normalized_rotation = {};
            double rotation_length = 0.0;
            if (!normalize_quaternion_double(
                    raw_rotation, normalized_rotation, rotation_length))
            {
                return false;
            }

            animation_component_array raw_rotation_derivative = {
                trs.rotation_velocity.x,
                trs.rotation_velocity.y,
                trs.rotation_velocity.z,
                trs.rotation_velocity.w
            };
            const double derivative_projection =
                dot_quaternion(
                    normalized_rotation, raw_rotation_derivative);
            animation_component_array normalized_rotation_derivative = {};
            for (std::size_t component = 0u; component < 4u; ++component)
            {
                normalized_rotation_derivative[component] =
                    (raw_rotation_derivative[component] -
                        normalized_rotation[component] *
                            derivative_projection) / rotation_length;
                if (!std::isfinite(normalized_rotation_derivative[component]))
                {
                    return false;
                }
            }

            const double x = normalized_rotation[0];
            const double y = normalized_rotation[1];
            const double z = normalized_rotation[2];
            const double w = normalized_rotation[3];
            const double xd = normalized_rotation_derivative[0];
            const double yd = normalized_rotation_derivative[1];
            const double zd = normalized_rotation_derivative[2];
            const double wd = normalized_rotation_derivative[3];

            const double rotation[3][3] = {
                {
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w)
                },
                {
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w)
                },
                {
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y)
                }
            };
            const double rotation_derivative[3][3] = {
                {
                    -4.0 * (y * yd + z * zd),
                    2.0 * (xd * y + x * yd - zd * w - z * wd),
                    2.0 * (xd * z + x * zd + yd * w + y * wd)
                },
                {
                    2.0 * (xd * y + x * yd + zd * w + z * wd),
                    -4.0 * (x * xd + z * zd),
                    2.0 * (yd * z + y * zd - xd * w - x * wd)
                },
                {
                    2.0 * (xd * z + x * zd - yd * w - y * wd),
                    2.0 * (yd * z + y * zd + xd * w + x * wd),
                    -4.0 * (x * xd + y * yd)
                }
            };

            const double scales[3] = {
                trs.scale.x, trs.scale.y, trs.scale.z
            };
            const double scale_derivatives[3] = {
                trs.scale_velocity.x,
                trs.scale_velocity.y,
                trs.scale_velocity.z
            };
            const double translations[3] = {
                trs.translation.x,
                trs.translation.y,
                trs.translation.z
            };
            const double translation_derivatives[3] = {
                trs.translation_velocity.x,
                trs.translation_velocity.y,
                trs.translation_velocity.z
            };

            out_local = zero_matrix();
            out_local_derivative = zero_matrix();
            for (std::size_t row = 0u; row < 3u; ++row)
            {
                for (std::size_t column = 0u; column < 3u; ++column)
                {
                    const double value =
                        rotation[row][column] * scales[column];
                    const double derivative =
                        rotation_derivative[row][column] * scales[column] +
                        rotation[row][column] *
                            scale_derivatives[column];
                    if (!std::isfinite(value) ||
                        !std::isfinite(derivative) ||
                        std::abs(value) >
                            static_cast<double>(std::numeric_limits<float>::max()) ||
                        std::abs(derivative) >
                            static_cast<double>(std::numeric_limits<float>::max()))
                    {
                        return false;
                    }
                    const std::size_t matrix_index = column * 4u + row;
                    out_local.value[matrix_index] = static_cast<float>(value);
                    out_local_derivative.value[matrix_index] =
                        static_cast<float>(derivative);
                }
                if (!std::isfinite(translations[row]) ||
                    !std::isfinite(translation_derivatives[row]) ||
                    std::abs(translations[row]) >
                        static_cast<double>(std::numeric_limits<float>::max()) ||
                    std::abs(translation_derivatives[row]) >
                        static_cast<double>(std::numeric_limits<float>::max()))
                {
                    return false;
                }
                out_local.value[12u + row] =
                    static_cast<float>(translations[row]);
                out_local_derivative.value[12u + row] =
                    static_cast<float>(translation_derivatives[row]);
            }
            out_local.value[15] = 1.0f;
            return finite_matrix(out_local) &&
                finite_matrix(out_local_derivative);
        }

        animation_hierarchy_sample_result build_animation_hierarchy_from_node_trs(
            const asset::model& model,
            const std::vector<hierarchy_node_trs>& node_trs)
        {
            const std::size_t node_count = model.nodes.size();
            if (node_count == 0u || node_trs.size() != node_count)
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_model,
                    "Animation Hierarchy builder requires one local transform for every Asset Node.");
            }

        std::vector<std::vector<uint32_t>> children(node_count);
        std::vector<uint32_t> indegree(node_count, 0u);
        std::vector<bool> child_seen(node_count, false);
        for (uint32_t parent_index = 0u;
            parent_index < node_count; ++parent_index)
        {
            const asset::node& parent = model.nodes[parent_index];
            for (uint32_t child_index : parent.children)
            {
                if (child_index >= node_count ||
                    child_index == parent_index ||
                    model.nodes[child_index].parent_index != parent_index ||
                    child_seen[child_index])
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_hierarchy,
                        "Asset Node Children contain an invalid, duplicated or inconsistent parent reference.");
                }
                child_seen[child_index] = true;
                children[parent_index].push_back(child_index);
            }
        }

        std::vector<uint32_t> topological_order;
        topological_order.reserve(node_count);
        std::vector<uint32_t> ready;
        ready.reserve(node_count);
        for (uint32_t node_index = 0u; node_index < node_count; ++node_index)
        {
            const uint32_t parent_index =
                model.nodes[node_index].parent_index;
            if (parent_index == asset::invalid_index)
            {
                ready.push_back(node_index);
                continue;
            }
            if (parent_index >= node_count ||
                parent_index == node_index ||
                !child_seen[node_index])
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_hierarchy,
                    "Asset Node Parent Index is invalid or does not agree with its parent's Child list.");
            }
            indegree[node_index] = 1u;
        }

        std::size_t ready_cursor = 0u;
        while (ready_cursor < ready.size())
        {
            const uint32_t node_index = ready[ready_cursor++];
            topological_order.push_back(node_index);
            for (uint32_t child_index : children[node_index])
            {
                if (indegree[child_index] != 1u)
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_hierarchy,
                        "Asset Node hierarchy contains an invalid indegree transition.");
                }
                indegree[child_index] = 0u;
                ready.push_back(child_index);
            }
        }
        if (topological_order.size() != node_count)
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_hierarchy,
                "Asset Node hierarchy contains a parent cycle or an unreachable Node.");
        }

        animation_hierarchy_sample result_value;
        result_value.local_transforms.resize(node_count);
        result_value.local_matrices.resize(node_count);
        result_value.local_matrix_derivatives.resize(node_count);
        result_value.global_matrices.resize(node_count);
        result_value.global_matrix_derivatives.resize(node_count);

        for (uint32_t node_index : topological_order)
        {
            const asset::node& node = model.nodes[node_index];
            const hierarchy_node_trs& trs = node_trs[node_index];
            animation_node_local_transform local_transform;
            local_transform.uses_matrix = node.uses_matrix;
            local_transform.matrix = node.local_matrix;
            local_transform.translation = trs.translation;
            local_transform.rotation = trs.rotation;
            local_transform.scale = trs.scale;
            local_transform.translation_velocity = trs.translation_velocity;
            local_transform.rotation_velocity = trs.rotation_velocity;
            local_transform.scale_velocity = trs.scale_velocity;
            result_value.local_transforms[node_index] = local_transform;
            asset::mat4 local_matrix;
            asset::mat4 local_derivative = zero_matrix();
            if (node.uses_matrix)
            {
                if (node_trs[node_index].has_translation_channel ||
                    node_trs[node_index].has_rotation_channel ||
                    node_trs[node_index].has_scale_channel)
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_transform,
                        "Animated TRS Channels cannot target an Asset Node stored as a static Matrix.");
                }
                local_matrix = node.local_matrix;
            }
            else if (!build_trs_matrix_pair(
                    node_trs[node_index],
                    local_matrix,
                    local_derivative))
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_transform,
                    "Asset Node Rest / Animated TRS produced a non-finite transform or non-positive Scale.");
            }

            result_value.local_matrices[node_index] = local_matrix;
            result_value.local_matrix_derivatives[node_index] =
                local_derivative;

            if (node.parent_index == asset::invalid_index)
            {
                result_value.global_matrices[node_index] = local_matrix;
                result_value.global_matrix_derivatives[node_index] =
                    local_derivative;
            }
            else
            {
                const asset::mat4& parent_global =
                    result_value.global_matrices[node.parent_index];
                const asset::mat4& parent_derivative =
                    result_value.global_matrix_derivatives[node.parent_index];
                asset::mat4 parent_times_local;
                asset::mat4 derivative_parent_times_local;
                asset::mat4 parent_times_local_derivative;
                asset::mat4 global_derivative;
                if (!multiply_matrix(
                        parent_global, local_matrix, parent_times_local) ||
                    !multiply_matrix(
                        parent_derivative, local_matrix,
                        derivative_parent_times_local) ||
                    !multiply_matrix(
                        parent_global, local_derivative,
                        parent_times_local_derivative) ||
                    !add_matrix(
                        derivative_parent_times_local,
                        parent_times_local_derivative,
                        global_derivative))
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_transform,
                        "Parent / Local Node Matrix multiplication or analytic hierarchy derivative overflowed.");
                }
                result_value.global_matrices[node_index] =
                    parent_times_local;
                result_value.global_matrix_derivatives[node_index] =
                    global_derivative;
            }
        }

        for (uint32_t mesh_node_index = 0u;
            mesh_node_index < node_count; ++mesh_node_index)
        {
            const asset::node& mesh_node = model.nodes[mesh_node_index];
            if (mesh_node.skin_index == asset::invalid_index)
            {
                continue;
            }
            if (mesh_node.mesh_index >= model.meshes.size() ||
                mesh_node.skin_index >= model.skins.size())
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_skin,
                    "A Node with a Skin Index must reference valid Mesh and Skin Asset records.");
            }

            const asset::skin& skin = model.skins[mesh_node.skin_index];
            if (skin.joints.empty() ||
                (!skin.inverse_bind_matrices.empty() &&
                    skin.inverse_bind_matrices.size() != skin.joints.size()))
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_skin,
                    "Skin Joint and Inverse Bind Matrix arrays must be non-empty and either both match or omit the optional Inverse Bind array.");
            }

            asset::mat4 inverse_mesh_global;
            if (!invert_affine_matrix(
                    result_value.global_matrices[mesh_node_index],
                    inverse_mesh_global))
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::singular_mesh_node,
                    "Mesh Node Global Matrix is singular and cannot define Skin Matrix space.");
            }

            asset::mat4 inverse_mesh_derivative_left;
            asset::mat4 inverse_mesh_derivative_right;
            asset::mat4 inverse_mesh_derivative;
            asset::mat4 negative_inverse_mesh_derivative = zero_matrix();
            if (!multiply_matrix(
                    inverse_mesh_global,
                    result_value.global_matrix_derivatives[mesh_node_index],
                    inverse_mesh_derivative_left) ||
                !multiply_matrix(
                    inverse_mesh_derivative_left,
                    inverse_mesh_global,
                    inverse_mesh_derivative_right))
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_skin,
                    "Mesh Node inverse Matrix derivative multiplication overflowed.");
            }
            for (std::size_t matrix_index = 0u; matrix_index < 16u; ++matrix_index)
            {
                negative_inverse_mesh_derivative.value[matrix_index] =
                    -inverse_mesh_derivative_right.value[matrix_index];
            }
            inverse_mesh_derivative =
                negative_inverse_mesh_derivative;

            animation_skin_pose skin_pose;
            skin_pose.mesh_node_index = mesh_node_index;
            skin_pose.skin_index = mesh_node.skin_index;
            skin_pose.skin_matrices.reserve(skin.joints.size());
            skin_pose.skin_matrix_derivatives.reserve(skin.joints.size());
            for (std::size_t joint_slot = 0u;
                joint_slot < skin.joints.size(); ++joint_slot)
            {
                const uint32_t joint_node_index = skin.joints[joint_slot];
                if (joint_node_index >= node_count)
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_skin,
                        "Skin references a Joint Node outside the Asset Model.");
                }

                const asset::mat4 inverse_bind =
                    skin.inverse_bind_matrices.empty()
                        ? asset::mat4{}
                        : skin.inverse_bind_matrices[joint_slot];
                if (!is_affine_matrix(inverse_bind))
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_skin,
                        "Skin Inverse Bind Matrix is non-finite or non-affine.");
                }

                asset::mat4 inverse_mesh_times_joint;
                asset::mat4 skin_matrix;
                asset::mat4 inverse_mesh_derivative_times_joint;
                asset::mat4 inverse_mesh_times_joint_derivative;
                asset::mat4 skin_matrix_derivative_pre_bind;
                asset::mat4 skin_matrix_derivative;
                asset::mat4 skin_matrix_derivative_left;
                if (!multiply_matrix(
                        inverse_mesh_global,
                        result_value.global_matrices[joint_node_index],
                        inverse_mesh_times_joint) ||
                    !multiply_matrix(
                        inverse_mesh_times_joint,
                        inverse_bind,
                        skin_matrix) ||
                    !multiply_matrix(
                        inverse_mesh_derivative,
                        result_value.global_matrices[joint_node_index],
                        inverse_mesh_derivative_times_joint) ||
                    !multiply_matrix(
                        inverse_mesh_global,
                        result_value.global_matrix_derivatives[joint_node_index],
                        inverse_mesh_times_joint_derivative) ||
                    !add_matrix(
                        inverse_mesh_derivative_times_joint,
                        inverse_mesh_times_joint_derivative,
                        skin_matrix_derivative_pre_bind) ||
                    !multiply_matrix(
                        skin_matrix_derivative_pre_bind,
                        inverse_bind,
                        skin_matrix_derivative))
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_skin,
                        "Skin Matrix or analytic Skin Matrix derivative calculation overflowed.");
                }

                skin_pose.skin_matrices.push_back(skin_matrix);
                skin_pose.skin_matrix_derivatives.push_back(
                    skin_matrix_derivative);
            }
            result_value.skin_poses.push_back(std::move(skin_pose));
        }


            animation_hierarchy_sample_result result;
            result.value = std::move(result_value);
            result.success = true;
            return result;
        }

    }

    animation_hierarchy_sample_result evaluate_animation_clip_hierarchy(
        const asset::model& model,
        const asset::animation_clip& clip,
        const animation_clip_span& span,
        double span_local_u,
        double span_world_duration)
    {
        const std::size_t node_count = model.nodes.size();
        if (node_count == 0u ||
            node_count >= static_cast<std::size_t>(asset::invalid_index) ||
            clip.tracks.empty() ||
            clip.tracks.size() != span.channels.size())
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_model,
                "Animation Hierarchy evaluation requires a non-empty Model and a complete Span channel for every Asset Track.");
        }
        if (!std::isfinite(span_local_u) ||
            span_local_u < 0.0 || span_local_u > 1.0 ||
            !std::isfinite(span_world_duration) ||
            span_world_duration <= 0.0)
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_clip_span,
                "Animation Hierarchy evaluation requires a Span-local coordinate in [0, 1] and a finite positive World-Time duration.");
        }

        std::vector<hierarchy_node_trs> node_trs(node_count);
        std::vector<std::array<bool, 3>> channel_seen(node_count, { false, false, false });
        for (std::size_t node_index = 0u; node_index < node_count; ++node_index)
        {
            const asset::node& node = model.nodes[node_index];
            hierarchy_node_trs& trs = node_trs[node_index];
            trs.translation = node.translation;
            trs.scale = node.scale;
            trs.rotation = node.rotation;
            trs.uses_matrix = node.uses_matrix;
            trs.static_matrix = node.local_matrix;
            if (node.uses_matrix && !is_affine_matrix(node.local_matrix))
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_transform,
                    "A static Asset Node Matrix is non-finite or non-affine.");
            }
        }

        for (std::size_t channel_index = 0u;
            channel_index < span.channels.size(); ++channel_index)
        {
            const animation_channel_span& channel =
                span.channels[channel_index];
            const asset::animation_track& track = clip.tracks[channel_index];
            if (channel.track_index != channel_index ||
                channel.node_index >= node_count ||
                channel.node_index != track.node_index ||
                channel.path != track.path ||
                model.nodes[channel.node_index].uses_matrix ||
                channel.clip_time_start != span.clip_time_start ||
                channel.clip_time_delta != span.clip_time_delta)
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_channel,
                    "Animation Span channel ordering, Node / Path mapping, common Clip-Time interval or static Matrix animation binding is invalid.");
            }

            std::size_t channel_slot = 0u;
            hierarchy_node_trs& trs = node_trs[channel.node_index];
            if (channel.path == asset::animation_path::translation)
            {
                channel_slot = 0u;
                trs.has_translation_channel = true;
            }
            else if (channel.path == asset::animation_path::rotation)
            {
                channel_slot = 1u;
                trs.has_rotation_channel = true;
            }
            else if (channel.path == asset::animation_path::scale)
            {
                channel_slot = 2u;
                trs.has_scale_channel = true;
            }
            else
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_channel,
                    "Animation Span contains an unsupported Node Path.");
            }
            if (channel_seen[channel.node_index][channel_slot])
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_channel,
                    "Animation Span contains duplicate channels for one Node / Path pair.");
            }
            channel_seen[channel.node_index][channel_slot] = true;

            const animation_channel_span_sample_result sample =
                evaluate_animation_channel_span(
                    channel, span_local_u, span_world_duration);
            if (!sample.success)
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_channel,
                    "Animation Channel evaluation failed: " + sample.message);
            }

            if (channel.path == asset::animation_path::translation)
            {
                trs.translation = {
                    sample.value.value.x,
                    sample.value.value.y,
                    sample.value.value.z
                };
                trs.translation_velocity = {
                    sample.value.world_time_derivative.x,
                    sample.value.world_time_derivative.y,
                    sample.value.world_time_derivative.z
                };
            }
            else if (channel.path == asset::animation_path::scale)
            {
                trs.scale = {
                    sample.value.value.x,
                    sample.value.value.y,
                    sample.value.value.z
                };
                trs.scale_velocity = {
                    sample.value.world_time_derivative.x,
                    sample.value.world_time_derivative.y,
                    sample.value.world_time_derivative.z
                };
            }
            else
            {
                trs.rotation = sample.value.value;
                trs.rotation_velocity =
                    sample.value.world_time_derivative;
            }
        }

        return build_animation_hierarchy_from_node_trs(model, node_trs);
    }



    animation_hierarchy_sample_result evaluate_animation_blend_clip_hierarchy(
        const asset::model& model,
        const asset::animation_clip& source_clip,
        const asset::animation_clip& target_clip,
        const animation_blend_clip_span& span,
        double span_local_u)
    {
        if (!std::isfinite(span_local_u) ||
            span_local_u < 0.0 || span_local_u > 1.0 ||
            !std::isfinite(span.world_time_start) ||
            !std::isfinite(span.world_time_duration) ||
            span.world_time_duration <= 0.0 ||
            !std::isfinite(span.world_time_start + span.world_time_duration) ||
            span.world_time_start + span.world_time_duration ==
                span.world_time_start)
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_clip_span,
                "Animation Blend Pose evaluation requires one finite positive synchronized World-Time Span and u in [0, 1].");
        }

        const double world_time_end =
            span.world_time_start + span.world_time_duration;
        const auto matches_world_span =
            [&span, world_time_end](double world_start, double duration)
        {
            return std::isfinite(world_start) &&
                std::isfinite(duration) &&
                duration > 0.0 &&
                world_start == span.world_time_start &&
                duration == span.world_time_duration &&
                world_start + duration == world_time_end;
        };
        if (!matches_world_span(
                span.source.world_time_start,
                span.source.world_time_duration) ||
            !matches_world_span(
                span.target.world_time_start,
                span.target.world_time_duration) ||
            !matches_world_span(
                span.blend_weight.world_time_start,
                span.blend_weight.world_time_duration))
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_blend,
                "Source, Target and Hermite Blend Weight must cover the exact same World-Time interval.");
        }

        const animation_hierarchy_sample_result source =
            evaluate_animation_clip_hierarchy(
                model,
                source_clip,
                span.source.clip,
                span_local_u,
                span.world_time_duration);
        if (!source.success)
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_blend,
                "Source Clip Pose evaluation failed: " + source.message);
        }

        const animation_hierarchy_sample_result target =
            evaluate_animation_clip_hierarchy(
                model,
                target_clip,
                span.target.clip,
                span_local_u,
                span.world_time_duration);
        if (!target.success)
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_blend,
                "Target Clip Pose evaluation failed: " + target.message);
        }

        const animation_blend_weight_sample_result blend_weight =
            evaluate_animation_blend_weight_span(
                span.blend_weight, span_local_u);
        if (!blend_weight.success ||
            !std::isfinite(blend_weight.value.weight) ||
            blend_weight.value.weight < 0.0 ||
            blend_weight.value.weight > 1.0 ||
            !std::isfinite(blend_weight.value.world_time_derivative))
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_blend,
                "Hermite Blend Weight evaluation failed: " + blend_weight.message);
        }

        const std::size_t node_count = model.nodes.size();
        if (source.value.local_transforms.size() != node_count ||
            target.value.local_transforms.size() != node_count)
        {
            return make_hierarchy_error(
                animation_hierarchy_error::invalid_model,
                "Source / Target Pose evaluators did not return one Local Transform per Asset Node.");
        }

        const double weight = blend_weight.value.weight;
        const double weight_derivative =
            blend_weight.value.world_time_derivative;
        std::vector<hierarchy_node_trs> blended_transforms(node_count);
        for (std::size_t node_index = 0u;
            node_index < node_count; ++node_index)
        {
            const animation_node_local_transform& source_node =
                source.value.local_transforms[node_index];
            const animation_node_local_transform& target_node =
                target.value.local_transforms[node_index];
            hierarchy_node_trs& blended =
                blended_transforms[node_index];

            if (source_node.uses_matrix != target_node.uses_matrix)
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_transform,
                    "Source / Target Pose disagree on whether an Asset Node uses a static Matrix.");
            }
            if (source_node.uses_matrix)
            {
                for (std::size_t matrix_index = 0u;
                    matrix_index < 16u; ++matrix_index)
                {
                    if (source_node.matrix.value[matrix_index] !=
                        target_node.matrix.value[matrix_index])
                    {
                        return make_hierarchy_error(
                            animation_hierarchy_error::invalid_transform,
                            "Source / Target Pose static Matrix Nodes must preserve the same immutable Asset Matrix.");
                    }
                }
                blended.uses_matrix = true;
                blended.static_matrix = source_node.matrix;
                continue;
            }

            const double source_translation[3] = {
                source_node.translation.x,
                source_node.translation.y,
                source_node.translation.z
            };
            const double target_translation[3] = {
                target_node.translation.x,
                target_node.translation.y,
                target_node.translation.z
            };
            const double source_translation_velocity[3] = {
                source_node.translation_velocity.x,
                source_node.translation_velocity.y,
                source_node.translation_velocity.z
            };
            const double target_translation_velocity[3] = {
                target_node.translation_velocity.x,
                target_node.translation_velocity.y,
                target_node.translation_velocity.z
            };
            const double source_scale[3] = {
                source_node.scale.x,
                source_node.scale.y,
                source_node.scale.z
            };
            const double target_scale[3] = {
                target_node.scale.x,
                target_node.scale.y,
                target_node.scale.z
            };
            const double source_scale_velocity[3] = {
                source_node.scale_velocity.x,
                source_node.scale_velocity.y,
                source_node.scale_velocity.z
            };
            const double target_scale_velocity[3] = {
                target_node.scale_velocity.x,
                target_node.scale_velocity.y,
                target_node.scale_velocity.z
            };
            double blended_translation[3] = {};
            double blended_translation_velocity[3] = {};
            double blended_scale[3] = {};
            double blended_scale_velocity[3] = {};
            for (std::size_t component = 0u; component < 3u; ++component)
            {
                blended_translation[component] =
                    (1.0 - weight) * source_translation[component] +
                    weight * target_translation[component];
                blended_translation_velocity[component] =
                    (1.0 - weight) * source_translation_velocity[component] +
                    weight * target_translation_velocity[component] +
                    weight_derivative *
                        (target_translation[component] -
                            source_translation[component]);
                blended_scale[component] =
                    (1.0 - weight) * source_scale[component] +
                    weight * target_scale[component];
                blended_scale_velocity[component] =
                    (1.0 - weight) * source_scale_velocity[component] +
                    weight * target_scale_velocity[component] +
                    weight_derivative *
                        (target_scale[component] - source_scale[component]);

                const double maximum_float =
                    static_cast<double>(std::numeric_limits<float>::max());
                if (!std::isfinite(blended_translation[component]) ||
                    !std::isfinite(blended_translation_velocity[component]) ||
                    !std::isfinite(blended_scale[component]) ||
                    !std::isfinite(blended_scale_velocity[component]) ||
                    std::abs(blended_translation[component]) > maximum_float ||
                    std::abs(blended_translation_velocity[component]) > maximum_float ||
                    std::abs(blended_scale[component]) > maximum_float ||
                    std::abs(blended_scale_velocity[component]) > maximum_float)
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_transform,
                        "Blended Node Translation / Scale or their analytic derivatives exceeded finite float range.");
                }
            }
            blended.translation = {
                static_cast<float>(blended_translation[0]),
                static_cast<float>(blended_translation[1]),
                static_cast<float>(blended_translation[2])
            };
            blended.translation_velocity = {
                static_cast<float>(blended_translation_velocity[0]),
                static_cast<float>(blended_translation_velocity[1]),
                static_cast<float>(blended_translation_velocity[2])
            };
            blended.scale = {
                static_cast<float>(blended_scale[0]),
                static_cast<float>(blended_scale[1]),
                static_cast<float>(blended_scale[2])
            };
            blended.scale_velocity = {
                static_cast<float>(blended_scale_velocity[0]),
                static_cast<float>(blended_scale_velocity[1]),
                static_cast<float>(blended_scale_velocity[2])
            };

            animation_component_array source_rotation = {
                source_node.rotation.x,
                source_node.rotation.y,
                source_node.rotation.z,
                source_node.rotation.w
            };
            animation_component_array target_rotation = {
                target_node.rotation.x,
                target_node.rotation.y,
                target_node.rotation.z,
                target_node.rotation.w
            };
            animation_component_array source_rotation_derivative = {
                source_node.rotation_velocity.x,
                source_node.rotation_velocity.y,
                source_node.rotation_velocity.z,
                source_node.rotation_velocity.w
            };
            animation_component_array target_rotation_derivative = {
                target_node.rotation_velocity.x,
                target_node.rotation_velocity.y,
                target_node.rotation_velocity.z,
                target_node.rotation_velocity.w
            };

            animation_component_array normalized_source_rotation = {};
            animation_component_array normalized_target_rotation = {};
            double source_rotation_length = 0.0;
            double target_rotation_length = 0.0;
            if (!normalize_quaternion_double(
                    source_rotation,
                    normalized_source_rotation,
                    source_rotation_length) ||
                !normalize_quaternion_double(
                    target_rotation,
                    normalized_target_rotation,
                    target_rotation_length))
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_transform,
                    "Source / Target Node Rotation contains an invalid quaternion.");
            }

            const double source_projection =
                dot_quaternion(
                    normalized_source_rotation,
                    source_rotation_derivative);
            const double target_projection =
                dot_quaternion(
                    normalized_target_rotation,
                    target_rotation_derivative);
            animation_component_array normalized_source_derivative = {};
            animation_component_array normalized_target_derivative = {};
            for (std::size_t component = 0u; component < 4u; ++component)
            {
                normalized_source_derivative[component] =
                    (source_rotation_derivative[component] -
                        normalized_source_rotation[component] *
                            source_projection) / source_rotation_length;
                normalized_target_derivative[component] =
                    (target_rotation_derivative[component] -
                        normalized_target_rotation[component] *
                            target_projection) / target_rotation_length;
            }

            if (dot_quaternion(
                    normalized_source_rotation,
                    normalized_target_rotation) < 0.0)
            {
                for (std::size_t component = 0u; component < 4u; ++component)
                {
                    normalized_target_rotation[component] =
                        -normalized_target_rotation[component];
                    normalized_target_derivative[component] =
                        -normalized_target_derivative[component];
                }
            }

            animation_component_array raw_blended_rotation = {};
            animation_component_array raw_blended_rotation_derivative = {};
            for (std::size_t component = 0u; component < 4u; ++component)
            {
                raw_blended_rotation[component] =
                    (1.0 - weight) * normalized_source_rotation[component] +
                    weight * normalized_target_rotation[component];
                raw_blended_rotation_derivative[component] =
                    (1.0 - weight) *
                        normalized_source_derivative[component] +
                    weight * normalized_target_derivative[component] +
                    weight_derivative *
                        (normalized_target_rotation[component] -
                            normalized_source_rotation[component]);
            }

            animation_component_array normalized_blended_rotation = {};
            double blended_rotation_length = 0.0;
            if (!normalize_quaternion_double(
                    raw_blended_rotation,
                    normalized_blended_rotation,
                    blended_rotation_length))
            {
                return make_hierarchy_error(
                    animation_hierarchy_error::invalid_transform,
                    "Hemisphere-aligned Source / Target Quaternion Blend produced a zero or non-finite quaternion.");
            }

            const double blended_projection =
                dot_quaternion(
                    normalized_blended_rotation,
                    raw_blended_rotation_derivative);
            animation_component_array blended_rotation_derivative = {};
            for (std::size_t component = 0u; component < 4u; ++component)
            {
                blended_rotation_derivative[component] =
                    (raw_blended_rotation_derivative[component] -
                        normalized_blended_rotation[component] *
                            blended_projection) / blended_rotation_length;
                if (!std::isfinite(normalized_blended_rotation[component]) ||
                    !std::isfinite(blended_rotation_derivative[component]) ||
                    std::abs(normalized_blended_rotation[component]) >
                        static_cast<double>(std::numeric_limits<float>::max()) ||
                    std::abs(blended_rotation_derivative[component]) >
                        static_cast<double>(std::numeric_limits<float>::max()))
                {
                    return make_hierarchy_error(
                        animation_hierarchy_error::invalid_transform,
                        "Blended Quaternion or its analytic derivative exceeded finite float range.");
                }
            }
            blended.rotation = {
                static_cast<float>(normalized_blended_rotation[0]),
                static_cast<float>(normalized_blended_rotation[1]),
                static_cast<float>(normalized_blended_rotation[2]),
                static_cast<float>(normalized_blended_rotation[3])
            };
            blended.rotation_velocity = {
                static_cast<float>(blended_rotation_derivative[0]),
                static_cast<float>(blended_rotation_derivative[1]),
                static_cast<float>(blended_rotation_derivative[2]),
                static_cast<float>(blended_rotation_derivative[3])
            };
        }

        return build_animation_hierarchy_from_node_trs(
            model, blended_transforms);
    }


    animation_trajectory_packet_build_result build_animation_trajectory_packet(
        const asset::model& model,
        const animation_trajectory_packet_build_desc& desc)
    {
        const auto fail =
            [](animation_trajectory_packet_error error,
                const std::string& message)
        {
            animation_trajectory_packet_build_result result;
            result.error = error;
            result.message = message;
            return result;
        };

        if (desc.animation_input_version == 0u)
        {
            return fail(
                animation_trajectory_packet_error::invalid_version,
                "Animation Trajectory Packet requires a non-zero immutable Input Version.");
        }
        if (desc.maximum_span_count == 0u ||
            desc.spans.empty() ||
            desc.spans.size() > desc.maximum_span_count ||
            desc.spans.size() >
                static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))
        {
            return fail(
                animation_trajectory_packet_error::capacity_exceeded,
                "Animation Trajectory Packet Span count is empty or exceeds its configured / uint32 capacity.");
        }

        const std::size_t node_count = model.nodes.size();
        if (node_count == 0u ||
            node_count >= static_cast<std::size_t>(asset::invalid_index) ||
            model.animations.empty() ||
            node_count >
                static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))
        {
            return fail(
                animation_trajectory_packet_error::invalid_model,
                "Animation Trajectory Packet requires a non-empty Model Node hierarchy and at least one Asset Clip.");
        }

        const auto to_gpu_vec4 = [](const asset::vec4& value)
        {
            gpu_float4 result;
            result.x = value.x;
            result.y = value.y;
            result.z = value.z;
            result.w = value.w;
            return result;
        };

        const auto to_gpu_matrix = [](const asset::mat4& value)
        {
            gpu_matrix4x4 result;
            result.column0 = {
                value.value[0], value.value[1],
                value.value[2], value.value[3]
            };
            result.column1 = {
                value.value[4], value.value[5],
                value.value[6], value.value[7]
            };
            result.column2 = {
                value.value[8], value.value[9],
                value.value[10], value.value[11]
            };
            result.column3 = {
                value.value[12], value.value[13],
                value.value[14], value.value[15]
            };
            return result;
        };

        const auto finite_float_from_double =
            [](double value, float& output) -> bool
        {
            if (!std::isfinite(value) ||
                std::abs(value) >
                    static_cast<double>(std::numeric_limits<float>::max()))
            {
                return false;
            }
            output = static_cast<float>(value);
            return std::isfinite(output);
        };

        // Validate and topologically order the complete Asset Node hierarchy.
        std::vector<std::vector<uint32_t>> children(node_count);
        std::vector<bool> child_seen(node_count, false);
        for (uint32_t parent_index = 0u;
            parent_index < node_count; ++parent_index)
        {
            const asset::node& parent = model.nodes[parent_index];
            for (uint32_t child_index : parent.children)
            {
                if (child_index >= node_count ||
                    child_index == parent_index ||
                    model.nodes[child_index].parent_index != parent_index ||
                    child_seen[child_index])
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_hierarchy,
                        "Asset Node Children contain an invalid, duplicated or inconsistent Parent reference.");
                }
                child_seen[child_index] = true;
                children[parent_index].push_back(child_index);
            }
        }

        std::vector<uint32_t> topological_order;
        topological_order.reserve(node_count);
        std::vector<uint32_t> ready;
        ready.reserve(node_count);
        for (uint32_t node_index = 0u; node_index < node_count; ++node_index)
        {
            const uint32_t parent_index =
                model.nodes[node_index].parent_index;
            if (parent_index == asset::invalid_index)
            {
                ready.push_back(node_index);
                continue;
            }
            if (parent_index >= node_count ||
                parent_index == node_index ||
                !child_seen[node_index])
            {
                return fail(
                    animation_trajectory_packet_error::invalid_hierarchy,
                    "Asset Node Parent Index is invalid or disagrees with its Parent Child list.");
            }
        }

        std::size_t ready_cursor = 0u;
        while (ready_cursor < ready.size())
        {
            const uint32_t node_index = ready[ready_cursor++];
            topological_order.push_back(node_index);
            for (uint32_t child_index : children[node_index])
            {
                ready.push_back(child_index);
            }
        }
        if (topological_order.size() != node_count)
        {
            return fail(
                animation_trajectory_packet_error::invalid_hierarchy,
                "Asset Node hierarchy contains a parent cycle or an unreachable Node.");
        }

        // Track Channel ranges use the same deterministic parent-before-child
        // order for every Span, so Skin records can store stable relative
        // offsets instead of assuming Asset Node Index equals Channel Index.
        std::vector<uint32_t> node_channel_offset_by_node(
            node_count, asset::invalid_index);
        for (std::size_t channel_offset = 0u;
            channel_offset < topological_order.size(); ++channel_offset)
        {
            node_channel_offset_by_node[topological_order[channel_offset]] =
                static_cast<uint32_t>(channel_offset);
        }

        animation_trajectory_packet candidate;
        candidate.animation_input_version = desc.animation_input_version;
        candidate.static_node_matrices.resize(node_count);
        std::vector<uint32_t> node_role_flags(node_count, 0u);
        std::vector<uint32_t> inverse_bind_index_by_node(
            node_count, asset::invalid_index);
        std::vector<asset::mat4> inverse_bind_by_node(node_count);

        // Build a dense Static Matrix Buffer indexed by Asset Node Index.
        // TRS Nodes carry Identity; affine Matrix Nodes carry their exact
        // immutable local transform and are identified by Role flag bit4.
        for (uint32_t node_index = 0u;
            node_index < node_count; ++node_index)
        {
            const asset::node& node = model.nodes[node_index];
            node_role_flags[node_index] =
                node.parent_index == asset::invalid_index
                    ? animation_node_role_model_root
                    : 0u;

            if (node.mesh_index != asset::invalid_index)
            {
                if (node.mesh_index >= model.meshes.size())
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_model,
                        "An Asset Node references a Mesh outside the Model Mesh array.");
                }
                node_role_flags[node_index] |= animation_node_role_mesh;
            }

            const asset::mat4 static_matrix =
                node.uses_matrix ? node.local_matrix : asset::mat4{};
            if (!is_affine_matrix(static_matrix))
            {
                return fail(
                    animation_trajectory_packet_error::invalid_model,
                    "A Static Node Matrix is non-finite or non-affine.");
            }
            candidate.static_node_matrices[node_index] =
                to_gpu_matrix(static_matrix);
            if (node.uses_matrix)
            {
                node_role_flags[node_index] |=
                    animation_node_role_static_matrix;
            }
            else
            {
                const quaternion node_rotation = normalize_quaternion({
                    node.rotation.x,
                    node.rotation.y,
                    node.rotation.z,
                    node.rotation.w
                });
                const double quaternion_length_squared =
                    static_cast<double>(node.rotation.x) * node.rotation.x +
                    static_cast<double>(node.rotation.y) * node.rotation.y +
                    static_cast<double>(node.rotation.z) * node.rotation.z +
                    static_cast<double>(node.rotation.w) * node.rotation.w;
                if (!finite(node.translation) ||
                    !finite(node.scale) ||
                    !std::isfinite(quaternion_length_squared) ||
                    quaternion_length_squared <= 1.0e-24 ||
                    !std::isfinite(node.scale.x) ||
                    !std::isfinite(node.scale.y) ||
                    !std::isfinite(node.scale.z) ||
                    node.scale.x <= 0.0f ||
                    node.scale.y <= 0.0f ||
                    node.scale.z <= 0.0f ||
                    !std::isfinite(node_rotation.x) ||
                    !std::isfinite(node_rotation.y) ||
                    !std::isfinite(node_rotation.z) ||
                    !std::isfinite(node_rotation.w))
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_model,
                        "An Asset Rest TRS Node contains non-finite Translation / Rotation / Scale or a non-positive Scale.");
                }
            }
        }

        // A channel record has one Inverse Bind Index per Joint Node. Reuse a
        // record when multiple Skins share the exact same Joint / IBM pair;
        // conflicting per-Skin Inverse Binds cannot be represented by this
        // v1 Node-indexed ABI and therefore fail closed.
        for (const asset::skin& skin : model.skins)
        {
            if (skin.joints.empty() ||
                (!skin.inverse_bind_matrices.empty() &&
                    skin.inverse_bind_matrices.size() != skin.joints.size()))
            {
                return fail(
                    animation_trajectory_packet_error::invalid_skin,
                    "Skin Joint and Inverse Bind arrays must be non-empty and have matching sizes when Inverse Binds are supplied.");
            }

            for (std::size_t joint_slot = 0u;
                joint_slot < skin.joints.size(); ++joint_slot)
            {
                const uint32_t joint_node_index = skin.joints[joint_slot];
                if (joint_node_index >= node_count)
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_skin,
                        "A Skin Joint Node Index is outside the Asset Model.");
                }

                const asset::mat4 inverse_bind =
                    skin.inverse_bind_matrices.empty()
                        ? asset::mat4{}
                        : skin.inverse_bind_matrices[joint_slot];
                if (!is_affine_matrix(inverse_bind))
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_skin,
                        "A Skin Inverse Bind Matrix is non-finite or non-affine.");
                }

                if (candidate.inverse_bind_matrices.size() >=
                    static_cast<std::size_t>(asset::invalid_index))
                {
                    return fail(
                        animation_trajectory_packet_error::capacity_exceeded,
                        "Per-Skin Inverse Bind Matrix count exceeds the uint32 record index space.");
                }

                // Inverse Binds belong to a Skin / Joint pair. The same Joint
                // Node may legally have different Inverse Binds in different
                // Skins, so retain one immutable matrix per ordered Skin joint.
                const uint32_t inverse_bind_record_index =
                    static_cast<uint32_t>(
                        candidate.inverse_bind_matrices.size());
                if (inverse_bind_index_by_node[joint_node_index] ==
                    asset::invalid_index)
                {
                    inverse_bind_index_by_node[joint_node_index] =
                        inverse_bind_record_index;
                    inverse_bind_by_node[joint_node_index] = inverse_bind;
                }
                candidate.inverse_bind_matrices.push_back(
                    to_gpu_matrix(inverse_bind));
                node_role_flags[joint_node_index] |=
                    animation_node_role_joint;
            }
        }

        // Every Skinned Mesh and Joint requires all of its Ancestors for a
        // complete Global Matrix. The Channel range currently stores all
        // Nodes, while these Role bits identify the minimal semantic subset.
        for (uint32_t node_index = 0u;
            node_index < node_count; ++node_index)
        {
            const asset::node& node = model.nodes[node_index];
            if (node.skin_index != asset::invalid_index)
            {
                if (node.skin_index >= model.skins.size() ||
                    node.mesh_index == asset::invalid_index)
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_skin,
                        "A Skinned Mesh Node must reference a valid Skin and Mesh.");
                }
            }

            const bool is_joint =
                (node_role_flags[node_index] &
                    animation_node_role_joint) != 0u;
            const bool is_skinned_mesh =
                node.skin_index != asset::invalid_index;
            if (!is_joint && !is_skinned_mesh)
            {
                continue;
            }

            uint32_t ancestor_index = node.parent_index;
            while (ancestor_index != asset::invalid_index)
            {
                node_role_flags[ancestor_index] |=
                    animation_node_role_required_joint_ancestor;
                ancestor_index =
                    model.nodes[ancestor_index].parent_index;
            }
        }

        // The static Skin table identifies which ordered Joint set each
        // Skinned Mesh Node consumes. Joint entries carry Skin-specific IBM
        // indices, even when a Joint Node is shared by multiple Skins.
        for (uint32_t mesh_node_index = 0u;
            mesh_node_index < node_count; ++mesh_node_index)
        {
            const asset::node& mesh_node = model.nodes[mesh_node_index];
            if (mesh_node.skin_index == asset::invalid_index)
            {
                continue;
            }
            if (mesh_node.skin_index >= model.skins.size() ||
                mesh_node.mesh_index == asset::invalid_index)
            {
                return fail(
                    animation_trajectory_packet_error::invalid_skin,
                    "A Skin mapping requires a Mesh Node with a valid Skin Index.");
            }

            const asset::skin& skin = model.skins[mesh_node.skin_index];
            if (skin.joints.empty() ||
                skin.joints.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) ||
                candidate.skin_joint_records.size() >
                    static_cast<std::size_t>(asset::invalid_index) -
                        skin.joints.size())
            {
                return fail(
                    animation_trajectory_packet_error::capacity_exceeded,
                    "Skin Joint mapping count is empty or exceeds its uint32 record range.");
            }
            if (candidate.skin_records.size() >=
                static_cast<std::size_t>(asset::invalid_index))
            {
                return fail(
                    animation_trajectory_packet_error::capacity_exceeded,
                    "Skin mapping count exceeds the uint32 record index space.");
            }

            animation_skin_gpu_record skin_record;
            skin_record.mesh_node_channel_offset =
                node_channel_offset_by_node[mesh_node_index];
            skin_record.skin_index = mesh_node.skin_index;
            skin_record.first_joint_record =
                static_cast<uint32_t>(candidate.skin_joint_records.size());
            skin_record.joint_count =
                static_cast<uint32_t>(skin.joints.size());

            for (std::size_t joint_slot = 0u;
                joint_slot < skin.joints.size(); ++joint_slot)
            {
                const uint32_t joint_node_index = skin.joints[joint_slot];
                if (joint_node_index >= node_count ||
                    inverse_bind_index_by_node[joint_node_index] ==
                        asset::invalid_index ||
                    node_channel_offset_by_node[joint_node_index] ==
                        asset::invalid_index)
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_skin,
                        "Skin Joint mapping references a Node without a validated Channel or Skin-specific Inverse Bind Matrix.");
                }

                // The inverse-bind index is resolved per Skin / Joint pair.
                // Revisit the source Skin record to find this exact matrix's
                // immutable Buffer element instead of using the Node's
                // canonical first-encountered compatibility index.
                uint32_t skin_joint_inverse_bind_index = asset::invalid_index;
                std::size_t preceding_joint_count = 0u;
                for (uint32_t preceding_mesh_node = 0u;
                    preceding_mesh_node < mesh_node_index;
                    ++preceding_mesh_node)
                {
                    const asset::node& previous_node =
                        model.nodes[preceding_mesh_node];
                    if (previous_node.skin_index == mesh_node.skin_index)
                    {
                        preceding_joint_count +=
                            model.skins[previous_node.skin_index].joints.size();
                    }
                }
                for (std::size_t previous_joint = 0u;
                    previous_joint < joint_slot; ++previous_joint)
                {
                    ++preceding_joint_count;
                }
                const uint64_t inverse_bind_index =
                    0u; // Resolved below from the ordered Skin upload range.

                (void)inverse_bind_index;
                // Skin matrices are appended in Skin-array order above, so
                // compute this Skin's base by summing preceding Skin sizes.
                std::size_t skin_joint_base = 0u;
                for (std::size_t previous_skin = 0u;
                    previous_skin < mesh_node.skin_index; ++previous_skin)
                {
                    skin_joint_base += model.skins[previous_skin].joints.size();
                }
                if (skin_joint_base >
                    static_cast<std::size_t>(asset::invalid_index) -
                        joint_slot)
                {
                    return fail(
                        animation_trajectory_packet_error::capacity_exceeded,
                        "Skin-specific Inverse Bind reference exceeds uint32 range.");
                }
                skin_joint_inverse_bind_index = static_cast<uint32_t>(
                    skin_joint_base + joint_slot);
                if (skin_joint_inverse_bind_index >=
                    candidate.inverse_bind_matrices.size())
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_skin,
                        "Skin-specific Inverse Bind record index is outside the immutable Buffer.");
                }

                animation_skin_joint_gpu_record joint_record;
                joint_record.joint_channel_offset =
                    node_channel_offset_by_node[joint_node_index];
                joint_record.inverse_bind_record_index =
                    skin_joint_inverse_bind_index;
                joint_record.joint_node_index = joint_node_index;
                candidate.skin_joint_records.push_back(joint_record);
            }
            candidate.skin_records.push_back(skin_record);
        }

        const uint32_t version_low =
            static_cast<uint32_t>(desc.animation_input_version & 0xffffffffull);
        const uint32_t version_high =
            static_cast<uint32_t>(desc.animation_input_version >> 32u);
        double previous_span_end = 0.0;
        bool has_previous_span = false;
        std::vector<uint32_t> span_ids;
        span_ids.reserve(desc.spans.size());

        for (std::size_t span_index = 0u;
            span_index < desc.spans.size(); ++span_index)
        {
            const animation_trajectory_span_input& input =
                desc.spans[span_index];
            const double span_end =
                input.world_time_start + input.world_time_duration;
            if (!std::isfinite(input.world_time_start) ||
                !std::isfinite(input.world_time_duration) ||
                input.world_time_duration <= 0.0 ||
                !std::isfinite(span_end) ||
                span_end == input.world_time_start ||
                (has_previous_span &&
                    input.world_time_start != previous_span_end))
            {
                return fail(
                    animation_trajectory_packet_error::invalid_span,
                    "Animation Trajectory Spans must be finite, positive, contiguous and representable in double World Time.");
            }
            if (std::find(
                    span_ids.begin(), span_ids.end(), input.span_id) !=
                span_ids.end())
            {
                return fail(
                    animation_trajectory_packet_error::invalid_span,
                    "Animation Trajectory Span IDs must be unique within one immutable Packet.");
            }
            span_ids.push_back(input.span_id);
            previous_span_end = span_end;
            has_previous_span = true;

            const std::size_t expected_track_count =
                input.has_blend ? 2u : 1u;
            if (input.tracks.size() != expected_track_count)
            {
                return fail(
                    animation_trajectory_packet_error::invalid_span,
                    "A v1 Animation Span requires one Track or exactly two Tracks with one Blend Record.");
            }
            if (input.has_blend &&
                (input.source_track_index >= input.tracks.size() ||
                    input.target_track_index >= input.tracks.size() ||
                    input.source_track_index == input.target_track_index ||
                    !valid_animation_blend_weight_curve(input.blend_curve) ||
                    (input.blend_curve.blend_start_time > input.world_time_start &&
                        input.blend_curve.blend_start_time < span_end) ||
                    (input.blend_curve.blend_end_time > input.world_time_start &&
                        input.blend_curve.blend_end_time < span_end)))
            {
                return fail(
                    animation_trajectory_packet_error::invalid_span,
                    "Blend Span must reference two distinct local Tracks, use a valid Hermite Curve and remain inside one Blend Curve boundary interval.");
            }
            if (!input.has_blend && input.tracks.size() != 1u)
            {
                return fail(
                    animation_trajectory_packet_error::invalid_span,
                    "A non-Blend Animation Span must contain exactly one Track.");
            }

            if (!std::isfinite(input.bounds.speed_bound) ||
                !std::isfinite(input.bounds.acceleration_bound) ||
                !std::isfinite(input.bounds.chord_deviation_bound) ||
                input.bounds.speed_bound < 0.0f ||
                input.bounds.acceleration_bound < 0.0f ||
                input.bounds.chord_deviation_bound < 0.0f ||
                !std::isfinite(input.bounds.quaternion_normalization_lower_bound) ||
                input.bounds.quaternion_normalization_lower_bound <= 1.0e-12f)
            {
                return fail(
                    animation_trajectory_packet_error::invalid_bounds,
                    "Animation Span requires finite non-negative outward-rounded Speed / Acceleration / Chord Bounds and a positive Quaternion Normalization lower bound.");
            }

            animation_span_gpu_record span_record;
            span_record.span_id = input.span_id;
            span_record.animation_input_version_low = version_low;
            span_record.animation_input_version_high = version_high;
            if (!finite_float_from_double(
                    input.world_time_duration,
                    span_record.span_duration_seconds) ||
                span_record.span_duration_seconds <= 0.0f)
            {
                return fail(
                    animation_trajectory_packet_error::invalid_span,
                    "Animation Span duration cannot be represented as a positive GPU float.");
            }
            const double inverse_span_duration =
                1.0 / input.world_time_duration;
            if (!finite_float_from_double(
                    inverse_span_duration,
                    span_record.inverse_span_duration) ||
                span_record.inverse_span_duration <= 0.0f)
            {
                return fail(
                    animation_trajectory_packet_error::invalid_span,
                    "Animation Span inverse duration cannot be represented as a positive GPU float.");
            }
            span_record.speed_bound = input.bounds.speed_bound;
            span_record.acceleration_bound =
                input.bounds.acceleration_bound;
            span_record.chord_deviation_bound =
                input.bounds.chord_deviation_bound;
            span_record.quaternion_normalization_lower_bound =
                input.bounds.quaternion_normalization_lower_bound;
            span_record.first_track_record =
                static_cast<uint32_t>(candidate.track_records.size());
            span_record.first_channel_record =
                static_cast<uint32_t>(candidate.channel_records.size());
            span_record.first_blend_record =
                static_cast<uint32_t>(candidate.blend_records.size());
            span_record.first_collider_record =
                static_cast<uint32_t>(candidate.collider_records.size());

            for (const animation_trajectory_track_span_input& track_input :
                input.tracks)
            {
                if (track_input.clip_index >= model.animations.size())
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_track,
                        "Animation Track Clip Index is outside the immutable Asset Model Animation array.");
                }
                const asset::animation_clip& clip =
                    model.animations[track_input.clip_index];
                const scheduled_animation_clip_span& scheduled =
                    track_input.scheduled;
                if (clip.tracks.empty() ||
                    scheduled.clip.channels.size() != clip.tracks.size() ||
                    scheduled.world_time_start != input.world_time_start ||
                    scheduled.world_time_duration != input.world_time_duration ||
                    !std::isfinite(scheduled.clip.clip_time_start) ||
                    !std::isfinite(scheduled.clip.clip_time_delta) ||
                    !std::isfinite(
                        scheduled.clip.clip_time_start +
                        scheduled.clip.clip_time_delta))
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_track,
                        "Scheduled Animation Clip Span must match the Packet World-Time interval and contain one channel record per original Asset Track.");
                }

                const double playback_speed =
                    scheduled.clip.clip_time_delta /
                    scheduled.world_time_duration;
                float gpu_clip_time_start = 0.0f;
                float gpu_clip_time_delta = 0.0f;
                float gpu_playback_speed = 0.0f;
                if (!finite_float_from_double(
                        scheduled.clip.clip_time_start,
                        gpu_clip_time_start) ||
                    !finite_float_from_double(
                        scheduled.clip.clip_time_delta,
                        gpu_clip_time_delta) ||
                    !finite_float_from_double(
                        playback_speed,
                        gpu_playback_speed))
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_track,
                        "Animation Clip Time / signed Delta / Playback Speed exceed finite GPU float range.");
                }
                if ((scheduled.clip.clip_time_delta < 0.0 &&
                        !track_input.reverse_playback) ||
                    (scheduled.clip.clip_time_delta > 0.0 &&
                        track_input.reverse_playback))
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_track,
                        "Animation Track Reverse Playback flag disagrees with its signed Clip-Time Delta.");
                }

                const animation_hierarchy_sample_result hierarchy_validation =
                    evaluate_animation_clip_hierarchy(
                        model,
                        clip,
                        scheduled.clip,
                        0.0,
                        scheduled.world_time_duration);
                if (!hierarchy_validation.success)
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_hierarchy,
                        "Animation Clip Span failed CPU Node / Skin validation before Packet construction: " +
                            hierarchy_validation.message);
                }

                animation_track_gpu_record track_record;
                track_record.clip_index = track_input.clip_index;
                track_record.first_channel_record =
                    static_cast<uint32_t>(candidate.channel_records.size());
                track_record.channel_record_count =
                    static_cast<uint32_t>(node_count);
                track_record.flags =
                    (track_input.loop ? animation_track_flag_loop : 0u) |
                    (track_input.reverse_playback
                        ? animation_track_flag_reverse_playback
                        : 0u);
                track_record.clip_time_start = gpu_clip_time_start;
                track_record.clip_time_delta = gpu_clip_time_delta;
                track_record.playback_speed = gpu_playback_speed;

                std::vector<uint32_t> node_channel_record_index(
                    node_count, asset::invalid_index);
                std::vector<std::array<bool, 3>> channel_seen(node_count);
                for (std::array<bool, 3>& seen : channel_seen)
                {
                    seen = { false, false, false };
                }

                for (uint32_t node_index : topological_order)
                {
                    const asset::node& node = model.nodes[node_index];
                    animation_channel_gpu_record channel_record;
                    channel_record.node_index = node_index;
                    channel_record.parent_index =
                        node.parent_index == asset::invalid_index
                            ? asset::invalid_index
                            : node_channel_record_index[node.parent_index];
                    channel_record.inverse_bind_record_index =
                        inverse_bind_index_by_node[node_index];
                    channel_record.node_role_flags =
                        node_role_flags[node_index];

                    asset::vec3 rest_translation;
                    asset::vec4 rest_rotation = {
                        0.0f, 0.0f, 0.0f, 1.0f
                    };
                    asset::vec3 rest_scale = {
                        1.0f, 1.0f, 1.0f
                    };
                    if (!node.uses_matrix)
                    {
                        rest_translation = node.translation;
                        rest_scale = node.scale;
                        const quaternion normalized_rotation =
                            normalize_quaternion({
                                node.rotation.x,
                                node.rotation.y,
                                node.rotation.z,
                                node.rotation.w
                            });
                        rest_rotation = {
                            normalized_rotation.x,
                            normalized_rotation.y,
                            normalized_rotation.z,
                            normalized_rotation.w
                        };
                    }

                    channel_record.translation_c0 = {
                        rest_translation.x,
                        rest_translation.y,
                        rest_translation.z,
                        0.0f
                    };
                    channel_record.rotation_parameter0 = {
                        rest_rotation.x,
                        rest_rotation.y,
                        rest_rotation.z,
                        rest_rotation.w
                    };
                    channel_record.scale_c0 = {
                        rest_scale.x,
                        rest_scale.y,
                        rest_scale.z,
                        0.0f
                    };

                    node_channel_record_index[node_index] =
                        static_cast<uint32_t>(
                            candidate.channel_records.size());
                    candidate.channel_records.push_back(channel_record);
                }

                for (std::size_t track_index = 0u;
                    track_index < clip.tracks.size(); ++track_index)
                {
                    const asset::animation_track& asset_track =
                        clip.tracks[track_index];
                    const animation_channel_span& channel_span =
                        scheduled.clip.channels[track_index];
                    if (channel_span.track_index != track_index ||
                        channel_span.node_index != asset_track.node_index ||
                        channel_span.path != asset_track.path ||
                        channel_span.interpolation_mode != asset_track.mode ||
                        channel_span.clip_time_start !=
                            scheduled.clip.clip_time_start ||
                        channel_span.clip_time_delta !=
                            scheduled.clip.clip_time_delta ||
                        channel_span.node_index >= node_count)
                    {
                        return fail(
                            animation_trajectory_packet_error::invalid_channel,
                            "Animation Channel Span does not match its original Track Index / Node / Path / Interpolation / common Clip-Time interval.");
                    }
                    if (model.nodes[channel_span.node_index].uses_matrix)
                    {
                        return fail(
                            animation_trajectory_packet_error::invalid_channel,
                            "An animated TRS Channel cannot target a static Matrix Node.");
                    }

                    for (const asset::vec4& coefficient :
                        channel_span.coefficients)
                    {
                        if (!finite_asset_vec4(coefficient))
                        {
                            return fail(
                                animation_trajectory_packet_error::invalid_channel,
                                "Animation Channel Span contains a non-finite GPU coefficient.");
                        }
                    }
                    const animation_channel_span_sample_result start_sample =
                        evaluate_animation_channel_span(
                            channel_span, 0.0, scheduled.world_time_duration);
                    const animation_channel_span_sample_result end_sample =
                        evaluate_animation_channel_span(
                            channel_span, 1.0, scheduled.world_time_duration);
                    if (!start_sample.success || !end_sample.success)
                    {
                        return fail(
                            animation_trajectory_packet_error::invalid_channel,
                            "Animation Channel Span endpoint evaluation failed: " +
                                (!start_sample.success
                                    ? start_sample.message
                                    : end_sample.message));
                    }

                    std::size_t channel_slot = 0u;
                    uint32_t channel_mask = 0u;
                    uint32_t interpolation_shift = 0u;
                    if (channel_span.path == asset::animation_path::translation)
                    {
                        channel_slot = 0u;
                        channel_mask = animation_channel_mask_translation;
                        interpolation_shift = 0u;
                    }
                    else if (channel_span.path == asset::animation_path::rotation)
                    {
                        channel_slot = 1u;
                        channel_mask = animation_channel_mask_rotation;
                        interpolation_shift = 2u;
                    }
                    else if (channel_span.path == asset::animation_path::scale)
                    {
                        channel_slot = 2u;
                        channel_mask = animation_channel_mask_scale;
                        interpolation_shift = 4u;
                    }
                    else
                    {
                        return fail(
                            animation_trajectory_packet_error::invalid_channel,
                            "Animation Channel contains an unsupported glTF Path.");
                    }

                    const uint32_t node_record_index =
                        node_channel_record_index[channel_span.node_index];
                    animation_channel_gpu_record& gpu_channel =
                        candidate.channel_records[node_record_index];
                    if (channel_seen[channel_span.node_index][channel_slot])
                    {
                        return fail(
                            animation_trajectory_packet_error::invalid_channel,
                            "Animation Clip contains duplicate channels for one Node / Path pair.");
                    }
                    channel_seen[channel_span.node_index][channel_slot] = true;
                    gpu_channel.channel_mask |= channel_mask;

                    const uint32_t interpolation_value =
                        static_cast<uint32_t>(
                            channel_span.interpolation_mode);
                    if (interpolation_value >
                        static_cast<uint32_t>(asset::interpolation::cubic_spline))
                    {
                        return fail(
                            animation_trajectory_packet_error::invalid_channel,
                            "Animation Channel Interpolation enum exceeds the fixed GPU ABI range.");
                    }
                    gpu_channel.interpolation_flags |=
                        interpolation_value << interpolation_shift;

                    if (channel_span.path == asset::animation_path::translation)
                    {
                        gpu_channel.translation_c0 =
                            to_gpu_vec4(channel_span.coefficients[0]);
                        gpu_channel.translation_c1 =
                            to_gpu_vec4(channel_span.coefficients[1]);
                        gpu_channel.translation_c2 =
                            to_gpu_vec4(channel_span.coefficients[2]);
                        gpu_channel.translation_c3 =
                            to_gpu_vec4(channel_span.coefficients[3]);
                    }
                    else if (channel_span.path == asset::animation_path::rotation)
                    {
                        gpu_channel.rotation_parameter0 =
                            to_gpu_vec4(channel_span.coefficients[0]);
                        gpu_channel.rotation_parameter1 =
                            to_gpu_vec4(channel_span.coefficients[1]);
                        gpu_channel.rotation_parameter2 =
                            to_gpu_vec4(channel_span.coefficients[2]);
                        gpu_channel.rotation_parameter3 =
                            to_gpu_vec4(channel_span.coefficients[3]);
                    }
                    else
                    {
                        gpu_channel.scale_c0 =
                            to_gpu_vec4(channel_span.coefficients[0]);
                        gpu_channel.scale_c1 =
                            to_gpu_vec4(channel_span.coefficients[1]);
                        gpu_channel.scale_c2 =
                            to_gpu_vec4(channel_span.coefficients[2]);
                        gpu_channel.scale_c3 =
                            to_gpu_vec4(channel_span.coefficients[3]);
                    }
                }

                candidate.track_records.push_back(track_record);
            }

            span_record.track_record_count =
                static_cast<uint32_t>(
                    candidate.track_records.size()) -
                span_record.first_track_record;
            span_record.channel_record_count =
                static_cast<uint32_t>(
                    candidate.channel_records.size()) -
                span_record.first_channel_record;
            span_record.blend_record_count =
                static_cast<uint32_t>(
                    candidate.blend_records.size()) -
                span_record.first_blend_record;
            span_record.collider_record_count = 0u;
            if (input.has_blend)
            {
                const double blend_duration =
                    input.blend_curve.blend_end_time -
                    input.blend_curve.blend_start_time;
                float gpu_blend_duration = 0.0f;
                float gpu_inverse_blend_duration = 0.0f;
                float gpu_blend_start_offset = 0.0f;
                if (!finite_float_from_double(
                        blend_duration, gpu_blend_duration) ||
                    gpu_blend_duration <= 0.0f ||
                    !finite_float_from_double(
                        1.0 / blend_duration,
                        gpu_inverse_blend_duration) ||
                    gpu_inverse_blend_duration <= 0.0f ||
                    !finite_float_from_double(
                        input.world_time_start -
                            input.blend_curve.blend_start_time,
                        gpu_blend_start_offset))
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_span,
                        "Hermite Blend duration / inverse / Span offset cannot be represented as finite GPU floats.");
                }

                const animation_component_array curve_coefficients =
                    blend_curve_coefficients(input.blend_curve);
                animation_blend_gpu_record blend_record;
                blend_record.source_track_index = input.source_track_index;
                blend_record.target_track_index = input.target_track_index;
                blend_record.weight_curve_flags =
                    animation_blend_flag_enabled;
                if (!finite_float_from_double(
                        curve_coefficients[0],
                        blend_record.weight_coefficients.x) ||
                    !finite_float_from_double(
                        curve_coefficients[1],
                        blend_record.weight_coefficients.y) ||
                    !finite_float_from_double(
                        curve_coefficients[2],
                        blend_record.weight_coefficients.z) ||
                    !finite_float_from_double(
                        curve_coefficients[3],
                        blend_record.weight_coefficients.w))
                {
                    return fail(
                        animation_trajectory_packet_error::invalid_span,
                        "Hermite Blend Weight coefficients exceed finite GPU float range.");
                }
                blend_record.duration_seconds = gpu_blend_duration;
                blend_record.inverse_duration =
                    gpu_inverse_blend_duration;
                blend_record.blend_start_offset_seconds =
                    gpu_blend_start_offset;
                candidate.blend_records.push_back(blend_record);
                span_record.flags |= animation_span_flag_has_blend;
            }
            candidate.span_world_time_starts.push_back(
                input.world_time_start);
            candidate.span_records.push_back(span_record);
        }

        animation_trajectory_packet_build_result result;
        result.value = std::move(candidate);
        result.success = true;
        return result;
    }


    bool initialize_animation_span_cursor(
        const animation_trajectory_packet& packet,
        uint32_t span_record_index,
        double segment_world_time,
        animation_span_cursor_gpu_record& out_cursor,
        std::string& diagnostic)
    {
        diagnostic.clear();
        if (packet.animation_input_version == 0u ||
            !std::isfinite(segment_world_time) ||
            span_record_index >= packet.span_records.size() ||
            packet.span_world_time_starts.size() !=
                packet.span_records.size())
        {
            diagnostic =
                "Animation Span Cursor initialization requires a valid immutable Packet, Span Index and finite World Time.";
            return false;
        }

        const animation_span_gpu_record& span =
            packet.span_records[span_record_index];
        const double world_start =
            packet.span_world_time_starts[span_record_index];
        const double world_duration =
            static_cast<double>(span.span_duration_seconds);
        const double world_end = world_start + world_duration;
        if (!std::isfinite(world_start) ||
            !std::isfinite(world_duration) ||
            world_duration <= 0.0 ||
            !std::isfinite(world_end) ||
            world_end <= world_start ||
            segment_world_time < world_start ||
            segment_world_time > world_end)
        {
            diagnostic =
                "Animation Segment World Time is outside the selected positive Span interval.";
            return false;
        }

        const double normalized_cursor =
            (segment_world_time - world_start) / world_duration;
        if (!std::isfinite(normalized_cursor) ||
            normalized_cursor < 0.0 ||
            normalized_cursor > 1.0)
        {
            diagnostic =
                "Animation Segment Start cannot be mapped to a normalized Span Cursor.";
            return false;
        }

        const float gpu_cursor =
            static_cast<float>(normalized_cursor);
        if (!std::isfinite(gpu_cursor) ||
            gpu_cursor < 0.0f ||
            gpu_cursor > 1.0f)
        {
            diagnostic =
                "Animation Segment Start Cursor cannot be represented as a normalized GPU float.";
            return false;
        }

        animation_span_cursor_gpu_record candidate;
        candidate.span_record_index = span_record_index;
        candidate.animation_input_version_low =
            span.animation_input_version_low;
        candidate.animation_input_version_high =
            span.animation_input_version_high;
        candidate.u_segment_start = gpu_cursor;
        out_cursor = candidate;
        return true;
    }

}
