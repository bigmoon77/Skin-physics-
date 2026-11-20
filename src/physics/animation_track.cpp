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

}
