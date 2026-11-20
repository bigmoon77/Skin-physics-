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
            !std::isfinite(clip_time_start + clip_time_delta))
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Span Clip-Time start / delta must be finite and representable.";
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
            !std::isfinite(clip_time_start + clip_time_delta))
        {
            result.error = animation_channel_span_error::invalid_time;
            result.message =
                "Animation Span schedule Clip-Time start / delta must be finite and representable.";
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
            !std::isfinite(span_local_u) || span_local_u < 0.0 ||
            span_local_u > 1.0 ||
            !std::isfinite(span_duration_seconds) ||
            span_duration_seconds <= 0.0)
        {
            return make_span_sample_error(
                animation_channel_span_error::invalid_time,
                "Animation Span evaluation requires finite Clip-Time metadata, u in [0, 1] and a finite positive World-Time duration.");
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

}
