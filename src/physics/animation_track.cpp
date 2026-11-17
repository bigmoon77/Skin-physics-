#include "skin_physics/physics/animation_track.hpp"

#include <algorithm>
#include <cmath>

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
}
