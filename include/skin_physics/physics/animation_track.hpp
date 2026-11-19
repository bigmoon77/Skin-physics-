#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::physics
{
    struct quaternion
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;
    };

    struct animation_keyframe
    {
        double time = 0.0;
        asset::vec3 translation;
        quaternion rotation;
        asset::vec3 scale = { 1.0f, 1.0f, 1.0f };
    };

    struct animation_track
    {
        std::vector<animation_keyframe> keyframes;
    };

    struct animation_sample_options
    {
        bool teleport = false;
        bool zero_velocity_on_teleport = true;
        asset::vec3 teleport_translation;
    };

    struct animation_transform_sample
    {
        asset::vec3 translation;
        quaternion rotation;
        asset::vec3 scale = { 1.0f, 1.0f, 1.0f };
        asset::vec3 translation_velocity;
    };

    enum class animation_error : uint8_t
    {
        none,
        empty_track,
        invalid_time,
        invalid_keyframe,
        non_monotonic_time,
        invalid_quaternion
    };

    struct animation_sample_result
    {
        animation_transform_sample value;
        animation_error error = animation_error::none;
        std::string message;
        bool success = false;
    };

    quaternion normalize_quaternion(const quaternion& value);
    quaternion slerp_quaternion(
        const quaternion& start,
        const quaternion& end,
        float interpolation);

    animation_sample_result sample_animation_track(
        const animation_track& track,
        double sample_time,
        const animation_sample_options& options = {});
}
