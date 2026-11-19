#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace skin_physics::physics
{
    enum class attribute_paint_mode : uint8_t
    {
        paint,
        smooth,
        average,
        erase
    };

    enum class attribute_paint_error : uint8_t
    {
        none,
        invalid_input,
        invalid_vertex,
        invalid_neighbor,
        non_finite_value,
        invalid_serialized_data
    };

    struct attribute_paint_result
    {
        std::vector<float> weights;
        attribute_paint_error error = attribute_paint_error::none;
        std::string message;
        bool success = false;
    };

    attribute_paint_result apply_attribute_paint(
        const std::vector<float>& source_weights,
        const std::vector<std::vector<uint32_t>>& adjacency,
        const std::vector<uint32_t>& selected_vertices,
        attribute_paint_mode mode,
        float target,
        float strength,
        float minimum_value = 0.0f,
        float maximum_value = 1.0f);

    bool serialize_attribute_weights(
        const std::vector<float>& weights,
        std::vector<uint8_t>& output);

    bool deserialize_attribute_weights(
        const std::vector<uint8_t>& input,
        std::vector<float>& output);
}
