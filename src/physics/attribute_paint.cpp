#include "skin_physics/physics/attribute_paint.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <iterator>

namespace skin_physics::physics
{
    namespace
    {
        constexpr uint8_t serialization_magic[4] = { 'S', 'P', 'A', 'W' };
        constexpr uint8_t serialization_version = 1;
        constexpr std::size_t serialization_header_size = 13;

        void append_u32_little_endian(std::vector<uint8_t>& output, uint32_t value)
        {
            output.push_back(static_cast<uint8_t>(value & 0xffu));
            output.push_back(static_cast<uint8_t>((value >> 8u) & 0xffu));
            output.push_back(static_cast<uint8_t>((value >> 16u) & 0xffu));
            output.push_back(static_cast<uint8_t>((value >> 24u) & 0xffu));
        }

        uint32_t read_u32_little_endian(const std::vector<uint8_t>& input, std::size_t offset)
        {
            return static_cast<uint32_t>(input[offset]) |
                (static_cast<uint32_t>(input[offset + 1]) << 8u) |
                (static_cast<uint32_t>(input[offset + 2]) << 16u) |
                (static_cast<uint32_t>(input[offset + 3]) << 24u);
        }

        uint32_t checksum(const std::vector<uint8_t>& data, std::size_t offset)
        {
            uint32_t hash = 2166136261u;
            for (std::size_t index = offset; index < data.size(); ++index)
            {
                hash ^= data[index];
                hash *= 16777619u;
            }
            return hash;
        }

        attribute_paint_result make_error(
            attribute_paint_error error,
            const std::string& message)
        {
            attribute_paint_result result;
            result.error = error;
            result.message = message;
            return result;
        }
    }

    attribute_paint_result apply_attribute_paint(
        const std::vector<float>& source_weights,
        const std::vector<std::vector<uint32_t>>& adjacency,
        const std::vector<uint32_t>& selected_vertices,
        attribute_paint_mode mode,
        float target,
        float strength,
        float minimum_value,
        float maximum_value)
    {
        if (source_weights.empty() || adjacency.size() != source_weights.size() ||
            selected_vertices.empty() || !std::isfinite(target) ||
            !std::isfinite(strength) || strength < 0.0f || strength > 1.0f ||
            !std::isfinite(minimum_value) || !std::isfinite(maximum_value) ||
            minimum_value > maximum_value)
        {
            return make_error(attribute_paint_error::invalid_input,
                "Paint inputs, range, adjacency or strength are invalid.");
        }
        for (float weight : source_weights)
        {
            if (!std::isfinite(weight))
            {
                return make_error(attribute_paint_error::non_finite_value,
                    "Source attribute weights must be finite.");
            }
        }

        std::vector<uint8_t> selected(source_weights.size(), 0);
        std::vector<uint32_t> unique_selected_vertices;
        unique_selected_vertices.reserve(selected_vertices.size());
        for (uint32_t vertex_id : selected_vertices)
        {
            if (vertex_id >= source_weights.size())
            {
                return make_error(attribute_paint_error::invalid_vertex,
                    "Selected vertex ID is outside the weight array.");
            }
            if (selected[vertex_id] == 0)
            {
                selected[vertex_id] = 1;
                unique_selected_vertices.push_back(vertex_id);
            }
        }
        for (std::size_t vertex_id = 0; vertex_id < adjacency.size(); ++vertex_id)
        {
            for (uint32_t neighbor : adjacency[vertex_id])
            {
                if (neighbor >= source_weights.size())
                {
                    return make_error(attribute_paint_error::invalid_neighbor,
                        "Adjacency contains an invalid neighbor ID.");
                }
            }
        }

        attribute_paint_result result;
        result.weights = source_weights;
        float selected_average = 0.0f;
        for (uint32_t vertex_id : unique_selected_vertices)
        {
            selected_average += source_weights[vertex_id];
        }
        selected_average /= static_cast<float>(unique_selected_vertices.size());

        for (uint32_t vertex_id : unique_selected_vertices)
        {
            float destination = target;
            float operation_strength = strength;
            if (mode == attribute_paint_mode::smooth ||
                mode == attribute_paint_mode::average)
            {
                destination = selected_average;
                operation_strength = mode == attribute_paint_mode::average ? 1.0f : strength;
            }
            else if (mode == attribute_paint_mode::erase)
            {
                destination = minimum_value;
            }

            const float blended = source_weights[vertex_id] +
                (destination - source_weights[vertex_id]) * operation_strength;
            result.weights[vertex_id] = std::max(minimum_value,
                std::min(maximum_value, blended));
        }

        result.success = true;
        return result;
    }

    bool serialize_attribute_weights(
        const std::vector<float>& weights,
        std::vector<uint8_t>& output)
    {
        static_assert(sizeof(float) == sizeof(uint32_t), "Attribute weight format requires 32-bit float.");
        if (weights.size() > std::numeric_limits<uint32_t>::max())
        {
            return false;
        }
        std::vector<uint8_t> serialized;
        serialized.reserve(serialization_header_size + weights.size() * sizeof(float));
        serialized.insert(serialized.end(), std::begin(serialization_magic), std::end(serialization_magic));
        serialized.push_back(serialization_version);
        append_u32_little_endian(serialized, static_cast<uint32_t>(weights.size()));
        append_u32_little_endian(serialized, 0u);

        for (float weight : weights)
        {
            if (!std::isfinite(weight))
            {
                return false;
            }
            uint32_t bits = 0;
            std::memcpy(&bits, &weight, sizeof(float));
            append_u32_little_endian(serialized, bits);
        }

        const uint32_t hash = checksum(serialized, serialization_header_size);
        serialized[9] = static_cast<uint8_t>(hash & 0xffu);
        serialized[10] = static_cast<uint8_t>((hash >> 8u) & 0xffu);
        serialized[11] = static_cast<uint8_t>((hash >> 16u) & 0xffu);
        serialized[12] = static_cast<uint8_t>((hash >> 24u) & 0xffu);
        output.swap(serialized);
        return true;
    }

    bool deserialize_attribute_weights(
        const std::vector<uint8_t>& input,
        std::vector<float>& output)
    {
        static_assert(sizeof(float) == sizeof(uint32_t), "Attribute weight format requires 32-bit float.");
        if (input.size() < serialization_header_size ||
            !std::equal(std::begin(serialization_magic), std::end(serialization_magic), input.begin()) ||
            input[4] != serialization_version)
        {
            return false;
        }

        const uint32_t count = read_u32_little_endian(input, 5);
        const std::size_t expected_size =
            serialization_header_size + static_cast<std::size_t>(count) * sizeof(float);
        if (expected_size != input.size())
        {
            return false;
        }
        const uint32_t stored_checksum = read_u32_little_endian(input, 9);
        std::vector<uint8_t> check_data = input;
        check_data[9] = 0;
        check_data[10] = 0;
        check_data[11] = 0;
        check_data[12] = 0;
        if (checksum(check_data, serialization_header_size) != stored_checksum)
        {
            return false;
        }

        std::vector<float> decoded;
        decoded.reserve(count);
        for (uint32_t index = 0; index < count; ++index)
        {
            const uint32_t bits = read_u32_little_endian(
                input, serialization_header_size + static_cast<std::size_t>(index) * sizeof(float));
            float weight = 0.0f;
            std::memcpy(&weight, &bits, sizeof(float));
            if (!std::isfinite(weight))
            {
                return false;
            }
            decoded.push_back(weight);
        }
        output.swap(decoded);
        return true;
    }
}
