#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "skin_physics/runtime/physics_runtime.hpp"

#include "skin_physics/asset/importer.hpp"
#include "skin_physics/physics/animation_track.hpp"
#include "skin_physics/physics/animation_gpu_resources.hpp"
#include "skin_physics/physics/skin_weights.hpp"
#include "skin_physics/simulation/d3d11_status_ring.hpp"
#include "skin_physics/simulation/d3d11_compute_graph.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <d3dcompiler.h>
#include <windows.h>

#pragma comment(lib, "d3dcompiler.lib")

namespace skin_physics::runtime
{
    namespace
    {
        constexpr double time_epsilon = 1.0e-12;
        constexpr float vector_epsilon_squared = 1.0e-16f;
        constexpr uint32_t minimum_status_ring_capacity = 2u;

        struct gpu_tick_status_record
        {
            uint32_t terminal_state = 0;
            uint32_t terminal_failure = 0;
            uint64_t committed_tick_sequence = 0;
            uint64_t committed_state_generation = 0;
            uint32_t active_state_set = 0;
            uint32_t diagnostic_code = 0;
            uint64_t dispatch_id = 0;
            uint64_t reservation_token = 0;
            uint32_t validation_mask = 0;
            uint32_t required_validation_mask = 0;
            uint32_t work_budget_block_flags = 0;
            uint32_t work_budget_block_origin = 0;
            uint32_t executed_integration_segments = 0;
            uint32_t minimum_required_integration_segments = 0;
            uint32_t reserved[2] = {};
        };

        static_assert(sizeof(gpu_tick_status_record) == 80,
            "GPU Tick Status ABI must remain 80 bytes.");

        struct alignas(16) gpu_tick_dispatch_plan
        {
            uint32_t expected_predecessor_sequence_low = 0u;
            uint32_t expected_predecessor_sequence_high = 0u;
            uint32_t expected_source_generation_low = 0u;
            uint32_t expected_source_generation_high = 0u;
            uint32_t expected_source_state_set = 0u;
            uint32_t destination_state_set = 0u;
            uint32_t tick_sequence_low = 0u;
            uint32_t tick_sequence_high = 0u;
            uint32_t dispatch_id_low = 0u;
            uint32_t dispatch_id_high = 0u;
            uint32_t reservation_token_low = 0u;
            uint32_t reservation_token_high = 0u;
            uint32_t required_validation_mask = 0u;
            uint32_t flags = 0u;
            uint32_t reserved[2] = {};
        };

        struct alignas(16) gpu_tick_commit_control
        {
            uint32_t committed_sequence_low = 0u;
            uint32_t committed_sequence_high = 0u;
            uint32_t state_generation_low = 1u;
            uint32_t state_generation_high = 0u;
            uint32_t active_state_set = 0u;
            uint32_t terminal_state = 0u;
            uint32_t terminal_failure = 0u;
            uint32_t validation_mask = 0u;
            uint32_t required_validation_mask = 1u;
            uint32_t dispatch_id_low = 0u;
            uint32_t dispatch_id_high = 0u;
            uint32_t reservation_token_low = 0u;
            uint32_t reservation_token_high = 0u;
            uint32_t last_gate_state = 0u;
            uint32_t diagnostic = 0u;
            uint32_t render_slot_id = asset::invalid_index;
            uint32_t render_slot_generation = 0u;
            uint32_t reserved[3] = {};
        };

        static_assert(sizeof(gpu_tick_dispatch_plan) == 64,
            "GPU Tick Dispatch Plan ABI must remain 64 bytes.");
        static_assert(sizeof(gpu_tick_commit_control) == 80,
            "GPU Tick Commit Control ABI must remain 80 bytes.");

        struct alignas(16) gpu_float4
        {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            float w = 0.0f;
        };

        static_assert(sizeof(gpu_float4) == 16,
            "GPU float4 ABI must remain 16 bytes.");

        struct gpu_soft_body_skin_weight_record
        {
            uint32_t joints[asset::max_skin_influences] = {};
            float weights[asset::max_skin_influences] = {};
        };

        static_assert(sizeof(gpu_soft_body_skin_weight_record) == 64u,
            "GPU Physics Vertex Skin Weight records must remain 64 bytes.");

        struct alignas(16) gpu_soft_body_tet_input
        {
            uint32_t vertex_ids[4] = {};
            float rest_inverse[3][4] = {};
            float rest_gradients[4][4] = {};
            float material0[4] = {};
            float material1[4] = {};
        };

        struct alignas(16) gpu_soft_body_tet_output
        {
            float values[14][4] = {};
        };

        struct alignas(16) gpu_soft_body_fem_parameters
        {
            uint32_t tetrahedron_count = 0u;
            uint32_t reserved[3] = {};
        };

        struct alignas(16) gpu_soft_body_gather_parameters
        {
            uint32_t vertex_count = 0u;
            uint32_t reserved[3] = {};
        };

        struct alignas(16) gpu_soft_body_integration_parameters
        {
            uint32_t vertex_count = 0u;
            float delta_time = 0.0f;
            float damping_rate = 0.0f;
            uint32_t has_impulse_events = 0u;
            float gravity[3] = {};
            float tracking_stiffness = 0.0f;
            float tracking_damping = 0.0f;
            float tracking_attribute_multiplier = 0.0f;
            uint32_t has_animation_target = 0u;
            uint32_t reserved2 = 0u;
        };

        struct alignas(16) gpu_soft_body_skinning_parameters
        {
            uint32_t vertex_count = 0u;
            uint32_t joint_count = 0u;
            uint32_t reserved[2] = {};
        };

        static_assert(sizeof(gpu_soft_body_integration_parameters) == 48u,
            "Soft Body Integration Parameters must remain a 48-byte HLSL cbuffer.");
        static_assert(sizeof(gpu_soft_body_skinning_parameters) == 16u,
            "Soft Body Skinning Parameters must remain a 16-byte HLSL cbuffer.");

        struct alignas(16) gpu_soft_body_distribution_reduce_parameters
        {
            uint32_t vertex_count = 0u;
            uint32_t group_count = 0u;
            uint32_t partial_sum_offset = 0u;
            uint32_t reserved = 0u;
        };

        struct alignas(16) gpu_soft_body_force_distribution_parameters
        {
            uint32_t vertex_count = 0u;
            uint32_t include_previous = 0u;
            uint32_t reserved[2] = {};
            float action_position[3] = {};
            float radius = 0.0f;
            float direction[3] = {};
            float magnitude = 0.0f;
        };

        static_assert(sizeof(gpu_soft_body_distribution_reduce_parameters) == 16,
            "Soft Body distribution reduction parameters must remain 16 bytes.");
        static_assert(sizeof(gpu_soft_body_force_distribution_parameters) == 48,
            "Soft Body Force / Impulse parameters must remain 48 bytes.");

        struct alignas(16) gpu_soft_body_render_binding_input
        {
            uint32_t physics_vertices[4] = {};
            float barycentric[4] = {};
            float rest_offset_local[4] = {};
            uint32_t metadata[4] = {};
        };

        struct alignas(16) gpu_soft_body_render_vertex_input
        {
            float texcoord[4] = {};
            float fallback_normal[4] = {};
            float fallback_tangent[4] = {};
            uint32_t metadata[4] = {};
        };

        struct alignas(16) gpu_soft_body_render_parameters
        {
            uint32_t render_vertex_count = 0u;
            uint32_t render_index_count = 0u;
            uint32_t reserved[2] = {};
            float root_rotation[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
            float inverse_root_rotation[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
            float inverse_root_translation[4] = {};
        };

        static_assert(sizeof(gpu_soft_body_render_binding_input) == 64,
            "Soft Body Render Binding ABI must remain 64 bytes.");
        static_assert(sizeof(gpu_soft_body_render_vertex_input) == 64,
            "Soft Body Render Vertex ABI must remain 64 bytes.");
        static_assert(sizeof(gpu_soft_body_render_parameters) == 64,
            "Soft Body Render Parameters ABI must remain 64 bytes.");

        static_assert(sizeof(gpu_soft_body_tet_input) == 160,
            "Soft Body Tet Input ABI must remain 160 bytes.");
        static_assert(sizeof(gpu_soft_body_tet_output) == 224,
            "Soft Body Tet Output ABI must remain 224 bytes.");
        static_assert(sizeof(gpu_soft_body_fem_parameters) == 16,
            "Soft Body FEM Parameters ABI must remain 16 bytes.");
        static_assert(sizeof(gpu_soft_body_gather_parameters) == 16,
            "Soft Body Gather Parameters ABI must remain 16 bytes.");
        static_assert(sizeof(gpu_soft_body_integration_parameters) == 48,
            "Soft Body Integration Parameters ABI must remain 48 bytes.");

        struct transform_sample
        {
            double time = 0.0;
            transform value;
            bool is_teleport = false;
        };

        bool finite_float(float value)
        {
            return std::isfinite(value);
        }

        bool finite_double(double value)
        {
            return std::isfinite(value);
        }

        std::string format_double_round_trip(double value)
        {
            std::ostringstream stream;
            stream.precision(std::numeric_limits<double>::max_digits10);
            stream << value;
            return stream.str();
        }

        bool finite_vec3(const asset::vec3& value)
        {
            return finite_float(value.x) &&
                finite_float(value.y) &&
                finite_float(value.z);
        }

        bool finite_vec4(const asset::vec4& value)
        {
            return finite_float(value.x) &&
                finite_float(value.y) &&
                finite_float(value.z) &&
                finite_float(value.w);
        }

        bool normalize_direction(asset::vec3& value)
        {
            if (!finite_vec3(value))
            {
                return false;
            }

            const double x = static_cast<double>(value.x);
            const double y = static_cast<double>(value.y);
            const double z = static_cast<double>(value.z);
            const double length_squared = x * x + y * y + z * z;
            if (!std::isfinite(length_squared) || length_squared <= 0.0)
            {
                return false;
            }

            const double inverse_length = 1.0 / std::sqrt(length_squared);
            if (!std::isfinite(inverse_length) || inverse_length <= 0.0)
            {
                return false;
            }

            value.x = static_cast<float>(x * inverse_length);
            value.y = static_cast<float>(y * inverse_length);
            value.z = static_cast<float>(z * inverse_length);
            if (!finite_vec3(value))
            {
                return false;
            }

            const double normalized_x = static_cast<double>(value.x);
            const double normalized_y = static_cast<double>(value.y);
            const double normalized_z = static_cast<double>(value.z);
            const double normalized_length_squared =
                normalized_x * normalized_x +
                normalized_y * normalized_y +
                normalized_z * normalized_z;
            return std::isfinite(normalized_length_squared) &&
                std::abs(normalized_length_squared - 1.0) <= 1.0e-5;
        }

        bool valid_transform(const transform& value)
        {
            if (!finite_vec3(value.translation) || !finite_vec4(value.rotation))
            {
                return false;
            }

            const double x = value.rotation.x;
            const double y = value.rotation.y;
            const double z = value.rotation.z;
            const double w = value.rotation.w;
            const double length_squared = x * x + y * y + z * z + w * w;
            return std::isfinite(length_squared) &&
                std::abs(length_squared - 1.0) <= 1.0e-3;
        }

        bool normalize_transform_rotation(transform& value)
        {
            if (!valid_transform(value))
            {
                return false;
            }

            const double x = value.rotation.x;
            const double y = value.rotation.y;
            const double z = value.rotation.z;
            const double w = value.rotation.w;
            const double inverse_length = 1.0 / std::sqrt(
                x * x + y * y + z * z + w * w);
            if (!std::isfinite(inverse_length) || inverse_length <= 0.0)
            {
                return false;
            }

            value.rotation.x = static_cast<float>(x * inverse_length);
            value.rotation.y = static_cast<float>(y * inverse_length);
            value.rotation.z = static_cast<float>(z * inverse_length);
            value.rotation.w = static_cast<float>(w * inverse_length);
            return finite_vec4(value.rotation);
        }

        bool same_com_identity(IUnknown* left, IUnknown* right)
        {
            if (left == nullptr || right == nullptr)
            {
                return false;
            }

            Microsoft::WRL::ComPtr<IUnknown> left_identity;
            Microsoft::WRL::ComPtr<IUnknown> right_identity;
            if (FAILED(left->QueryInterface(
                    IID_PPV_ARGS(left_identity.GetAddressOf()))) ||
                FAILED(right->QueryInterface(
                    IID_PPV_ARGS(right_identity.GetAddressOf()))))
            {
                return false;
            }
            return left_identity.Get() == right_identity.Get();
        }

        status make_status(
            status_code code,
            const std::string& diagnostic,
            int32_t native_code = 0)
        {
            status result;
            result.code = code;
            result.native_code = native_code;
            result.diagnostic = diagnostic;
            return result;
        }

        uint32_t next_power_of_two(uint32_t value)
        {
            if (value <= 1u)
            {
                return 1u;
            }
            --value;
            value |= value >> 1u;
            value |= value >> 2u;
            value |= value >> 4u;
            value |= value >> 8u;
            value |= value >> 16u;
            if (value == std::numeric_limits<uint32_t>::max())
            {
                return 0u;
            }
            return value + 1u;
        }

        bool create_structured_buffer(
            ID3D11Device* device,
            uint32_t byte_width,
            uint32_t structure_stride,
            UINT bind_flags,
            const void* initial_data,
            ID3D11Buffer** out_buffer)
        {
            if (device == nullptr || out_buffer == nullptr ||
                byte_width == 0u || structure_stride == 0u ||
                (byte_width % structure_stride) != 0u)
            {
                return false;
            }

            D3D11_BUFFER_DESC description = {};
            description.ByteWidth = byte_width;
            description.Usage = D3D11_USAGE_DEFAULT;
            description.BindFlags = bind_flags;
            description.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
            description.StructureByteStride = structure_stride;

            D3D11_SUBRESOURCE_DATA initial = {};
            initial.pSysMem = initial_data;
            return SUCCEEDED(device->CreateBuffer(
                &description,
                initial_data != nullptr ? &initial : nullptr,
                out_buffer));
        }

        template <typename T>
        bool create_structured_buffer_from_vector(
            ID3D11Device* device,
            const std::vector<T>& values,
            UINT bind_flags,
            ID3D11Buffer** out_buffer)
        {
            static_assert(std::is_trivially_copyable<T>::value,
                "Structured GPU buffer elements must be trivially copyable.");
            if (values.empty() ||
                values.size() >
                    static_cast<std::size_t>(
                        std::numeric_limits<uint32_t>::max() / sizeof(T)))
            {
                return false;
            }

            const uint32_t byte_width = static_cast<uint32_t>(
                values.size() * sizeof(T));
            return create_structured_buffer(
                device,
                byte_width,
                static_cast<uint32_t>(sizeof(T)),
                bind_flags,
                values.data(),
                out_buffer);
        }

        bool create_constant_buffer(
            ID3D11Device* device,
            uint32_t byte_width,
            const void* initial_data,
            ID3D11Buffer** out_buffer)
        {
            if (device == nullptr || out_buffer == nullptr ||
                initial_data == nullptr || byte_width == 0u ||
                (byte_width & 15u) != 0u ||
                byte_width > D3D11_REQ_CONSTANT_BUFFER_ELEMENT_COUNT * 16u)
            {
                return false;
            }

            D3D11_BUFFER_DESC description = {};
            description.ByteWidth = byte_width;
            description.Usage = D3D11_USAGE_DEFAULT;
            description.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            description.CPUAccessFlags = 0u;
            description.MiscFlags = 0u;

            D3D11_SUBRESOURCE_DATA initial = {};
            initial.pSysMem = initial_data;
            return SUCCEEDED(device->CreateBuffer(
                &description, &initial, out_buffer));
        }

        asset::mat4 make_root_transform_matrix(const transform& root)
        {
            const double x = root.rotation.x;
            const double y = root.rotation.y;
            const double z = root.rotation.z;
            const double w = root.rotation.w;
            asset::mat4 matrix;
            matrix.value = {
                static_cast<float>(1.0 - 2.0 * (y * y + z * z)),
                static_cast<float>(2.0 * (x * y + z * w)),
                static_cast<float>(2.0 * (x * z - y * w)),
                0.0f,
                static_cast<float>(2.0 * (x * y - z * w)),
                static_cast<float>(1.0 - 2.0 * (x * x + z * z)),
                static_cast<float>(2.0 * (y * z + x * w)),
                0.0f,
                static_cast<float>(2.0 * (x * z + y * w)),
                static_cast<float>(2.0 * (y * z - x * w)),
                static_cast<float>(1.0 - 2.0 * (x * x + y * y)),
                0.0f,
                root.translation.x,
                root.translation.y,
                root.translation.z,
                1.0f
            };
            return matrix;
        }

        asset::mat4 make_root_transform_derivative_matrix(
            const transform& root,
            const asset::vec4& rotation_derivative,
            const asset::vec3& translation_derivative)
        {
            const double x = root.rotation.x;
            const double y = root.rotation.y;
            const double z = root.rotation.z;
            const double w = root.rotation.w;
            const double dx = rotation_derivative.x;
            const double dy = rotation_derivative.y;
            const double dz = rotation_derivative.z;
            const double dw = rotation_derivative.w;

            asset::mat4 matrix;
            matrix.value = {
                static_cast<float>(-4.0 * (y * dy + z * dz)),
                static_cast<float>(2.0 * (dx * y + x * dy + dz * w + z * dw)),
                static_cast<float>(2.0 * (dx * z + x * dz - dy * w - y * dw)),
                0.0f,
                static_cast<float>(2.0 * (dx * y + x * dy - dz * w - z * dw)),
                static_cast<float>(-4.0 * (x * dx + z * dz)),
                static_cast<float>(2.0 * (dy * z + y * dz + dx * w + x * dw)),
                0.0f,
                static_cast<float>(2.0 * (dx * z + x * dz + dy * w + y * dw)),
                static_cast<float>(2.0 * (dy * z + y * dz - dx * w - x * dw)),
                static_cast<float>(-4.0 * (x * dx + y * dy)),
                0.0f,
                translation_derivative.x,
                translation_derivative.y,
                translation_derivative.z,
                0.0f
            };
            return matrix;
        }

        bool evaluate_root_transform_samples(
            const std::vector<transform_sample>& samples,
            const transform& fallback_transform,
            double world_time,
            transform& out_transform,
            asset::mat4& out_derivative,
            std::string& diagnostic)
        {
            diagnostic.clear();
            out_derivative.value.fill(0.0f);
            if (!finite_double(world_time) ||
                !valid_transform(fallback_transform))
            {
                diagnostic =
                    "Root Transform evaluation received a non-finite time or invalid fallback Transform.";
                return false;
            }

            if (samples.empty())
            {
                out_transform = fallback_transform;
                return true;
            }

            for (std::size_t index = 0u; index < samples.size(); ++index)
            {
                if (!finite_double(samples[index].time) ||
                    samples[index].time < 0.0 ||
                    !valid_transform(samples[index].value) ||
                    (index > 0u &&
                        samples[index].time <= samples[index - 1u].time))
                {
                    diagnostic =
                        "Immutable Root Transform Samples are not finite, normalized and strictly time ordered.";
                    return false;
                }
            }

            const auto upper = std::upper_bound(
                samples.begin(), samples.end(), world_time,
                [](double time, const transform_sample& sample)
                {
                    return time < sample.time;
                });
            if (upper == samples.begin())
            {
                out_transform = fallback_transform;
                return true;
            }

            const transform_sample& start = *(upper - 1);
            if (world_time == start.time && start.is_teleport)
            {
                out_transform = start.value;
                return true;
            }
            if (upper == samples.end() || upper->is_teleport)
            {
                out_transform = start.value;
                return true;
            }

            const transform_sample& end = *upper;
            const double duration = end.time - start.time;
            const double normalized =
                (world_time - start.time) / duration;
            if (!finite_double(duration) || duration <= 0.0 ||
                !finite_double(normalized) ||
                normalized < 0.0 || normalized > 1.0)
            {
                diagnostic =
                    "Root Transform Sample interval cannot be represented as a finite positive interpolation span.";
                return false;
            }

            const double inverse_duration = 1.0 / duration;
            out_transform.translation = {
                static_cast<float>(
                    static_cast<double>(start.value.translation.x) +
                    (static_cast<double>(end.value.translation.x) -
                        start.value.translation.x) * normalized),
                static_cast<float>(
                    static_cast<double>(start.value.translation.y) +
                    (static_cast<double>(end.value.translation.y) -
                        start.value.translation.y) * normalized),
                static_cast<float>(
                    static_cast<double>(start.value.translation.z) +
                    (static_cast<double>(end.value.translation.z) -
                        start.value.translation.z) * normalized)
            };
            const asset::vec3 translation_derivative = {
                static_cast<float>(
                    (static_cast<double>(end.value.translation.x) -
                        start.value.translation.x) * inverse_duration),
                static_cast<float>(
                    (static_cast<double>(end.value.translation.y) -
                        start.value.translation.y) * inverse_duration),
                static_cast<float>(
                    (static_cast<double>(end.value.translation.z) -
                        start.value.translation.z) * inverse_duration)
            };

            std::array<double, 4> q0 = {
                start.value.rotation.x, start.value.rotation.y,
                start.value.rotation.z, start.value.rotation.w
            };
            std::array<double, 4> q1 = {
                end.value.rotation.x, end.value.rotation.y,
                end.value.rotation.z, end.value.rotation.w
            };
            double cosine = 0.0;
            for (uint32_t component = 0u; component < 4u; ++component)
            {
                cosine += q0[component] * q1[component];
            }
            if (cosine < 0.0)
            {
                cosine = -cosine;
                for (double& component : q1)
                {
                    component = -component;
                }
            }
            cosine = std::max(-1.0, std::min(1.0, cosine));

            std::array<double, 4> q = {};
            std::array<double, 4> q_derivative = {};
            if (cosine > 0.9995)
            {
                std::array<double, 4> raw = {};
                std::array<double, 4> raw_derivative = {};
                double squared_length = 0.0;
                for (uint32_t component = 0u; component < 4u; ++component)
                {
                    raw[component] =
                        q0[component] * (1.0 - normalized) +
                        q1[component] * normalized;
                    raw_derivative[component] =
                        (q1[component] - q0[component]) * inverse_duration;
                    squared_length += raw[component] * raw[component];
                }
                if (!std::isfinite(squared_length) ||
                    squared_length <= 1.0e-24)
                {
                    diagnostic =
                        "Root Transform near-linear Quaternion interpolation produced a zero-length value.";
                    return false;
                }
                const double raw_length = std::sqrt(squared_length);
                double projection = 0.0;
                for (uint32_t component = 0u; component < 4u; ++component)
                {
                    q[component] = raw[component] / raw_length;
                    projection += q[component] * raw_derivative[component];
                }
                for (uint32_t component = 0u; component < 4u; ++component)
                {
                    q_derivative[component] =
                        (raw_derivative[component] -
                            q[component] * projection) / raw_length;
                }
            }
            else
            {
                const double angle = std::acos(cosine);
                const double sine = std::sin(angle);
                if (!std::isfinite(sine) || std::abs(sine) <= 1.0e-12)
                {
                    diagnostic =
                        "Root Transform Quaternion Slerp denominator is unresolved.";
                    return false;
                }
                const double angle_u = angle * normalized;
                const double cosine_u = std::cos(angle_u);
                const double sine_u = std::sin(angle_u);
                for (uint32_t component = 0u; component < 4u; ++component)
                {
                    const double tangent =
                        (q1[component] - cosine * q0[component]) / sine;
                    q[component] =
                        q0[component] * cosine_u +
                        tangent * sine_u;
                    q_derivative[component] =
                        angle * inverse_duration *
                        (-q0[component] * sine_u +
                            tangent * cosine_u);
                }
            }

            out_transform.rotation = {
                static_cast<float>(q[0]),
                static_cast<float>(q[1]),
                static_cast<float>(q[2]),
                static_cast<float>(q[3])
            };
            const asset::vec4 rotation_derivative = {
                static_cast<float>(q_derivative[0]),
                static_cast<float>(q_derivative[1]),
                static_cast<float>(q_derivative[2]),
                static_cast<float>(q_derivative[3])
            };
            if (!normalize_transform_rotation(out_transform) ||
                !finite_vec3(translation_derivative) ||
                !finite_vec4(rotation_derivative))
            {
                diagnostic =
                    "Root Transform interpolation or analytic derivative exceeded finite float range.";
                return false;
            }

            out_derivative = make_root_transform_derivative_matrix(
                out_transform, rotation_derivative, translation_derivative);
            if (!std::all_of(
                    out_derivative.value.begin(),
                    out_derivative.value.end(),
                    [](float value) { return std::isfinite(value); }))
            {
                diagnostic =
                    "Root Transform analytic derivative matrix is non-finite.";
                return false;
            }
            return true;
        }

        asset::mat4 multiply_asset_matrices(
            const asset::mat4& left,
            const asset::mat4& right)
        {
            asset::mat4 result;
            result.value.fill(0.0f);
            for (uint32_t column = 0u; column < 4u; ++column)
            {
                for (uint32_t row = 0u; row < 4u; ++row)
                {
                    double value = 0.0;
                    for (uint32_t inner = 0u; inner < 4u; ++inner)
                    {
                        value += static_cast<double>(
                            left.value[inner * 4u + row]) *
                            right.value[column * 4u + inner];
                    }
                    result.value[column * 4u + row] =
                        static_cast<float>(value);
                }
            }
            return result;
        }

        physics::gpu_matrix4x4 to_gpu_matrix(const asset::mat4& source)
        {
            physics::gpu_matrix4x4 result;
            result.column0 = {
                source.value[0], source.value[1],
                source.value[2], source.value[3]
            };
            result.column1 = {
                source.value[4], source.value[5],
                source.value[6], source.value[7]
            };
            result.column2 = {
                source.value[8], source.value[9],
                source.value[10], source.value[11]
            };
            result.column3 = {
                source.value[12], source.value[13],
                source.value[14], source.value[15]
            };
            return result;
        }

        bool finite_asset_matrix(const asset::mat4& matrix)
        {
            return std::all_of(
                matrix.value.begin(), matrix.value.end(),
                [](float value) { return finite_float(value); });
        }

        asset::vec3 transform_position(
            const transform& root,
            const asset::vec3& local_position)
        {
            const asset::vec3 quaternion = {
                root.rotation.x, root.rotation.y, root.rotation.z
            };
            const asset::vec3 cross0 = {
                quaternion.y * local_position.z - quaternion.z * local_position.y,
                quaternion.z * local_position.x - quaternion.x * local_position.z,
                quaternion.x * local_position.y - quaternion.y * local_position.x
            };
            const asset::vec3 cross1 = {
                quaternion.y * cross0.z - quaternion.z * cross0.y,
                quaternion.z * cross0.x - quaternion.x * cross0.z,
                quaternion.x * cross0.y - quaternion.y * cross0.x
            };

            return {
                local_position.x + 2.0f *
                    (root.rotation.w * cross0.x + cross1.x) +
                    root.translation.x,
                local_position.y + 2.0f *
                    (root.rotation.w * cross0.y + cross1.y) +
                    root.translation.y,
                local_position.z + 2.0f *
                    (root.rotation.w * cross0.z + cross1.z) +
                    root.translation.z
            };
        }

        bool create_structured_srv(
            ID3D11Device* device,
            ID3D11Buffer* buffer,
            uint32_t element_count,
            ID3D11ShaderResourceView** out_view)
        {
            if (device == nullptr || buffer == nullptr ||
                out_view == nullptr || element_count == 0u)
            {
                return false;
            }

            D3D11_SHADER_RESOURCE_VIEW_DESC description = {};
            description.Format = DXGI_FORMAT_UNKNOWN;
            description.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
            description.Buffer.FirstElement = 0u;
            description.Buffer.NumElements = element_count;
            return SUCCEEDED(device->CreateShaderResourceView(
                buffer, &description, out_view));
        }

        bool create_structured_uav(
            ID3D11Device* device,
            ID3D11Buffer* buffer,
            uint32_t element_count,
            ID3D11UnorderedAccessView** out_view)
        {
            if (device == nullptr || buffer == nullptr ||
                out_view == nullptr || element_count == 0u)
            {
                return false;
            }

            D3D11_UNORDERED_ACCESS_VIEW_DESC description = {};
            description.Format = DXGI_FORMAT_UNKNOWN;
            description.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
            description.Buffer.FirstElement = 0u;
            description.Buffer.NumElements = element_count;
            return SUCCEEDED(device->CreateUnorderedAccessView(
                buffer, &description, out_view));
        }

        bool compile_runtime_compute_shader(
            ID3D11Device* device,
            const std::filesystem::path& shader_path,
            const char* entry_point,
            ID3D11ComputeShader** out_shader,
            std::string& diagnostic)
        {
            diagnostic.clear();
            if (device == nullptr || entry_point == nullptr ||
                out_shader == nullptr ||
                !std::filesystem::is_regular_file(shader_path))
            {
                diagnostic = "Runtime Compute Shader source file does not exist.";
                return false;
            }

            Microsoft::WRL::ComPtr<ID3DBlob> bytecode;
            Microsoft::WRL::ComPtr<ID3DBlob> errors;
            const HRESULT compile_result = D3DCompileFromFile(
                shader_path.c_str(),
                nullptr,
                D3D_COMPILE_STANDARD_FILE_INCLUDE,
                entry_point,
                "cs_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS,
                0u,
                bytecode.GetAddressOf(),
                errors.GetAddressOf());

            if (FAILED(compile_result))
            {
                if (errors)
                {
                    diagnostic.assign(
                        static_cast<const char*>(errors->GetBufferPointer()),
                        errors->GetBufferSize());
                }
                else
                {
                    diagnostic = "D3DCompileFromFile failed without an error blob.";
                }
                return false;
            }

            const HRESULT shader_result = device->CreateComputeShader(
                bytecode->GetBufferPointer(),
                bytecode->GetBufferSize(),
                nullptr,
                out_shader);
            if (FAILED(shader_result))
            {
                diagnostic = "D3D11 CreateComputeShader failed for a compiled Runtime shader.";
                return false;
            }
            return true;
        }

        std::filesystem::path resolve_shader_package_root(
            const std::string& configured_root,
            std::string& diagnostic)
        {
            diagnostic.clear();
            std::filesystem::path root(configured_root);
            if (root.is_absolute())
            {
                return root.lexically_normal();
            }

            std::vector<wchar_t> module_path(32768u, L'\0');
            const DWORD length = GetModuleFileNameW(
                nullptr,
                module_path.data(),
                static_cast<DWORD>(module_path.size()));
            if (length == 0u || length >= module_path.size())
            {
                diagnostic = "Unable to resolve the Runtime module directory for a Package-relative Shader Root.";
                return {};
            }
            module_path.resize(length);
            const std::filesystem::path executable_path(module_path.begin(), module_path.end());
            return (executable_path.parent_path() / root).lexically_normal();
        }

        bool valid_animation_track(
            const asset::model& model,
            const animation_track_desc& track)
        {
            return track.clip_index < model.animations.size() &&
                finite_double(track.time) &&
                finite_double(track.speed);
        }

        bool valid_blend_weight_curve(const animation_blend_desc& desc)
        {
            physics::animation_blend_weight_curve_desc curve;
            curve.blend_start_time = desc.blend_start_time;
            curve.blend_end_time = desc.blend_end_time;
            curve.weight_start = desc.weight_start;
            curve.weight_end = desc.weight_end;
            curve.tangent_start = desc.tangent_start;
            curve.tangent_end = desc.tangent_end;
            return physics::valid_animation_blend_weight_curve(curve);
        }

        bool valid_world_desc(
            const world_desc& desc,
            const runtime_desc& runtime)
        {
            if (!finite_double(desc.fixed_delta_time) ||
                desc.fixed_delta_time <= 0.0 ||
                !finite_double(desc.time_scale) ||
                desc.time_scale < 0.0 ||
                !finite_vec3(desc.gravity) ||
                (desc.maximum_in_flight_ticks != 1u &&
                    desc.maximum_in_flight_ticks != 2u &&
                    desc.maximum_in_flight_ticks != 4u) ||
                desc.minimum_substeps == 0u ||
                desc.maximum_substeps < desc.minimum_substeps ||
                desc.maximum_substeps > runtime.maximum_world_substeps_hard_limit ||
                desc.maximum_integration_segments_per_tick == 0u ||
                desc.maximum_integration_segments_per_tick >
                    runtime.maximum_integration_segments_per_tick_hard_limit ||
                !finite_double(desc.world_max_substep_delta_time) ||
                desc.world_max_substep_delta_time < 0.0 ||
                !finite_double(desc.contact_max_substep_delta_time) ||
                desc.contact_max_substep_delta_time < 0.0 ||
                desc.contact_impulse_iterations < 1u ||
                desc.contact_impulse_iterations > 16u ||
                !finite_float(desc.contact_impulse_relaxation) ||
                desc.contact_impulse_relaxation <= 0.0f ||
                desc.contact_impulse_relaxation > 1.0f ||
                !finite_float(desc.contact_impulse_residual_tolerance) ||
                desc.contact_impulse_residual_tolerance <= 0.0f ||
                desc.maximum_steps_per_advance == 0u)
            {
                return false;
            }

            if (desc.enable_cloth_temporal_quality_limit &&
                (!finite_float(desc.cloth_temporal_quality_factor) ||
                    desc.cloth_temporal_quality_factor <= 0.0f))
            {
                return false;
            }
            return true;
        }

        bool valid_runtime_desc(const runtime_desc& desc)
        {
            return !desc.shader_package_root.empty() &&
                desc.maximum_world_substeps_hard_limit > 0u &&
                desc.maximum_integration_segments_per_tick_hard_limit > 0u &&
                desc.query_ring_capacity >= minimum_status_ring_capacity;
        }

        bool valid_collider_desc(const collider_desc& desc)
        {
            if (desc.collision_mesh.vertices.empty() ||
                desc.collision_mesh.indices.empty() ||
                (desc.collision_mesh.indices.size() % 3u) != 0u ||
                !valid_transform(desc.initial_transform) ||
                !finite_float(desc.thickness) ||
                desc.thickness < 0.0f ||
                !finite_float(desc.friction) ||
                desc.friction < 0.0f ||
                desc.friction > 1.0f ||
                !finite_float(desc.restitution) ||
                desc.restitution < 0.0f ||
                desc.restitution > 1.0f)
            {
                return false;
            }

            for (const asset::vertex& vertex : desc.collision_mesh.vertices)
            {
                if (!finite_vec3(vertex.position))
                {
                    return false;
                }
            }
            for (uint32_t index : desc.collision_mesh.indices)
            {
                if (index >= desc.collision_mesh.vertices.size())
                {
                    return false;
                }
            }
            return true;
        }

        bool valid_mesh_physics_parameters(const mesh_physics_parameters& parameters)
        {
            if (!finite_float(parameters.density) ||
                parameters.density <= 0.0f ||
                !finite_float(parameters.young_modulus) ||
                parameters.young_modulus <= 0.0f ||
                !finite_float(parameters.poisson_ratio) ||
                parameters.poisson_ratio <= -1.0f ||
                parameters.poisson_ratio >= 0.5f ||
                !finite_float(parameters.damping) ||
                parameters.damping < 0.0f ||
                !finite_float(parameters.tracking_stiffness) ||
                parameters.tracking_stiffness < 0.0f ||
                !finite_float(parameters.tracking_damping) ||
                parameters.tracking_damping < 0.0f)
            {
                return false;
            }
            return std::all_of(
                parameters.attribute_multipliers.begin(),
                parameters.attribute_multipliers.end(),
                [](float value)
                {
                    return finite_float(value) && value >= 0.0f;
                });
        }

        bool valid_mesh_asset(const physics::physics_mesh& mesh)
        {
            const std::size_t vertex_count = mesh.rest_positions.size();
            const std::size_t tetrahedron_count = mesh.tetrahedra.size();
            if (vertex_count == 0u ||
                vertex_count >= static_cast<std::size_t>(asset::invalid_index) ||
                tetrahedron_count == 0u ||
                tetrahedron_count >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) ||
                mesh.vertex_tetrahedron_ids.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) ||
                mesh.mass.size() != vertex_count ||
                mesh.inverse_mass.size() != vertex_count ||
                mesh.vertex_tetrahedron_offsets.size() != vertex_count + 1u ||
                mesh.vertex_tetrahedron_offsets.front() != 0u ||
                mesh.vertex_tetrahedron_offsets.back() !=
                    mesh.vertex_tetrahedron_ids.size())
            {
                return false;
            }

            for (const asset::vec3& position : mesh.rest_positions)
            {
                if (!finite_vec3(position))
                {
                    return false;
                }
            }
            if (!mesh.vertex_attribute_weights.empty() &&
                mesh.vertex_attribute_weights.size() != vertex_count)
            {
                return false;
            }
            for (const physics::physics_vertex_attribute_weights& weights :
                mesh.vertex_attribute_weights)
            {
                for (float weight : weights)
                {
                    if (!finite_float(weight) || weight < 0.0f || weight > 1.0f)
                    {
                        return false;
                    }
                }
            }

            for (std::size_t vertex_index = 0u;
                vertex_index < vertex_count; ++vertex_index)
            {
                if (!finite_float(mesh.mass[vertex_index]) ||
                    mesh.mass[vertex_index] < 0.0f ||
                    !finite_float(mesh.inverse_mass[vertex_index]) ||
                    mesh.inverse_mass[vertex_index] < 0.0f ||
                    mesh.vertex_tetrahedron_offsets[vertex_index] >
                        mesh.vertex_tetrahedron_offsets[vertex_index + 1u] ||
                    mesh.vertex_tetrahedron_offsets[vertex_index + 1u] >
                        mesh.vertex_tetrahedron_ids.size())
                {
                    return false;
                }
            }

            std::vector<std::vector<uint32_t>> expected_vertex_tetrahedra(
                vertex_count);
            for (uint32_t tetrahedron_index = 0u;
                tetrahedron_index < static_cast<uint32_t>(tetrahedron_count);
                ++tetrahedron_index)
            {
                const physics::tetrahedron& tetrahedron =
                    mesh.tetrahedra[tetrahedron_index];
                if (!finite_float(tetrahedron.rest_volume) ||
                    tetrahedron.rest_volume <= 0.0f ||
                    !finite_float(tetrahedron.quality) ||
                    !finite_float(tetrahedron.minimum_dihedral_degrees))
                {
                    return false;
                }

                for (float value : tetrahedron.rest_inverse)
                {
                    if (!finite_float(value))
                    {
                        return false;
                    }
                }

                for (const asset::vec3& gradient :
                    tetrahedron.rest_shape_gradients)
                {
                    if (!finite_vec3(gradient))
                    {
                        return false;
                    }
                }

                std::array<uint32_t, 4> sorted_vertices =
                    tetrahedron.vertices;
                std::sort(sorted_vertices.begin(), sorted_vertices.end());
                if (std::adjacent_find(
                        sorted_vertices.begin(), sorted_vertices.end()) !=
                    sorted_vertices.end())
                {
                    return false;
                }

                for (uint32_t local_vertex = 0u;
                    local_vertex < 4u; ++local_vertex)
                {
                    const uint32_t vertex_index =
                        tetrahedron.vertices[local_vertex];
                    if (vertex_index >= vertex_count)
                    {
                        return false;
                    }
                    expected_vertex_tetrahedra[vertex_index].push_back(
                        tetrahedron_index);
                }
            }

            std::size_t expected_offset = 0u;
            for (std::size_t vertex_index = 0u;
                vertex_index < vertex_count; ++vertex_index)
            {
                const std::vector<uint32_t>& expected =
                    expected_vertex_tetrahedra[vertex_index];
                const uint32_t begin =
                    mesh.vertex_tetrahedron_offsets[vertex_index];
                const uint32_t end =
                    mesh.vertex_tetrahedron_offsets[vertex_index + 1u];

                if (begin != expected_offset ||
                    static_cast<std::size_t>(end - begin) != expected.size())
                {
                    return false;
                }

                for (std::size_t incidence = 0u;
                    incidence < expected.size(); ++incidence)
                {
                    if (mesh.vertex_tetrahedron_ids[
                            static_cast<std::size_t>(begin) + incidence] !=
                        expected[incidence])
                    {
                        return false;
                    }
                }
                expected_offset = end;
            }

            return expected_offset == mesh.vertex_tetrahedron_ids.size();
        }

        uint32_t count_pending_ticks(double accumulator, double fixed_delta_time)
        {
            if (!finite_double(accumulator) ||
                !finite_double(fixed_delta_time) ||
                fixed_delta_time <= 0.0 ||
                accumulator + time_epsilon < fixed_delta_time)
            {
                return 0u;
            }
            const double count = std::floor(
                (accumulator + time_epsilon) / fixed_delta_time);
            if (!std::isfinite(count) ||
                count >= static_cast<double>(std::numeric_limits<uint32_t>::max()))
            {
                return std::numeric_limits<uint32_t>::max();
            }
            return static_cast<uint32_t>(count);
        }
    }

    bool status::success() const
    {
        return code == status_code::ok;
    }

    struct physics_runtime::implementation
    {
        struct render_candidate_mesh_slot
        {
            uint32_t committed_generation = 0u;
            uint64_t committed_tick_sequence = 0u;
            Microsoft::WRL::ComPtr<ID3D11Buffer> position_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> position_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> position_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> normal_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> normal_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> normal_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> tangent_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> tangent_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> tangent_uav;
        };

        struct render_candidate_slot_state
        {
            uint32_t generation = 0u;
            bool reserved = false;
            bool committed = false;
            bool retired = false;
            uint64_t committed_tick_sequence = 0u;
        };

        struct soft_body_gpu_mesh
        {
            uint32_t mesh_index = asset::invalid_index;
            uint32_t skin_node_index = asset::invalid_index;
            uint32_t skin_index = asset::invalid_index;
            uint32_t skin_joint_count = 0u;
            uint32_t vertex_count = 0u;
            uint32_t tetrahedron_count = 0u;
            Microsoft::WRL::ComPtr<ID3D11Buffer> skin_weights_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> skin_weights_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> animation_tracking_weights_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> animation_tracking_weights_srv;

            // Immutable Mesh-local source positions; FEM State Banks remain World-space.
            Microsoft::WRL::ComPtr<ID3D11Buffer> rest_positions_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> rest_positions_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> animation_target_positions_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> animation_target_positions_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> animation_target_positions_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> animation_target_velocities_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> animation_target_velocities_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> animation_target_velocities_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> animation_target_status_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> animation_target_status_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> animation_target_status_uav;

            std::array<Microsoft::WRL::ComPtr<ID3D11Buffer>, 2> position_banks;
            std::array<Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>, 2>
                position_bank_srvs;
            std::array<Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>, 2>
                position_bank_uavs;
            std::array<Microsoft::WRL::ComPtr<ID3D11Buffer>, 2> velocity_banks;
            std::array<Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>, 2>
                velocity_bank_srvs;
            std::array<Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>, 2>
                velocity_bank_uavs;

            Microsoft::WRL::ComPtr<ID3D11Buffer> position_scratch;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> position_scratch_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> position_scratch_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> velocity_scratch;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> velocity_scratch_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> velocity_scratch_uav;

            Microsoft::WRL::ComPtr<ID3D11Buffer> tetrahedron_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> tetrahedron_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> tetrahedron_output_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> tetrahedron_output_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> tetrahedron_output_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> vertex_tetrahedron_offsets_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> vertex_tetrahedron_offsets_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> vertex_tetrahedron_ids_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> vertex_tetrahedron_ids_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> inverse_mass_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> inverse_mass_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> vertex_damping_rate_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> vertex_damping_rate_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> vertex_force_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> vertex_force_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> vertex_force_uav;

            Microsoft::WRL::ComPtr<ID3D11Buffer> rest_lumped_mass_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> rest_lumped_mass_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> distribution_weight_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> distribution_weight_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> distribution_weight_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> distribution_partial_sum_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> distribution_partial_sum_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> distribution_partial_sum_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> distribution_sum_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> distribution_sum_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> distribution_sum_uav;
            std::array<Microsoft::WRL::ComPtr<ID3D11Buffer>, 2> force_accumulator_buffers;
            std::array<Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>, 2>
                force_accumulator_srvs;
            std::array<Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>, 2>
                force_accumulator_uavs;
            std::array<Microsoft::WRL::ComPtr<ID3D11Buffer>, 2> impulse_accumulator_buffers;
            std::array<Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>, 2>
                impulse_accumulator_srvs;
            std::array<Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>, 2>
                impulse_accumulator_uavs;
            Microsoft::WRL::ComPtr<ID3D11Buffer> distribution_reduce_parameters;

            Microsoft::WRL::ComPtr<ID3D11Buffer> fem_parameters;
            Microsoft::WRL::ComPtr<ID3D11Buffer> gather_parameters;
            Microsoft::WRL::ComPtr<ID3D11Buffer> integration_parameters;

            uint32_t render_vertex_count = 0u;
            uint32_t render_index_count = 0u;
            Microsoft::WRL::ComPtr<ID3D11Buffer> render_binding_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> render_binding_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> render_vertex_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> render_vertex_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> render_index_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> render_index_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> render_vertex_triangle_offsets_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> render_vertex_triangle_offsets_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> render_vertex_triangle_ids_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> render_vertex_triangle_ids_srv;
            Microsoft::WRL::ComPtr<ID3D11Buffer> render_parameters;
            std::vector<render_candidate_mesh_slot> render_candidate_slots;

            double rest_density = 0.0;
            std::vector<float> initial_inverse_masses;
            std::vector<gpu_float4> initial_positions;
            std::vector<gpu_float4> initial_velocities;
        };

        struct model_slot
        {
            uint32_t generation = 1u;
            bool occupied = false;
            bool retired = false;
            asset::model source_model;
            std::vector<model_mesh_physics_asset> mesh_physics_assets;
            std::vector<soft_body_gpu_mesh> gpu_soft_body_meshes;
            transform root_transform;
            std::vector<transform_sample> root_samples;
            bool has_animation_playback = false;
            animation_playback_desc animation_playback;
            bool has_animation_blend = false;
            animation_blend_desc animation_blend;
            double animation_world_time_origin = 0.0;
            std::vector<mesh_physics_parameters> mesh_parameters;
            std::vector<bool> mesh_parameters_valid;
        };

        struct collider_slot
        {
            uint32_t generation = 1u;
            bool occupied = false;
            bool retired = false;
            bool kinematic = false;
            collider_desc description;
            std::vector<transform_sample> pose_samples;
        };

        struct physics_mesh_input_snapshot
        {
            uint32_t mesh_index = asset::invalid_index;
            std::vector<physics::physics_vertex_attribute_weights> vertex_attribute_weights;
        };

        struct model_input_snapshot
        {
            uint32_t model_index = asset::invalid_index;
            uint32_t generation = 0u;
            transform root_transform;
            std::vector<transform_sample> root_samples;
            bool has_animation_playback = false;
            animation_playback_desc animation_playback;
            bool has_animation_blend = false;
            animation_blend_desc animation_blend;
            double animation_world_time_origin = 0.0;
            std::vector<mesh_physics_parameters> mesh_parameters;
            std::vector<bool> mesh_parameters_valid;
            std::vector<physics_mesh_input_snapshot> physics_meshes;
        };

        struct collider_input_snapshot
        {
            uint32_t collider_index = asset::invalid_index;
            uint32_t generation = 0u;
            bool kinematic = false;
            collider_desc description;
            std::vector<transform_sample> pose_samples;
        };

        struct tick_input_snapshot
        {
            uint64_t version = 0u;
            world_desc description;
            std::vector<force_desc> forces;
            std::vector<impulse_desc> impulses;
            std::vector<model_input_snapshot> models;
            std::vector<collider_input_snapshot> colliders;
        };

        struct integration_segment
        {
            double start_time = 0.0;
            double delta_time = 0.0;
            std::vector<uint32_t> active_force_indices;
            std::vector<uint32_t> impulse_indices;
        };

        struct model_animation_tick_schedule
        {
            uint32_t model_index = asset::invalid_index;
            uint32_t model_generation = 0u;
            bool has_blend = false;
            std::vector<physics::scheduled_animation_clip_span> playback_spans;
            std::vector<physics::animation_blend_clip_span> blend_spans;
        };

        struct tick_reservation
        {
            uint64_t sequence = 0u;
            double start_time = 0.0;
            double delta_time = 0.0;
            std::vector<integration_segment> integration_segments;
            std::vector<model_animation_tick_schedule> animation_schedules;
            std::shared_ptr<const tick_input_snapshot> input_snapshot;
            bool from_single_step = false;
            bool blocked = false;
            bool rejected = false;
            bool predecessor_blocked = false;
            uint64_t budget_revision = 0u;
            uint32_t planned_substeps = 0u;
            uint32_t planned_integration_segments = 0u;
        };

        struct force_distribution_global_scratch
        {
            uint32_t partial_sum_capacity = 0u;
            Microsoft::WRL::ComPtr<ID3D11Buffer> partial_sum_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> partial_sum_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> partial_sum_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> normalized_sum_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> normalized_sum_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> normalized_sum_uav;
        };

        struct soft_body_mesh_work_state
        {
            uint32_t model_index = asset::invalid_index;
            model_slot* model = nullptr;
            soft_body_gpu_mesh* gpu_mesh = nullptr;
            float damping_rate = 0.0f;
            bool source_is_scratch = false;
            uint32_t source_bank = 0u;
            ID3D11ShaderResourceView* source_position_srv = nullptr;
            ID3D11ShaderResourceView* source_velocity_srv = nullptr;
            ID3D11UnorderedAccessView* destination_position_uav = nullptr;
            ID3D11UnorderedAccessView* destination_velocity_uav = nullptr;
            bool destination_is_scratch = false;
            ID3D11ShaderResourceView* integrated_force_srv = nullptr;
            ID3D11ShaderResourceView* integrated_impulse_srv = nullptr;
            ID3D11ShaderResourceView* animation_target_status_srv = nullptr;
            ID3D11UnorderedAccessView* animation_cursor_status_uav = nullptr;
            float tracking_stiffness = 0.0f;
            float tracking_damping = 0.0f;
            float tracking_attribute_multiplier = 1.0f;
            bool has_animation_target = false;
            uint32_t force_event_count = 0u;
            uint32_t impulse_event_count = 0u;
        };

        struct animation_packet_work_state
        {
            uint32_t model_index = asset::invalid_index;
            uint32_t model_generation = 0u;
            physics::animation_trajectory_packet packet;
            physics::animation_trajectory_gpu_resources resources;
        };

        struct in_flight_tick
        {
            in_flight_tick() = default;
            in_flight_tick(const in_flight_tick&) = delete;
            in_flight_tick& operator=(const in_flight_tick&) = delete;
            in_flight_tick(in_flight_tick&&) noexcept = default;
            in_flight_tick& operator=(in_flight_tick&&) noexcept = default;

            tick_reservation reservation;
            uint64_t dispatch_id = 0u;
            uint64_t reservation_token = 0u;
            uint64_t input_snapshot_version = 0u;
            std::vector<Microsoft::WRL::ComPtr<ID3D11Buffer>>
                segment_parameter_buffers;
            std::vector<Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>>
                segment_shader_resource_views;
            std::vector<Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>>
                segment_unordered_access_views;
            std::vector<animation_packet_work_state>
                animation_packet_resources;
            force_distribution_global_scratch force_distribution_scratch;
            uint64_t expected_predecessor_sequence = 0u;
            uint64_t expected_source_generation = 1u;
            uint32_t expected_source_state_set = 0u;
            uint32_t expected_destination_state_set = 1u;
            uint32_t render_candidate_slot_id = asset::invalid_index;
            uint32_t render_candidate_slot_generation = 0u;
            Microsoft::WRL::ComPtr<ID3D11Buffer> plan_buffer;
            Microsoft::WRL::ComPtr<ID3D11Buffer> status_buffer;
        };

        struct world_slot
        {
            world_slot() = default;
            world_slot(const world_slot&) = delete;
            world_slot& operator=(const world_slot&) = delete;
            world_slot(world_slot&&) noexcept = default;
            world_slot& operator=(world_slot&&) noexcept = default;

            uint32_t generation = 1u;
            bool occupied = false;
            bool retired = false;
            world_desc description;
            world_execution_budget execution_budget;
            uint64_t execution_budget_revision = 1u;
            uint64_t next_input_snapshot_version = 1u;
            bool cpu_preflight_block_status_observed = false;
            bool gpu_dynamic_toi_block_status_observed = false;
            playback_state playback = playback_state::stopped;
            double accumulator_seconds = 0.0;
            double requested_simulation_time = 0.0;
            double cancelled_requested_time = 0.0;
            double reserved_tick_time = 0.0;
            double gpu_committed_simulation_time = 0.0;
            double cpu_observed_completed_time = 0.0;
            uint64_t submitted_sequence = 0u;
            uint64_t gpu_committed_sequence = 0u;
            uint64_t cpu_observed_sequence = 0u;
            uint64_t next_tick_sequence = 1u;
            uint32_t pending_single_step_requests = 0u;
            std::deque<tick_reservation> reservations;
            std::deque<in_flight_tick> in_flight_ticks;
            uint32_t in_flight_tick_count = 0u;
            uint64_t gpu_state_generation = 1u;
            uint32_t active_state_set = 0u;
            uint64_t scheduled_predecessor_sequence = 0u;
            uint64_t scheduled_source_generation = 1u;
            uint32_t scheduled_source_state_set = 0u;
            Microsoft::WRL::ComPtr<ID3D11Buffer> commit_control_buffer;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> commit_control_uav;
            bool render_lease_active = false;
            uint64_t render_lease_generation = 0u;
            uint32_t leased_render_slot_id = asset::invalid_index;
            uint32_t leased_render_slot_generation = 0u;
            std::vector<render_candidate_slot_state> render_candidate_slots;
            uint32_t latest_render_slot_id = asset::invalid_index;
            uint32_t latest_render_slot_generation = 0u;
            tick_state state = tick_state::idle;
            status_code last_tick_status = status_code::ok;
            work_budget_block_origin budget_block_origin =
                work_budget_block_origin::none;
            work_budget_resolution budget_resolution =
                work_budget_resolution::none;
            uint8_t work_budget_block_flags = 0u;
            uint32_t planned_substeps = 0u;
            uint32_t planned_integration_segments = 0u;
            uint32_t executed_integration_segments = 0u;
            uint32_t minimum_required_integration_segments = 0u;
            std::string diagnostic;
            std::vector<model_slot> models;
            std::vector<collider_slot> colliders;
            std::vector<force_desc> forces;
            std::vector<impulse_desc> impulses;
            std::unordered_set<uint64_t> event_ids;
            std::unordered_set<uint64_t> observed_tick_sequences;
            std::unordered_map<uint64_t, double> observed_tick_deltas;
        };

        runtime_desc desc;
        Microsoft::WRL::ComPtr<ID3D11Device> device;
        Microsoft::WRL::ComPtr<ID3D11DeviceContext> immediate_context;
        std::unique_ptr<simulation::d3d11_status_ring> status_ring;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_begin_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_validate_empty_world_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_validate_soft_body_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_commit_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_animation_skinning_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_channel_evaluate_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_blend_local_transform_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_node_hierarchy_evaluate_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_skin_matrix_evaluate_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_span_cursor_advance_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_fem_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_gather_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_weight_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_reduce_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_finalize_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_apply_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_integrate_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_copy_state_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_render_feedback_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_render_validate_shader;
        std::vector<world_slot> worlds;
        std::thread::id submission_thread;
        uint64_t device_generation = 1u;
        uint64_t next_dispatch_id = 1u;
        bool initialized = false;
        bool accepting_submissions = true;

        bool on_submission_thread() const
        {
            return submission_thread == std::this_thread::get_id();
        }

        status thread_status() const
        {
            if (!on_submission_thread())
            {
                return make_status(
                    status_code::submission_thread_violation,
                    "Runtime API must be called from the registered Immediate Context submission thread.");
            }
            return make_status(status_code::ok, {});
        }

        world_slot* find_world(world_handle handle)
        {
            if (handle.index >= worlds.size())
            {
                return nullptr;
            }
            world_slot& world = worlds[handle.index];
            if (!world.occupied || world.generation != handle.generation)
            {
                return nullptr;
            }
            return &world;
        }

        const world_slot* find_world(world_handle handle) const
        {
            if (handle.index >= worlds.size())
            {
                return nullptr;
            }
            const world_slot& world = worlds[handle.index];
            if (!world.occupied || world.generation != handle.generation)
            {
                return nullptr;
            }
            return &world;
        }

        model_slot* find_model(world_slot& world, model_handle handle)
        {
            if (handle.index >= world.models.size())
            {
                return nullptr;
            }
            model_slot& model = world.models[handle.index];
            if (!model.occupied || model.generation != handle.generation)
            {
                return nullptr;
            }
            return &model;
        }

        collider_slot* find_collider(world_slot& world, collider_handle handle)
        {
            if (handle.index >= world.colliders.size())
            {
                return nullptr;
            }
            collider_slot& collider = world.colliders[handle.index];
            if (!collider.occupied || collider.generation != handle.generation)
            {
                return nullptr;
            }
            return &collider;
        }

        const model_input_snapshot* find_model_input_snapshot(
            const tick_input_snapshot& snapshot,
            uint32_t model_index,
            uint32_t generation) const
        {
            const auto iterator = std::find_if(
                snapshot.models.begin(),
                snapshot.models.end(),
                [model_index, generation](const model_input_snapshot& value)
                {
                    return value.model_index == model_index &&
                        value.generation == generation;
                });
            return iterator == snapshot.models.end() ? nullptr : &(*iterator);
        }

        const physics_mesh_input_snapshot* find_physics_mesh_input_snapshot(
            const model_input_snapshot& model_snapshot,
            uint32_t mesh_index) const
        {
            const auto iterator = std::find_if(
                model_snapshot.physics_meshes.begin(),
                model_snapshot.physics_meshes.end(),
                [mesh_index](const physics_mesh_input_snapshot& value)
                {
                    return value.mesh_index == mesh_index;
                });
            return iterator == model_snapshot.physics_meshes.end()
                ? nullptr
                : &(*iterator);
        }

        bool build_tick_input_snapshot(
            const world_slot& world,
            uint64_t version,
            std::shared_ptr<const tick_input_snapshot>& out_snapshot,
            std::string& diagnostic) const
        {
            out_snapshot.reset();
            diagnostic.clear();
            if (version == 0u ||
                world.models.size() >= static_cast<std::size_t>(asset::invalid_index) ||
                world.colliders.size() >= static_cast<std::size_t>(asset::invalid_index))
            {
                diagnostic =
                    "Tick Input Snapshot Version or Handle index range is invalid.";
                return false;
            }

            try
            {
                std::shared_ptr<tick_input_snapshot> snapshot =
                    std::make_shared<tick_input_snapshot>();
                snapshot->version = version;
                snapshot->description = world.description;
                snapshot->forces = world.forces;
                snapshot->impulses = world.impulses;
                snapshot->models.reserve(world.models.size());
                snapshot->colliders.reserve(world.colliders.size());

                for (uint32_t model_index = 0u;
                    model_index < world.models.size(); ++model_index)
                {
                    const model_slot& model = world.models[model_index];
                    if (!model.occupied)
                    {
                        continue;
                    }

                    model_input_snapshot model_snapshot;
                    model_snapshot.model_index = model_index;
                    model_snapshot.generation = model.generation;
                    model_snapshot.root_transform = model.root_transform;
                    model_snapshot.root_samples = model.root_samples;
                    model_snapshot.has_animation_playback =
                        model.has_animation_playback;
                    model_snapshot.animation_playback = model.animation_playback;
                    model_snapshot.has_animation_blend = model.has_animation_blend;
                    model_snapshot.animation_blend = model.animation_blend;
                    model_snapshot.animation_world_time_origin =
                        model.animation_world_time_origin;
                    model_snapshot.mesh_parameters = model.mesh_parameters;
                    model_snapshot.mesh_parameters_valid =
                        model.mesh_parameters_valid;
                    model_snapshot.physics_meshes.reserve(
                        model.mesh_physics_assets.size());

                    for (const model_mesh_physics_asset& mesh_asset :
                        model.mesh_physics_assets)
                    {
                        physics_mesh_input_snapshot mesh_snapshot;
                        mesh_snapshot.mesh_index = mesh_asset.mesh_index;
                        mesh_snapshot.vertex_attribute_weights =
                            mesh_asset.physics_mesh.vertex_attribute_weights;
                        model_snapshot.physics_meshes.push_back(
                            std::move(mesh_snapshot));
                    }

                    snapshot->models.push_back(std::move(model_snapshot));
                }

                for (uint32_t collider_index = 0u;
                    collider_index < world.colliders.size(); ++collider_index)
                {
                    const collider_slot& collider = world.colliders[collider_index];
                    if (!collider.occupied)
                    {
                        continue;
                    }

                    collider_input_snapshot collider_snapshot;
                    collider_snapshot.collider_index = collider_index;
                    collider_snapshot.generation = collider.generation;
                    collider_snapshot.kinematic = collider.kinematic;
                    collider_snapshot.description = collider.description;
                    collider_snapshot.pose_samples = collider.pose_samples;
                    snapshot->colliders.push_back(std::move(collider_snapshot));
                }

                out_snapshot = std::move(snapshot);
                return true;
            }
            catch (...)
            {
                diagnostic =
                    "Unable to allocate or copy the immutable Tick Input Snapshot.";
                out_snapshot.reset();
                return false;
            }
        }

        bool has_world_submission(const world_slot& world) const
        {
            return world.in_flight_tick_count != 0u ||
                world.reserved_tick_time > time_epsilon;
        }

        status validate_input_mutation(const world_slot& world) const
        {
            if (has_world_submission(world) || world.render_lease_active)
            {
                return make_status(status_code::busy,
                    "World input cannot change after a Tick Reservation or Render Lease has fixed the current input state.");
            }
            return make_status(status_code::ok, {});
        }

        uint64_t world_identity(uint32_t index, uint32_t generation) const
        {
            return (static_cast<uint64_t>(generation) << 32u) | index;
        }

        void restore_reservation_in_sequence_order(
            world_slot& world,
            tick_reservation reservation)
        {
            const auto insertion_point = std::find_if(
                world.reservations.begin(),
                world.reservations.end(),
                [&](const tick_reservation& current)
                {
                    return current.sequence > reservation.sequence;
                });
            world.reservations.insert(
                insertion_point, std::move(reservation));
        }

        void synchronize_scheduled_state_with_committed(world_slot& world)
        {
            if (world.in_flight_tick_count != 0u)
            {
                return;
            }
            world.scheduled_predecessor_sequence = world.gpu_committed_sequence;
            world.scheduled_source_generation = world.gpu_state_generation;
            world.scheduled_source_state_set = world.active_state_set;
        }

        bool build_soft_body_tet_inputs(
            const physics::physics_mesh& mesh,
            const std::vector<physics::physics_vertex_attribute_weights>&
                vertex_attribute_weights,
            const mesh_physics_parameters& parameters,
            std::vector<gpu_soft_body_tet_input>& out_inputs,
            float& out_damping_rate,
            std::vector<float>& out_vertex_damping_rates,
            std::string& diagnostic) const
        {
            diagnostic.clear();
            out_inputs.clear();
            out_vertex_damping_rates.clear();
            if (mesh.tetrahedra.empty() ||
                mesh.tetrahedra.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) ||
                vertex_attribute_weights.size() != mesh.rest_positions.size())
            {
                diagnostic =
                    "Soft Body Tet count or transferred per-vertex Physics Attribute Weight layout is invalid.";
                return false;
            }

            const float poisson_denominator = 2.0f * (1.0f + parameters.poisson_ratio);
            const float bulk_denominator = 3.0f * (1.0f - 2.0f * parameters.poisson_ratio);
            if (!finite_float(poisson_denominator) ||
                !finite_float(bulk_denominator) ||
                poisson_denominator <= 0.0f ||
                bulk_denominator <= 0.0f)
            {
                diagnostic = "Soft Body Lamé parameter denominator is invalid.";
                return false;
            }

            const float shear_modulus = parameters.young_modulus / poisson_denominator;
            const float bulk_modulus = parameters.young_modulus / bulk_denominator;
            const std::size_t shape_channel = static_cast<std::size_t>(
                asset::physics_attribute_channel::shape_recovery);
            const std::size_t volume_channel = static_cast<std::size_t>(
                asset::physics_attribute_channel::volume);
            const std::size_t damping_channel = static_cast<std::size_t>(
                asset::physics_attribute_channel::damping);
            const float shape_multiplier = parameters.attribute_multipliers[shape_channel];
            const float volume_multiplier = parameters.attribute_multipliers[volume_channel];
            out_damping_rate =
                parameters.damping * parameters.attribute_multipliers[damping_channel];

            if (!finite_float(shear_modulus) ||
                !finite_float(bulk_modulus) ||
                !finite_float(out_damping_rate) ||
                shear_modulus <= 0.0f ||
                bulk_modulus <= 0.0f ||
                out_damping_rate < 0.0f)
            {
                diagnostic = "Soft Body material conversion produced a non-finite or invalid coefficient.";
                return false;
            }

            out_vertex_damping_rates.resize(mesh.rest_positions.size());
            for (std::size_t vertex_index = 0u;
                vertex_index < vertex_attribute_weights.size();
                ++vertex_index)
            {
                const float weight =
                    vertex_attribute_weights[vertex_index][damping_channel];
                const float damping_rate = out_damping_rate * weight;
                if (!finite_float(weight) || weight < 0.0f || weight > 1.0f ||
                    !finite_float(damping_rate) || damping_rate < 0.0f)
                {
                    diagnostic =
                        "Soft Body per-vertex Damping Paint produced an invalid damping rate.";
                    return false;
                }
                out_vertex_damping_rates[vertex_index] = damping_rate;
            }

            out_inputs.resize(mesh.tetrahedra.size());
            for (std::size_t tet_index = 0;
                tet_index < mesh.tetrahedra.size(); ++tet_index)
            {
                const physics::tetrahedron& source = mesh.tetrahedra[tet_index];
                gpu_soft_body_tet_input& destination = out_inputs[tet_index];
                float average_shape_recovery = 0.0f;
                float average_volume_retention = 0.0f;
                for (uint32_t local_vertex = 0u; local_vertex < 4u; ++local_vertex)
                {
                    const uint32_t vertex_index = source.vertices[local_vertex];
                    if (vertex_index >= vertex_attribute_weights.size())
                    {
                        diagnostic = "Soft Body Tet references an invalid Physics Attribute Weight vertex.";
                        return false;
                    }

                    destination.vertex_ids[local_vertex] = vertex_index;
                    const asset::vec3& gradient =
                        source.rest_shape_gradients[local_vertex];
                    destination.rest_gradients[local_vertex][0] = gradient.x;
                    destination.rest_gradients[local_vertex][1] = gradient.y;
                    destination.rest_gradients[local_vertex][2] = gradient.z;
                    destination.rest_gradients[local_vertex][3] = 0.0f;
                    average_shape_recovery +=
                        vertex_attribute_weights[vertex_index][shape_channel] * 0.25f;
                    average_volume_retention +=
                        vertex_attribute_weights[vertex_index][volume_channel] * 0.25f;
                }

                for (uint32_t row = 0u; row < 3u; ++row)
                {
                    for (uint32_t column = 0u; column < 3u; ++column)
                    {
                        destination.rest_inverse[row][column] =
                            source.rest_inverse[row * 3u + column];
                    }
                    destination.rest_inverse[row][3] = 0.0f;
                }

                destination.material0[0] = source.rest_volume;
                destination.material0[1] = shear_modulus;
                destination.material0[2] = bulk_modulus;
                destination.material0[3] =
                    shape_multiplier * average_shape_recovery;
                destination.material1[0] =
                    volume_multiplier * average_volume_retention;
                destination.material1[1] = parameters.young_modulus;
                destination.material1[2] = 0.1f;
                destination.material1[3] = 0.0f;
            }
            return true;
        }

        bool initialize_soft_body_gpu_mesh(
            uint32_t mesh_index,
            const model_mesh_physics_asset& mesh_asset,
            const asset::skin* source_skin,
            const asset::mesh& render_mesh,
            const transform& root_transform,
            uint32_t render_candidate_slot_count,
            soft_body_gpu_mesh& out_mesh,
            std::string& diagnostic)
        {
            diagnostic.clear();
            const physics::physics_mesh& mesh = mesh_asset.physics_mesh;
            if (mesh.rest_positions.empty() ||
                mesh.rest_positions.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) ||
                mesh.tetrahedra.empty() ||
                mesh.tetrahedra.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))
            {
                diagnostic = "Soft Body GPU Mesh has an empty or unrepresentable vertex / Tet count.";
                return false;
            }

            soft_body_gpu_mesh candidate;
            candidate.mesh_index = mesh_index;
            candidate.skin_node_index = mesh_asset.skin_node_index;
            candidate.skin_index = mesh_asset.skin_index;
            candidate.vertex_count = static_cast<uint32_t>(mesh.rest_positions.size());
            candidate.tetrahedron_count = static_cast<uint32_t>(mesh.tetrahedra.size());
            std::vector<gpu_soft_body_skin_weight_record> skin_weight_records;
            if (mesh_asset.skin_index == asset::invalid_index)
            {
                if (source_skin != nullptr ||
                    mesh_asset.skin_node_index != asset::invalid_index ||
                    mesh.has_skin_weights || !mesh.skin_weights.empty())
                {
                    diagnostic =
                        "Unskinned Soft Body Mesh contains unexpected Skin metadata or Physics Vertex influences.";
                    return false;
                }
            }
            else
            {
                if (source_skin == nullptr ||
                    mesh_asset.skin_node_index == asset::invalid_index ||
                    source_skin->joints.empty() ||
                    !mesh.has_skin_weights ||
                    mesh.skin_weights.size() != mesh.rest_positions.size())
                {
                    diagnostic =
                        "Skinned Soft Body GPU Mesh requires a valid Mesh Node / Skin and one transferred Physics Vertex influence record per Vertex.";
                    return false;
                }

                candidate.skin_joint_count =
                    static_cast<uint32_t>(source_skin->joints.size());
                skin_weight_records.resize(mesh.skin_weights.size());
                for (std::size_t vertex_index = 0u;
                    vertex_index < mesh.skin_weights.size(); ++vertex_index)
                {
                    const physics::physics_skin_weight& source_weights =
                        mesh.skin_weights[vertex_index];
                    gpu_soft_body_skin_weight_record& destination =
                        skin_weight_records[vertex_index];
                    double weight_sum = 0.0;
                    for (std::size_t influence_index = 0u;
                        influence_index < asset::max_skin_influences;
                        ++influence_index)
                    {
                        const float weight = source_weights.weights[influence_index];
                        const uint16_t joint = source_weights.joints[influence_index];
                        if (!finite_float(weight) || weight < 0.0f ||
                            (weight > 0.0f && joint >= candidate.skin_joint_count))
                        {
                            diagnostic =
                                "GPU Skin Weight upload contains an invalid influence index or non-finite weight.";
                            return false;
                        }
                        destination.joints[influence_index] =
                            static_cast<uint32_t>(joint);
                        destination.weights[influence_index] = weight;
                        weight_sum += weight;
                    }
                    if (!finite_double(weight_sum) ||
                        std::abs(weight_sum - 1.0) > 1.0e-3)
                    {
                        diagnostic =
                            "GPU Skin Weight upload requires normalized Physics Vertex influences.";
                        return false;
                    }
                }
            }

            candidate.initial_positions.resize(mesh.rest_positions.size());
            candidate.initial_velocities.resize(mesh.rest_positions.size());
            candidate.initial_inverse_masses = mesh.inverse_mass;
            if (render_mesh.vertices.empty() ||
                render_mesh.indices.empty() ||
                (render_mesh.indices.size() % 3u) != 0u ||
                render_mesh.vertices.size() >=
                    static_cast<std::size_t>(asset::invalid_index) ||
                render_mesh.indices.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) ||
                mesh_asset.render_bindings.size() != render_mesh.vertices.size() ||
                render_candidate_slot_count < 2u)
            {
                diagnostic =
                    "Soft Body Render Candidate requires valid triangle topology, one Binding per Render Vertex, and at least two Ring Slots.";
                return false;
            }
            candidate.render_vertex_count =
                static_cast<uint32_t>(render_mesh.vertices.size());
            candidate.render_index_count =
                static_cast<uint32_t>(render_mesh.indices.size());

            double total_rest_mass = 0.0;
            double total_rest_volume = 0.0;
            for (float mass : mesh.mass)
            {
                total_rest_mass += static_cast<double>(mass);
            }
            for (const physics::tetrahedron& tetrahedron : mesh.tetrahedra)
            {
                total_rest_volume += static_cast<double>(tetrahedron.rest_volume);
            }
            if (!finite_double(total_rest_mass) ||
                !finite_double(total_rest_volume) ||
                total_rest_mass <= 0.0 ||
                total_rest_volume <= 0.0)
            {
                diagnostic = "Soft Body Physics Mesh has an invalid total rest mass or rest volume.";
                return false;
            }
            candidate.rest_density = total_rest_mass / total_rest_volume;
            if (!finite_double(candidate.rest_density) ||
                candidate.rest_density <= 0.0)
            {
                diagnostic = "Soft Body Physics Mesh rest density is not finite and positive.";
                return false;
            }

            for (std::size_t vertex_index = 0;
                vertex_index < mesh.rest_positions.size(); ++vertex_index)
            {
                const asset::vec3 position = transform_position(
                    root_transform, mesh.rest_positions[vertex_index]);
                if (!finite_vec3(position))
                {
                    diagnostic = "Soft Body initial world-space position is not finite.";
                    return false;
                }
                candidate.initial_positions[vertex_index] = {
                    position.x, position.y, position.z, 1.0f
                };
                candidate.initial_velocities[vertex_index] = {};
            }

            const mesh_physics_parameters default_parameters;
            std::vector<gpu_soft_body_tet_input> tet_inputs;
            float damping_rate = 0.0f;
            std::vector<float> initial_vertex_damping_rates;
            if (!build_soft_body_tet_inputs(
                    mesh, mesh.vertex_attribute_weights, default_parameters,
                    tet_inputs, damping_rate, initial_vertex_damping_rates,
                    diagnostic))
            {
                return false;
            }

            std::vector<gpu_soft_body_tet_output> tet_output(
                mesh.tetrahedra.size(), gpu_soft_body_tet_output{});
            std::vector<gpu_float4> vertex_force(
                mesh.rest_positions.size(), gpu_float4{});
            std::vector<gpu_float4> mesh_local_rest_positions;
            mesh_local_rest_positions.reserve(mesh.rest_positions.size());
            for (const asset::vec3& position : mesh.rest_positions)
            {
                mesh_local_rest_positions.push_back({
                    position.x, position.y, position.z, 1.0f
                });
            }
            const std::vector<gpu_float4> empty_animation_targets(
                mesh.rest_positions.size(), gpu_float4{});
            const std::vector<uint32_t> empty_animation_target_status(1u, 0u);
            std::vector<float> initial_animation_tracking_weights(
                mesh.rest_positions.size(), 1.0f);
            if (mesh.vertex_attribute_weights.size() ==
                mesh.rest_positions.size())
            {
                const std::size_t tracking_channel =
                    static_cast<std::size_t>(
                        asset::physics_attribute_channel::animation_tracking);
                for (std::size_t vertex_index = 0u;
                    vertex_index < mesh.rest_positions.size();
                    ++vertex_index)
                {
                    initial_animation_tracking_weights[vertex_index] =
                        mesh.vertex_attribute_weights[vertex_index][tracking_channel];
                }
            }

            const UINT state_bind_flags =
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
            for (uint32_t bank_index = 0u; bank_index < 2u; ++bank_index)
            {
                if (!create_structured_buffer_from_vector(
                        device.Get(), candidate.initial_positions,
                        state_bind_flags,
                        candidate.position_banks[bank_index].GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(), candidate.position_banks[bank_index].Get(),
                        candidate.vertex_count,
                        candidate.position_bank_srvs[bank_index].GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(), candidate.position_banks[bank_index].Get(),
                        candidate.vertex_count,
                        candidate.position_bank_uavs[bank_index].GetAddressOf()) ||
                    !create_structured_buffer_from_vector(
                        device.Get(), candidate.initial_velocities,
                        state_bind_flags,
                        candidate.velocity_banks[bank_index].GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(), candidate.velocity_banks[bank_index].Get(),
                        candidate.vertex_count,
                        candidate.velocity_bank_srvs[bank_index].GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(), candidate.velocity_banks[bank_index].Get(),
                        candidate.vertex_count,
                        candidate.velocity_bank_uavs[bank_index].GetAddressOf()))
                {
                    diagnostic = "Unable to allocate both persistent Soft Body State Banks.";
                    return false;
                }
            }

            if (!create_structured_buffer_from_vector(
                    device.Get(), candidate.initial_positions,
                    state_bind_flags, candidate.position_scratch.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.position_scratch.Get(),
                    candidate.vertex_count,
                    candidate.position_scratch_srv.GetAddressOf()) ||
                !create_structured_uav(
                    device.Get(), candidate.position_scratch.Get(),
                    candidate.vertex_count,
                    candidate.position_scratch_uav.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), candidate.initial_velocities,
                    state_bind_flags, candidate.velocity_scratch.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.velocity_scratch.Get(),
                    candidate.vertex_count,
                    candidate.velocity_scratch_srv.GetAddressOf()) ||
                !create_structured_uav(
                    device.Get(), candidate.velocity_scratch.Get(),
                    candidate.vertex_count,
                    candidate.velocity_scratch_uav.GetAddressOf()))
            {
                diagnostic = "Unable to allocate Soft Body Tick Scratch State.";
                return false;
            }

            if (!create_structured_buffer_from_vector(
                    device.Get(), mesh_local_rest_positions,
                    D3D11_BIND_SHADER_RESOURCE,
                    candidate.rest_positions_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.rest_positions_buffer.Get(),
                    candidate.vertex_count,
                    candidate.rest_positions_srv.GetAddressOf()))
            {
                diagnostic =
                    "Unable to upload immutable Mesh-local Rest Position records for the Soft Body GPU Mesh.";
                return false;
            }

            if (!create_structured_buffer_from_vector(
                    device.Get(),
                    initial_animation_tracking_weights,
                    D3D11_BIND_SHADER_RESOURCE,
                    candidate.animation_tracking_weights_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(),
                    candidate.animation_tracking_weights_buffer.Get(),
                    candidate.vertex_count,
                    candidate.animation_tracking_weights_srv.GetAddressOf()))
            {
                diagnostic =
                    "Unable to upload per-vertex Animation Tracking Paint weights.";
                return false;
            }

            if (!skin_weight_records.empty() &&
                (!create_structured_buffer_from_vector(
                    device.Get(), skin_weight_records,
                    D3D11_BIND_SHADER_RESOURCE,
                    candidate.skin_weights_buffer.GetAddressOf()) ||
                 !create_structured_srv(
                    device.Get(), candidate.skin_weights_buffer.Get(),
                    candidate.vertex_count,
                    candidate.skin_weights_srv.GetAddressOf()) ||
                 !create_structured_buffer_from_vector(
                    device.Get(), empty_animation_targets,
                    state_bind_flags,
                    candidate.animation_target_positions_buffer.GetAddressOf()) ||
                 !create_structured_srv(
                    device.Get(), candidate.animation_target_positions_buffer.Get(),
                    candidate.vertex_count,
                    candidate.animation_target_positions_srv.GetAddressOf()) ||
                 !create_structured_uav(
                    device.Get(), candidate.animation_target_positions_buffer.Get(),
                    candidate.vertex_count,
                    candidate.animation_target_positions_uav.GetAddressOf()) ||
                 !create_structured_buffer_from_vector(
                    device.Get(), empty_animation_targets,
                    state_bind_flags,
                    candidate.animation_target_velocities_buffer.GetAddressOf()) ||
                 !create_structured_srv(
                    device.Get(), candidate.animation_target_velocities_buffer.Get(),
                    candidate.vertex_count,
                    candidate.animation_target_velocities_srv.GetAddressOf()) ||
                 !create_structured_uav(
                    device.Get(), candidate.animation_target_velocities_buffer.Get(),
                    candidate.vertex_count,
                    candidate.animation_target_velocities_uav.GetAddressOf()) ||
                 !create_structured_buffer_from_vector(
                    device.Get(), empty_animation_target_status,
                    state_bind_flags,
                    candidate.animation_target_status_buffer.GetAddressOf()) ||
                 !create_structured_srv(
                    device.Get(), candidate.animation_target_status_buffer.Get(),
                    1u,
                    candidate.animation_target_status_srv.GetAddressOf()) ||
                 !create_structured_uav(
                    device.Get(), candidate.animation_target_status_buffer.Get(),
                    1u,
                    candidate.animation_target_status_uav.GetAddressOf())))
            {
                diagnostic =
                    "Unable to upload Physics Vertex Skin Weights or allocate LBS Target Position / Velocity / Status buffers for the Soft Body GPU Mesh.";
                return false;
            }

            if (!create_structured_buffer_from_vector(
                    device.Get(), tet_inputs, D3D11_BIND_SHADER_RESOURCE,
                    candidate.tetrahedron_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.tetrahedron_buffer.Get(),
                    candidate.tetrahedron_count,
                    candidate.tetrahedron_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), tet_output, state_bind_flags,
                    candidate.tetrahedron_output_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.tetrahedron_output_buffer.Get(),
                    candidate.tetrahedron_count,
                    candidate.tetrahedron_output_srv.GetAddressOf()) ||
                !create_structured_uav(
                    device.Get(), candidate.tetrahedron_output_buffer.Get(),
                    candidate.tetrahedron_count,
                    candidate.tetrahedron_output_uav.GetAddressOf()))
            {
                diagnostic = "Unable to allocate Soft Body Tet Input / Output resources.";
                return false;
            }

            if (mesh.vertex_tetrahedron_offsets.size() !=
                    mesh.rest_positions.size() + 1u ||
                mesh.vertex_tetrahedron_ids.empty() ||
                mesh.inverse_mass.size() != mesh.rest_positions.size())
            {
                diagnostic = "Soft Body Vertex-to-Tet CSR or inverse-mass array is invalid.";
                return false;
            }

            if (!create_structured_buffer_from_vector(
                    device.Get(), mesh.vertex_tetrahedron_offsets,
                    D3D11_BIND_SHADER_RESOURCE,
                    candidate.vertex_tetrahedron_offsets_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.vertex_tetrahedron_offsets_buffer.Get(),
                    static_cast<uint32_t>(mesh.vertex_tetrahedron_offsets.size()),
                    candidate.vertex_tetrahedron_offsets_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), mesh.vertex_tetrahedron_ids,
                    D3D11_BIND_SHADER_RESOURCE,
                    candidate.vertex_tetrahedron_ids_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.vertex_tetrahedron_ids_buffer.Get(),
                    static_cast<uint32_t>(mesh.vertex_tetrahedron_ids.size()),
                    candidate.vertex_tetrahedron_ids_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), mesh.inverse_mass,
                    D3D11_BIND_SHADER_RESOURCE,
                    candidate.inverse_mass_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.inverse_mass_buffer.Get(),
                    candidate.vertex_count,
                    candidate.inverse_mass_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), initial_vertex_damping_rates,
                    D3D11_BIND_SHADER_RESOURCE,
                    candidate.vertex_damping_rate_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.vertex_damping_rate_buffer.Get(),
                    candidate.vertex_count,
                    candidate.vertex_damping_rate_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), vertex_force, state_bind_flags,
                    candidate.vertex_force_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.vertex_force_buffer.Get(),
                    candidate.vertex_count,
                    candidate.vertex_force_srv.GetAddressOf()) ||
                !create_structured_uav(
                    device.Get(), candidate.vertex_force_buffer.Get(),
                    candidate.vertex_count,
                    candidate.vertex_force_uav.GetAddressOf()))
            {
                diagnostic = "Unable to allocate Soft Body CSR / Mass / Force resources.";
                return false;
            }


            if (mesh.mass.size() != candidate.vertex_count ||
                candidate.vertex_count >
                    std::numeric_limits<uint32_t>::max() / sizeof(float))
            {
                diagnostic =
                    "Soft Body Force / Impulse distribution requires one Rest Lumped Mass per Physics Vertex.";
                return false;
            }

            const uint32_t distribution_group_count =
                (candidate.vertex_count - 1u) / 64u + 1u;
            if (distribution_group_count >
                D3D11_CS_DISPATCH_MAX_THREAD_GROUPS_PER_DIMENSION)
            {
                diagnostic =
                    "Soft Body Force / Impulse distribution exceeds the D3D11 one-dimensional Dispatch group limit.";
                return false;
            }
            const uint32_t distribution_weight_byte_width =
                candidate.vertex_count * static_cast<uint32_t>(sizeof(float));
            const UINT distribution_bind_flags =
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
            std::vector<float> zero_distribution_partials(
                distribution_group_count, 0.0f);
            std::vector<float> zero_distribution_sum(1u, 0.0f);
            std::vector<gpu_float4> zero_accumulator(
                candidate.vertex_count, gpu_float4{});

            if (!create_structured_buffer_from_vector(
                    device.Get(), mesh.mass, D3D11_BIND_SHADER_RESOURCE,
                    candidate.rest_lumped_mass_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.rest_lumped_mass_buffer.Get(),
                    candidate.vertex_count,
                    candidate.rest_lumped_mass_srv.GetAddressOf()) ||
                !create_structured_buffer(
                    device.Get(), distribution_weight_byte_width,
                    static_cast<uint32_t>(sizeof(float)),
                    distribution_bind_flags, nullptr,
                    candidate.distribution_weight_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.distribution_weight_buffer.Get(),
                    candidate.vertex_count,
                    candidate.distribution_weight_srv.GetAddressOf()) ||
                !create_structured_uav(
                    device.Get(), candidate.distribution_weight_buffer.Get(),
                    candidate.vertex_count,
                    candidate.distribution_weight_uav.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), zero_distribution_partials,
                    distribution_bind_flags,
                    candidate.distribution_partial_sum_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.distribution_partial_sum_buffer.Get(),
                    distribution_group_count,
                    candidate.distribution_partial_sum_srv.GetAddressOf()) ||
                !create_structured_uav(
                    device.Get(), candidate.distribution_partial_sum_buffer.Get(),
                    distribution_group_count,
                    candidate.distribution_partial_sum_uav.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), zero_distribution_sum,
                    distribution_bind_flags,
                    candidate.distribution_sum_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.distribution_sum_buffer.Get(),
                    1u, candidate.distribution_sum_srv.GetAddressOf()) ||
                !create_structured_uav(
                    device.Get(), candidate.distribution_sum_buffer.Get(),
                    1u, candidate.distribution_sum_uav.GetAddressOf()))
            {
                diagnostic =
                    "Unable to allocate Soft Body Force / Impulse distribution Weight and Reduction resources.";
                return false;
            }

            for (uint32_t accumulator_index = 0u;
                accumulator_index < 2u; ++accumulator_index)
            {
                if (!create_structured_buffer_from_vector(
                        device.Get(), zero_accumulator,
                        distribution_bind_flags,
                        candidate.force_accumulator_buffers[accumulator_index].GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(),
                        candidate.force_accumulator_buffers[accumulator_index].Get(),
                        candidate.vertex_count,
                        candidate.force_accumulator_srvs[accumulator_index].GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(),
                        candidate.force_accumulator_buffers[accumulator_index].Get(),
                        candidate.vertex_count,
                        candidate.force_accumulator_uavs[accumulator_index].GetAddressOf()) ||
                    !create_structured_buffer_from_vector(
                        device.Get(), zero_accumulator,
                        distribution_bind_flags,
                        candidate.impulse_accumulator_buffers[accumulator_index].GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(),
                        candidate.impulse_accumulator_buffers[accumulator_index].Get(),
                        candidate.vertex_count,
                        candidate.impulse_accumulator_srvs[accumulator_index].GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(),
                        candidate.impulse_accumulator_buffers[accumulator_index].Get(),
                        candidate.vertex_count,
                        candidate.impulse_accumulator_uavs[accumulator_index].GetAddressOf()))
                {
                    diagnostic =
                        "Unable to allocate Soft Body Force / Impulse accumulation buffers.";
                    return false;
                }
            }

            gpu_soft_body_distribution_reduce_parameters distribution_reduce_parameters;
            distribution_reduce_parameters.vertex_count = candidate.vertex_count;
            distribution_reduce_parameters.group_count = distribution_group_count;
            if (!create_constant_buffer(
                    device.Get(), sizeof(distribution_reduce_parameters),
                    &distribution_reduce_parameters,
                    candidate.distribution_reduce_parameters.GetAddressOf()))
            {
                diagnostic =
                    "Unable to allocate Soft Body Force / Impulse reduction parameters.";
                return false;
            }

            gpu_soft_body_fem_parameters fem_parameters;
            fem_parameters.tetrahedron_count = candidate.tetrahedron_count;
            gpu_soft_body_gather_parameters gather_parameters;
            gather_parameters.vertex_count = candidate.vertex_count;
            gpu_soft_body_integration_parameters integration_parameters;
            integration_parameters.vertex_count = candidate.vertex_count;

            if (!create_constant_buffer(
                    device.Get(), sizeof(fem_parameters), &fem_parameters,
                    candidate.fem_parameters.GetAddressOf()) ||
                !create_constant_buffer(
                    device.Get(), sizeof(gather_parameters), &gather_parameters,
                    candidate.gather_parameters.GetAddressOf()) ||
                !create_constant_buffer(
                    device.Get(), sizeof(integration_parameters),
                    &integration_parameters,
                    candidate.integration_parameters.GetAddressOf()))
            {
                diagnostic = "Unable to allocate Soft Body Compute Parameter buffers.";
                return false;
            }

            std::vector<gpu_soft_body_render_binding_input> render_binding_inputs(
                render_mesh.vertices.size());
            for (std::size_t render_index = 0u;
                render_index < render_mesh.vertices.size(); ++render_index)
            {
                const physics::render_vertex_binding& source_binding =
                    mesh_asset.render_bindings[render_index];
                gpu_soft_body_render_binding_input& destination =
                    render_binding_inputs[render_index];
                for (uint32_t local_index = 0u; local_index < 4u; ++local_index)
                {
                    destination.physics_vertices[local_index] =
                        source_binding.physics_vertices[local_index];
                    destination.barycentric[local_index] =
                        source_binding.barycentric[local_index];
                }
                destination.rest_offset_local[0] =
                    source_binding.rest_offset_local.x;
                destination.rest_offset_local[1] =
                    source_binding.rest_offset_local.y;
                destination.rest_offset_local[2] =
                    source_binding.rest_offset_local.z;
                destination.rest_offset_local[3] = 0.0f;
                destination.metadata[0] =
                    static_cast<uint32_t>(source_binding.kind);
            }

            std::vector<gpu_soft_body_render_vertex_input> render_vertex_inputs(
                render_mesh.vertices.size());
            for (std::size_t render_index = 0u;
                render_index < render_mesh.vertices.size(); ++render_index)
            {
                const asset::vertex& source_vertex =
                    render_mesh.vertices[render_index];
                gpu_soft_body_render_vertex_input& destination =
                    render_vertex_inputs[render_index];
                destination.texcoord[0] = source_vertex.texcoord_0.x;
                destination.texcoord[1] = source_vertex.texcoord_0.y;
                destination.fallback_normal[0] = source_vertex.normal.x;
                destination.fallback_normal[1] = source_vertex.normal.y;
                destination.fallback_normal[2] = source_vertex.normal.z;
                destination.fallback_normal[3] = 0.0f;
                destination.fallback_tangent[0] = source_vertex.tangent.x;
                destination.fallback_tangent[1] = source_vertex.tangent.y;
                destination.fallback_tangent[2] = source_vertex.tangent.z;
                destination.fallback_tangent[3] = source_vertex.tangent.w;
                destination.metadata[0] = source_vertex.attributes;
            }

            std::vector<uint32_t> vertex_triangle_offsets(
                render_mesh.vertices.size() + 1u, 0u);
            for (std::size_t index_offset = 0u;
                index_offset < render_mesh.indices.size();
                index_offset += 3u)
            {
                const uint32_t i0 = render_mesh.indices[index_offset];
                const uint32_t i1 = render_mesh.indices[index_offset + 1u];
                const uint32_t i2 = render_mesh.indices[index_offset + 2u];
                if (i0 >= candidate.render_vertex_count ||
                    i1 >= candidate.render_vertex_count ||
                    i2 >= candidate.render_vertex_count ||
                    i0 == i1 || i1 == i2 || i2 == i0)
                {
                    diagnostic =
                        "Soft Body Render Candidate topology contains an invalid or repeated triangle vertex.";
                    return false;
                }
                ++vertex_triangle_offsets[i0 + 1u];
                ++vertex_triangle_offsets[i1 + 1u];
                ++vertex_triangle_offsets[i2 + 1u];
            }
            for (std::size_t vertex_index = 1u;
                vertex_index < vertex_triangle_offsets.size();
                ++vertex_index)
            {
                const uint64_t next =
                    static_cast<uint64_t>(vertex_triangle_offsets[vertex_index]) +
                    vertex_triangle_offsets[vertex_index - 1u];
                if (next > std::numeric_limits<uint32_t>::max())
                {
                    diagnostic =
                        "Soft Body Render Vertex-to-Triangle incidence count exceeds uint32.";
                    return false;
                }
                vertex_triangle_offsets[vertex_index] =
                    static_cast<uint32_t>(next);
            }

            std::vector<uint32_t> vertex_triangle_ids(
                render_mesh.indices.size(), 0u);
            std::vector<uint32_t> triangle_cursor = vertex_triangle_offsets;
            const uint32_t render_triangle_count =
                candidate.render_index_count / 3u;
            for (uint32_t triangle_index = 0u;
                triangle_index < render_triangle_count;
                ++triangle_index)
            {
                const std::size_t index_offset =
                    static_cast<std::size_t>(triangle_index) * 3u;
                for (uint32_t local_index = 0u; local_index < 3u; ++local_index)
                {
                    const uint32_t vertex_index =
                        render_mesh.indices[index_offset + local_index];
                    vertex_triangle_ids[triangle_cursor[vertex_index]++] =
                        triangle_index;
                }
            }

            gpu_soft_body_render_parameters render_parameters;
            render_parameters.render_vertex_count = candidate.render_vertex_count;
            render_parameters.render_index_count = candidate.render_index_count;
            render_parameters.root_rotation[0] = root_transform.rotation.x;
            render_parameters.root_rotation[1] = root_transform.rotation.y;
            render_parameters.root_rotation[2] = root_transform.rotation.z;
            render_parameters.root_rotation[3] = root_transform.rotation.w;
            render_parameters.inverse_root_rotation[0] = -root_transform.rotation.x;
            render_parameters.inverse_root_rotation[1] = -root_transform.rotation.y;
            render_parameters.inverse_root_rotation[2] = -root_transform.rotation.z;
            render_parameters.inverse_root_rotation[3] = root_transform.rotation.w;

            const asset::vec3 negative_translation = {
                -root_transform.translation.x,
                -root_transform.translation.y,
                -root_transform.translation.z
            };
            const asset::vec3 inverse_quaternion = {
                render_parameters.inverse_root_rotation[0],
                render_parameters.inverse_root_rotation[1],
                render_parameters.inverse_root_rotation[2]
            };
            const asset::vec3 first_cross = {
                inverse_quaternion.y * negative_translation.z -
                    inverse_quaternion.z * negative_translation.y,
                inverse_quaternion.z * negative_translation.x -
                    inverse_quaternion.x * negative_translation.z,
                inverse_quaternion.x * negative_translation.y -
                    inverse_quaternion.y * negative_translation.x
            };
            const asset::vec3 second_cross = {
                inverse_quaternion.y * first_cross.z -
                    inverse_quaternion.z * first_cross.y,
                inverse_quaternion.z * first_cross.x -
                    inverse_quaternion.x * first_cross.z,
                inverse_quaternion.x * first_cross.y -
                    inverse_quaternion.y * first_cross.x
            };
            render_parameters.inverse_root_translation[0] =
                negative_translation.x + 2.0f *
                    (root_transform.rotation.w * first_cross.x + second_cross.x);
            render_parameters.inverse_root_translation[1] =
                negative_translation.y + 2.0f *
                    (root_transform.rotation.w * first_cross.y + second_cross.y);
            render_parameters.inverse_root_translation[2] =
                negative_translation.z + 2.0f *
                    (root_transform.rotation.w * first_cross.z + second_cross.z);
            render_parameters.inverse_root_translation[3] = 0.0f;

            const UINT render_static_bind_flags = D3D11_BIND_SHADER_RESOURCE;
            if (!create_structured_buffer_from_vector(
                    device.Get(), render_binding_inputs, render_static_bind_flags,
                    candidate.render_binding_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.render_binding_buffer.Get(),
                    candidate.render_vertex_count,
                    candidate.render_binding_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), render_vertex_inputs, render_static_bind_flags,
                    candidate.render_vertex_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.render_vertex_buffer.Get(),
                    candidate.render_vertex_count,
                    candidate.render_vertex_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), render_mesh.indices, render_static_bind_flags,
                    candidate.render_index_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.render_index_buffer.Get(),
                    candidate.render_index_count,
                    candidate.render_index_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), vertex_triangle_offsets, render_static_bind_flags,
                    candidate.render_vertex_triangle_offsets_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.render_vertex_triangle_offsets_buffer.Get(),
                    candidate.render_vertex_count + 1u,
                    candidate.render_vertex_triangle_offsets_srv.GetAddressOf()) ||
                !create_structured_buffer_from_vector(
                    device.Get(), vertex_triangle_ids, render_static_bind_flags,
                    candidate.render_vertex_triangle_ids_buffer.GetAddressOf()) ||
                !create_structured_srv(
                    device.Get(), candidate.render_vertex_triangle_ids_buffer.Get(),
                    static_cast<uint32_t>(vertex_triangle_ids.size()),
                    candidate.render_vertex_triangle_ids_srv.GetAddressOf()) ||
                !create_constant_buffer(
                    device.Get(), sizeof(render_parameters), &render_parameters,
                    candidate.render_parameters.GetAddressOf()))
            {
                diagnostic =
                    "Unable to allocate Soft Body Render Binding / Topology / Parameter resources.";
                return false;
            }

            const std::vector<gpu_float4> empty_render_stream(
                candidate.render_vertex_count, gpu_float4{});
            candidate.render_candidate_slots.resize(render_candidate_slot_count);
            for (render_candidate_mesh_slot& render_slot :
                candidate.render_candidate_slots)
            {
                if (!create_structured_buffer_from_vector(
                        device.Get(), empty_render_stream, state_bind_flags,
                        render_slot.position_buffer.GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(), render_slot.position_buffer.Get(),
                        candidate.render_vertex_count,
                        render_slot.position_srv.GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(), render_slot.position_buffer.Get(),
                        candidate.render_vertex_count,
                        render_slot.position_uav.GetAddressOf()) ||
                    !create_structured_buffer_from_vector(
                        device.Get(), empty_render_stream, state_bind_flags,
                        render_slot.normal_buffer.GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(), render_slot.normal_buffer.Get(),
                        candidate.render_vertex_count,
                        render_slot.normal_srv.GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(), render_slot.normal_buffer.Get(),
                        candidate.render_vertex_count,
                        render_slot.normal_uav.GetAddressOf()) ||
                    !create_structured_buffer_from_vector(
                        device.Get(), empty_render_stream, state_bind_flags,
                        render_slot.tangent_buffer.GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(), render_slot.tangent_buffer.Get(),
                        candidate.render_vertex_count,
                        render_slot.tangent_srv.GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(), render_slot.tangent_buffer.Get(),
                        candidate.render_vertex_count,
                        render_slot.tangent_uav.GetAddressOf()))
                {
                    diagnostic =
                        "Unable to allocate the per-Tick Render Candidate Ring Buffers.";
                    return false;
                }
            }

            out_mesh = std::move(candidate);
            return true;
        }

        bool update_soft_body_materials(
            model_slot& model,
            const model_input_snapshot& model_snapshot,
            soft_body_gpu_mesh& gpu_mesh,
            float& out_damping_rate,
            std::string& diagnostic)
        {
            const auto asset_iterator = std::find_if(
                model.mesh_physics_assets.begin(),
                model.mesh_physics_assets.end(),
                [&](const model_mesh_physics_asset& value)
                {
                    return value.mesh_index == gpu_mesh.mesh_index;
                });
            const physics_mesh_input_snapshot* mesh_snapshot =
                find_physics_mesh_input_snapshot(
                    model_snapshot, gpu_mesh.mesh_index);
            if (asset_iterator == model.mesh_physics_assets.end() ||
                mesh_snapshot == nullptr ||
                gpu_mesh.mesh_index >= model_snapshot.mesh_parameters.size() ||
                gpu_mesh.mesh_index >= model_snapshot.mesh_parameters_valid.size())
            {
                diagnostic = "Soft Body GPU Mesh lost its immutable Physics Mesh Asset owner.";
                return false;
            }

            mesh_physics_parameters parameters =
                model_snapshot.mesh_parameters_valid[gpu_mesh.mesh_index]
                    ? model_snapshot.mesh_parameters[gpu_mesh.mesh_index]
                    : mesh_physics_parameters{};
            if (!model_snapshot.mesh_parameters_valid[gpu_mesh.mesh_index])
            {
                parameters.density = static_cast<float>(gpu_mesh.rest_density);
            }

            std::vector<gpu_soft_body_tet_input> tet_inputs;
            std::vector<float> vertex_damping_rates;
            if (!build_soft_body_tet_inputs(
                    asset_iterator->physics_mesh,
                    mesh_snapshot->vertex_attribute_weights,
                    parameters,
                    tet_inputs,
                    out_damping_rate,
                    vertex_damping_rates,
                    diagnostic))
            {
                return false;
            }
            if (tet_inputs.size() != gpu_mesh.tetrahedron_count ||
                gpu_mesh.initial_inverse_masses.size() != gpu_mesh.vertex_count)
            {
                diagnostic = "Soft Body Tet / inverse-mass layout changed after GPU resources were created.";
                return false;
            }

            const double density_scale =
                gpu_mesh.rest_density / static_cast<double>(parameters.density);
            if (!finite_double(density_scale) || density_scale <= 0.0)
            {
                diagnostic = "Soft Body density remapping produced an invalid inverse-mass scale.";
                return false;
            }
            std::vector<float> remapped_inverse_masses =
                gpu_mesh.initial_inverse_masses;
            for (float& inverse_mass : remapped_inverse_masses)
            {
                const double remapped =
                    static_cast<double>(inverse_mass) * density_scale;
                if (!finite_double(remapped) || remapped < 0.0 ||
                    remapped > static_cast<double>(
                        std::numeric_limits<float>::max()))
                {
                    diagnostic = "Soft Body density remapping produced a non-finite inverse mass.";
                    return false;
                }
                inverse_mass = static_cast<float>(remapped);
            }

            if (vertex_damping_rates.size() != gpu_mesh.vertex_count ||
                mesh_snapshot->vertex_attribute_weights.size() !=
                    gpu_mesh.vertex_count ||
                gpu_mesh.animation_tracking_weights_buffer == nullptr)
            {
                diagnostic =
                    "Soft Body Damping / Animation Tracking Paint layout changed after GPU resource creation.";
                return false;
            }
            const std::size_t tracking_channel =
                static_cast<std::size_t>(
                    asset::physics_attribute_channel::animation_tracking);
            std::vector<float> tracking_weights(gpu_mesh.vertex_count, 0.0f);
            for (uint32_t vertex_index = 0u;
                vertex_index < gpu_mesh.vertex_count; ++vertex_index)
            {
                const float weight =
                    mesh_snapshot->vertex_attribute_weights[vertex_index]
                        [tracking_channel];
                if (!finite_float(weight) || weight < 0.0f || weight > 1.0f)
                {
                    diagnostic =
                        "Soft Body Animation Tracking Paint contains a non-finite or out-of-range vertex weight.";
                    return false;
                }
                tracking_weights[vertex_index] = weight;
            }

            immediate_context->UpdateSubresource(
                gpu_mesh.animation_tracking_weights_buffer.Get(),
                0u, nullptr, tracking_weights.data(), 0u, 0u);
            immediate_context->UpdateSubresource(
                gpu_mesh.tetrahedron_buffer.Get(), 0u, nullptr,
                tet_inputs.data(), 0u, 0u);
            immediate_context->UpdateSubresource(
                gpu_mesh.inverse_mass_buffer.Get(), 0u, nullptr,
                remapped_inverse_masses.data(), 0u, 0u);
            immediate_context->UpdateSubresource(
                gpu_mesh.vertex_damping_rate_buffer.Get(), 0u, nullptr,
                vertex_damping_rates.data(), 0u, 0u);
            return true;
        }

        bool world_has_soft_body_work(const world_slot& world) const
        {
            for (const model_slot& model : world.models)
            {
                if (model.occupied && !model.gpu_soft_body_meshes.empty())
                {
                    return true;
                }
            }
            return false;
        }

        bool compute_soft_body_cfl_time_limit(
            const world_slot& world,
            const tick_input_snapshot& input_snapshot,
            double& out_dt_limit,
            std::string& diagnostic) const
        {
            constexpr double cfl_factor = 0.2;
            out_dt_limit = std::numeric_limits<double>::infinity();
            diagnostic.clear();

            for (uint32_t model_index = 0u;
                model_index < world.models.size(); ++model_index)
            {
                const model_slot& model = world.models[model_index];
                if (!model.occupied)
                {
                    continue;
                }

                const model_input_snapshot* model_snapshot =
                    find_model_input_snapshot(
                        input_snapshot, model_index, model.generation);
                if (model_snapshot == nullptr)
                {
                    diagnostic =
                        "Tick Input Snapshot lost an occupied Model generation.";
                    return false;
                }

                for (const model_mesh_physics_asset& mesh_asset :
                    model.mesh_physics_assets)
                {
                    if (mesh_asset.mesh_index >= model.source_model.meshes.size() ||
                        model.source_model.meshes[mesh_asset.mesh_index].mode !=
                            asset::physics_mode::soft_body)
                    {
                        continue;
                    }

                    const physics::physics_mesh& mesh = mesh_asset.physics_mesh;
                    const physics_mesh_input_snapshot* mesh_snapshot =
                        find_physics_mesh_input_snapshot(
                            *model_snapshot, mesh_asset.mesh_index);
                    if (mesh_snapshot == nullptr ||
                        mesh_snapshot->vertex_attribute_weights.size() !=
                            mesh.rest_positions.size() ||
                        mesh.mass.size() != mesh.rest_positions.size() ||
                        mesh.tetrahedra.empty())
                    {
                        diagnostic =
                            "Soft Body CFL Preflight requires transferred per-vertex paint, lumped mass and Tet data.";
                        return false;
                    }

                    if (mesh_asset.mesh_index >= model_snapshot->mesh_parameters.size() ||
                        mesh_asset.mesh_index >= model_snapshot->mesh_parameters_valid.size())
                    {
                        diagnostic =
                            "Tick Input Snapshot lost a Model Mesh Physics Parameter entry.";
                        return false;
                    }

                    const bool parameters_valid =
                        model_snapshot->mesh_parameters_valid[mesh_asset.mesh_index];
                    mesh_physics_parameters parameters =
                        parameters_valid
                            ? model_snapshot->mesh_parameters[mesh_asset.mesh_index]
                            : mesh_physics_parameters{};

                    if (!parameters_valid)
                    {
                        double total_mass = 0.0;
                        double total_volume = 0.0;
                        for (float mass : mesh.mass)
                        {
                            total_mass += static_cast<double>(mass);
                        }
                        for (const physics::tetrahedron& tet : mesh.tetrahedra)
                        {
                            total_volume += static_cast<double>(tet.rest_volume);
                        }
                        if (!finite_double(total_mass) ||
                            !finite_double(total_volume) ||
                            total_mass <= 0.0 ||
                            total_volume <= 0.0)
                        {
                            diagnostic =
                                "Soft Body CFL Preflight could not derive the default Rest Density.";
                            return false;
                        }
                        parameters.density = static_cast<float>(
                            total_mass / total_volume);
                    }

                    const bool has_animation_trajectory =
                        model_snapshot->has_animation_playback ||
                        model_snapshot->has_animation_blend ||
                        !model_snapshot->root_samples.empty();
                    const std::size_t tracking_channel =
                        static_cast<std::size_t>(
                            asset::physics_attribute_channel::animation_tracking);
                    if (has_animation_trajectory &&
                        parameters.tracking_stiffness > 0.0f &&
                        parameters.attribute_multipliers[tracking_channel] > 0.0f)
                    {
                        const auto gpu_mesh_iterator = std::find_if(
                            model.gpu_soft_body_meshes.begin(),
                            model.gpu_soft_body_meshes.end(),
                            [&mesh_asset](const soft_body_gpu_mesh& value)
                            {
                                return value.mesh_index == mesh_asset.mesh_index;
                            });
                        if (gpu_mesh_iterator == model.gpu_soft_body_meshes.end() ||
                            !finite_double(gpu_mesh_iterator->rest_density) ||
                            gpu_mesh_iterator->rest_density <= 0.0 ||
                            mesh.inverse_mass.size() != mesh.rest_positions.size())
                        {
                            diagnostic =
                                "Animation Tracking Preflight lost its Rest Density or inverse-mass vertex layout.";
                            return false;
                        }

                        const double effective_tracking_stiffness =
                            static_cast<double>(parameters.tracking_stiffness) *
                            static_cast<double>(
                                parameters.attribute_multipliers[tracking_channel]);
                        const double density_mass_scale =
                            static_cast<double>(parameters.density) /
                            gpu_mesh_iterator->rest_density;
                        if (!finite_double(effective_tracking_stiffness) ||
                            effective_tracking_stiffness < 0.0 ||
                            !finite_double(density_mass_scale) ||
                            density_mass_scale <= 0.0)
                        {
                            diagnostic =
                                "Animation Tracking Preflight produced an invalid effective stiffness or density scale.";
                            return false;
                        }

                        double maximum_tracking_omega_squared = 0.0;
                        for (std::size_t vertex_index = 0u;
                            vertex_index < mesh.rest_positions.size();
                            ++vertex_index)
                        {
                            const float tracking_weight =
                                mesh_snapshot->vertex_attribute_weights[vertex_index]
                                    [tracking_channel];
                            const float rest_mass = mesh.mass[vertex_index];
                            const float inverse_mass = mesh.inverse_mass[vertex_index];
                            if (!finite_float(tracking_weight) ||
                                tracking_weight < 0.0f || tracking_weight > 1.0f ||
                                !finite_float(rest_mass) || rest_mass < 0.0f ||
                                !finite_float(inverse_mass) || inverse_mass < 0.0f)
                            {
                                diagnostic =
                                    "Animation Tracking Preflight encountered invalid per-vertex weight, mass or inverse mass.";
                                return false;
                            }
                            if (tracking_weight == 0.0f ||
                                rest_mass == 0.0f ||
                                inverse_mass == 0.0f)
                            {
                                continue;
                            }

                            const double effective_mass =
                                static_cast<double>(rest_mass) * density_mass_scale;
                            const double vertex_tracking_stiffness =
                                effective_tracking_stiffness *
                                static_cast<double>(tracking_weight);
                            const double omega_squared =
                                vertex_tracking_stiffness / effective_mass;
                            if (!finite_double(effective_mass) ||
                                effective_mass <= 0.0 ||
                                !finite_double(vertex_tracking_stiffness) ||
                                vertex_tracking_stiffness < 0.0 ||
                                !finite_double(omega_squared) ||
                                omega_squared < 0.0)
                            {
                                diagnostic =
                                    "Animation Tracking Preflight produced a non-finite per-vertex frequency.";
                                return false;
                            }
                            maximum_tracking_omega_squared =
                                std::max(maximum_tracking_omega_squared, omega_squared);
                        }

                        if (maximum_tracking_omega_squared > 0.0)
                        {
                            constexpr double tracking_stability_factor = 0.2;
                            const double tracking_omega =
                                std::sqrt(maximum_tracking_omega_squared);
                            const double tracking_dt_limit =
                                tracking_stability_factor / tracking_omega;
                            if (!finite_double(tracking_omega) ||
                                tracking_omega <= 0.0 ||
                                !finite_double(tracking_dt_limit) ||
                                tracking_dt_limit <= 0.0)
                            {
                                diagnostic =
                                    "Animation Tracking Preflight produced an invalid Stability Time Limit.";
                                return false;
                            }
                            out_dt_limit =
                                std::min(out_dt_limit, tracking_dt_limit);
                        }
                    }

                    const double shear_denominator =
                        2.0 * (1.0 + static_cast<double>(parameters.poisson_ratio));
                    const double bulk_denominator =
                        3.0 * (1.0 - 2.0 * static_cast<double>(parameters.poisson_ratio));
                    if (!finite_float(parameters.density) ||
                        parameters.density <= 0.0f ||
                        !finite_float(parameters.young_modulus) ||
                        parameters.young_modulus <= 0.0f ||
                        !finite_float(parameters.poisson_ratio) ||
                        parameters.poisson_ratio <= -1.0f ||
                        parameters.poisson_ratio >= 0.5f ||
                        !std::isfinite(shear_denominator) ||
                        !std::isfinite(bulk_denominator) ||
                        shear_denominator <= 0.0 ||
                        bulk_denominator <= 0.0)
                    {
                        diagnostic = "Soft Body CFL Preflight material values are invalid.";
                        return false;
                    }

                    const double shear_modulus =
                        static_cast<double>(parameters.young_modulus) /
                        shear_denominator;
                    const double bulk_modulus =
                        static_cast<double>(parameters.young_modulus) /
                        bulk_denominator;
                    const std::size_t shape_channel = static_cast<std::size_t>(
                        asset::physics_attribute_channel::shape_recovery);
                    const std::size_t volume_channel = static_cast<std::size_t>(
                        asset::physics_attribute_channel::volume);

                    const double shape_multiplier =
                        parameters.attribute_multipliers[shape_channel];
                    const double volume_multiplier =
                        parameters.attribute_multipliers[volume_channel];
                    if (!finite_double(shear_modulus) ||
                        !finite_double(bulk_modulus) ||
                        !finite_double(shape_multiplier) ||
                        !finite_double(volume_multiplier) ||
                        shape_multiplier < 0.0 ||
                        volume_multiplier < 0.0)
                    {
                        diagnostic =
                            "Soft Body CFL Preflight material scaling is non-finite or negative.";
                        return false;
                    }

                    const auto subtract_position = [](
                        const asset::vec3& left,
                        const asset::vec3& right)
                    {
                        return std::array<double, 3>{
                            static_cast<double>(left.x) - right.x,
                            static_cast<double>(left.y) - right.y,
                            static_cast<double>(left.z) - right.z
                        };
                    };
                    const auto cross_product = [](
                        const std::array<double, 3>& left,
                        const std::array<double, 3>& right)
                    {
                        return std::array<double, 3>{
                            left[1] * right[2] - left[2] * right[1],
                            left[2] * right[0] - left[0] * right[2],
                            left[0] * right[1] - left[1] * right[0]
                        };
                    };
                    const auto vector_length = [](
                        const std::array<double, 3>& value)
                    {
                        return std::sqrt(
                            value[0] * value[0] +
                            value[1] * value[1] +
                            value[2] * value[2]);
                    };

                    static const uint32_t face_vertices[4][3] = {
                        { 0u, 1u, 2u },
                        { 0u, 1u, 3u },
                        { 0u, 2u, 3u },
                        { 1u, 2u, 3u }
                    };

                    for (const physics::tetrahedron& tet : mesh.tetrahedra)
                    {
                        if (!finite_float(tet.rest_volume) || tet.rest_volume <= 0.0f)
                        {
                            diagnostic =
                                "Soft Body CFL Preflight encountered an invalid Tet Rest Volume.";
                            return false;
                        }

                        double average_shape_weight = 0.0;
                        double average_volume_weight = 0.0;
                        for (uint32_t local_vertex = 0u;
                            local_vertex < 4u; ++local_vertex)
                        {
                            const uint32_t vertex_index = tet.vertices[local_vertex];
                            if (vertex_index >= mesh.rest_positions.size())
                            {
                                diagnostic =
                                    "Soft Body CFL Preflight encountered an out-of-range Tet Vertex.";
                                return false;
                            }
                            const physics::physics_vertex_attribute_weights& weights =
                                mesh_snapshot->vertex_attribute_weights[vertex_index];
                            const float shape_weight = weights[shape_channel];
                            const float volume_weight = weights[volume_channel];
                            if (!finite_float(shape_weight) ||
                                !finite_float(volume_weight) ||
                                shape_weight < 0.0f || shape_weight > 1.0f ||
                                volume_weight < 0.0f || volume_weight > 1.0f)
                            {
                                diagnostic =
                                    "Soft Body CFL Preflight encountered invalid Shape Recovery / Volume Paint.";
                                return false;
                            }
                            average_shape_weight +=
                                static_cast<double>(shape_weight) * 0.25;
                            average_volume_weight +=
                                static_cast<double>(volume_weight) * 0.25;
                        }

                        const double effective_shear =
                            shear_modulus * shape_multiplier * average_shape_weight;
                        const double effective_bulk =
                            bulk_modulus * volume_multiplier * average_volume_weight;
                        const double wave_speed_squared =
                            (effective_bulk + 4.0 * effective_shear / 3.0) /
                            static_cast<double>(parameters.density);
                        if (!finite_double(wave_speed_squared) ||
                            wave_speed_squared < 0.0)
                        {
                            diagnostic =
                                "Soft Body CFL Preflight produced an invalid longitudinal wave speed.";
                            return false;
                        }
                        if (wave_speed_squared <= 1.0e-24)
                        {
                            // Fully painted-out Shape / Volume channels contribute no
                            // explicit elastic wave-speed restriction for this Tet.
                            continue;
                        }

                        std::array<asset::vec3, 4> positions = {};
                        for (uint32_t local_vertex = 0u;
                            local_vertex < 4u; ++local_vertex)
                        {
                            positions[local_vertex] =
                                mesh.rest_positions[tet.vertices[local_vertex]];
                        }

                        double maximum_face_area = 0.0;
                        for (const auto& face : face_vertices)
                        {
                            const std::array<double, 3> edge0 = subtract_position(
                                positions[face[1]], positions[face[0]]);
                            const std::array<double, 3> edge1 = subtract_position(
                                positions[face[2]], positions[face[0]]);
                            const double face_area = 0.5 * vector_length(
                                cross_product(edge0, edge1));
                            if (!finite_double(face_area))
                            {
                                diagnostic =
                                    "Soft Body CFL Preflight produced a non-finite Tet face area.";
                                return false;
                            }
                            maximum_face_area =
                                std::max(maximum_face_area, face_area);
                        }

                        if (maximum_face_area <= 1.0e-20)
                        {
                            diagnostic =
                                "Soft Body CFL Preflight encountered a degenerate Rest Tet face.";
                            return false;
                        }

                        const double minimum_altitude =
                            3.0 * static_cast<double>(tet.rest_volume) /
                            maximum_face_area;
                        const double wave_speed = std::sqrt(wave_speed_squared);
                        const double dt_limit =
                            cfl_factor * minimum_altitude / wave_speed;
                        if (!finite_double(minimum_altitude) ||
                            minimum_altitude <= 0.0 ||
                            !finite_double(wave_speed) ||
                            wave_speed <= 0.0 ||
                            !finite_double(dt_limit) ||
                            dt_limit <= 0.0)
                        {
                            diagnostic =
                                "Soft Body CFL Preflight produced an invalid Tet Stability Time Limit.";
                            return false;
                        }

                        out_dt_limit = std::min(out_dt_limit, dt_limit);
                    }
                }
            }
            return true;
        }

        bool reserve_render_candidate_slot(
            world_slot& world,
            uint64_t tick_sequence,
            uint32_t& out_slot_id,
            uint32_t& out_slot_generation,
            std::string& diagnostic)
        {
            out_slot_id = asset::invalid_index;
            out_slot_generation = 0u;
            diagnostic.clear();

            if (!world_has_soft_body_work(world))
            {
                return true;
            }
            if (world.render_lease_active)
            {
                diagnostic =
                    "Host Render Lease is active; a new Render Candidate cannot be admitted until Release.";
                return false;
            }

            for (uint32_t slot_id = 0u;
                slot_id < world.render_candidate_slots.size();
                ++slot_id)
            {
                if (world.latest_render_slot_id == slot_id)
                {
                    continue;
                }

                render_candidate_slot_state& slot =
                    world.render_candidate_slots[slot_id];
                if (slot.retired || slot.reserved)
                {
                    continue;
                }

                if (slot.generation == 0u)
                {
                    slot.generation = 1u;
                }
                else if (slot.generation == std::numeric_limits<uint32_t>::max())
                {
                    slot.retired = true;
                    continue;
                }
                else
                {
                    ++slot.generation;
                }

                slot.reserved = true;
                slot.committed = false;
                slot.committed_tick_sequence = tick_sequence;
                out_slot_id = slot_id;
                out_slot_generation = slot.generation;
                return true;
            }

            diagnostic =
                "Render Candidate Ring has no reusable Slot while the latest CPU-observed Commit is retained.";
            return false;
        }

        void release_render_candidate_slot(
            world_slot& world,
            uint32_t slot_id,
            uint32_t slot_generation)
        {
            if (slot_id >= world.render_candidate_slots.size())
            {
                return;
            }

            render_candidate_slot_state& slot =
                world.render_candidate_slots[slot_id];
            if (slot.generation != slot_generation || !slot.reserved)
            {
                return;
            }
            slot.reserved = false;
            slot.committed = false;
            slot.committed_tick_sequence = 0u;
            for (model_slot& model : world.models)
            {
                if (!model.occupied)
                {
                    continue;
                }
                for (soft_body_gpu_mesh& gpu_mesh : model.gpu_soft_body_meshes)
                {
                    if (slot_id < gpu_mesh.render_candidate_slots.size())
                    {
                        render_candidate_mesh_slot& mesh_slot =
                            gpu_mesh.render_candidate_slots[slot_id];
                        if (mesh_slot.committed_generation == slot_generation)
                        {
                            mesh_slot.committed_generation = 0u;
                            mesh_slot.committed_tick_sequence = 0u;
                        }
                    }
                }
            }
        }

        bool commit_render_candidate_slot(
            world_slot& world,
            uint32_t slot_id,
            uint32_t slot_generation,
            uint64_t tick_sequence)
        {
            if (slot_id >= world.render_candidate_slots.size())
            {
                return slot_id == asset::invalid_index;
            }

            render_candidate_slot_state& slot =
                world.render_candidate_slots[slot_id];
            if (!slot.reserved ||
                slot.generation != slot_generation ||
                slot.committed_tick_sequence != tick_sequence)
            {
                return false;
            }

            slot.reserved = false;
            slot.committed = true;
            world.latest_render_slot_id = slot_id;
            world.latest_render_slot_generation = slot_generation;
            for (model_slot& model : world.models)
            {
                if (!model.occupied)
                {
                    continue;
                }
                for (soft_body_gpu_mesh& gpu_mesh : model.gpu_soft_body_meshes)
                {
                    if (slot_id < gpu_mesh.render_candidate_slots.size())
                    {
                        gpu_mesh.render_candidate_slots[slot_id].committed_generation =
                            slot_generation;
                        gpu_mesh.render_candidate_slots[slot_id].committed_tick_sequence =
                            tick_sequence;
                    }
                }
            }
            return true;
        }


        bool force_target_matches_mesh(
            const target_filter& target,
            uint32_t model_index,
            const model_slot& model,
            uint32_t mesh_index) const
        {
            if (target.model.index == asset::invalid_index)
            {
                return target.mesh_index == asset::invalid_index;
            }
            if (target.model.index != model_index ||
                target.model.generation != model.generation)
            {
                return false;
            }
            return target.mesh_index == asset::invalid_index ||
                target.mesh_index == mesh_index;
        }

        bool supports_soft_body_target_scope(
            const world_slot& world,
            const target_filter& target) const
        {
            if (target.model.index == asset::invalid_index)
            {
                if (target.mesh_index != asset::invalid_index)
                {
                    return false;
                }
                for (const model_slot& model : world.models)
                {
                    if (model.occupied && !model.gpu_soft_body_meshes.empty())
                    {
                        return true;
                    }
                }
                return false;
            }

            if (target.model.index >= world.models.size())
            {
                return false;
            }
            const model_slot& model = world.models[target.model.index];
            if (!model.occupied || model.generation != target.model.generation)
            {
                return false;
            }

            if (target.mesh_index == asset::invalid_index)
            {
                return !model.gpu_soft_body_meshes.empty();
            }
            if (target.mesh_index >= model.source_model.meshes.size() ||
                model.source_model.meshes[target.mesh_index].mode !=
                    asset::physics_mode::soft_body)
            {
                return false;
            }

            return std::any_of(
                model.gpu_soft_body_meshes.begin(),
                model.gpu_soft_body_meshes.end(),
                [&target](const soft_body_gpu_mesh& value)
                {
                    return value.mesh_index == target.mesh_index;
                });
        }

        bool world_has_unsupported_physics_work(
            const world_slot& world,
            const tick_input_snapshot& input_snapshot) const
        {
            for (const force_desc& force : input_snapshot.forces)
            {
                if (!supports_soft_body_target_scope(world, force.target))
                {
                    return true;
                }
            }

            for (const impulse_desc& impulse : input_snapshot.impulses)
            {
                if (!supports_soft_body_target_scope(world, impulse.target))
                {
                    return true;
                }
            }

            if (!input_snapshot.colliders.empty())
            {
                return true;
            }

            for (uint32_t model_index = 0u;
                model_index < world.models.size(); ++model_index)
            {
                const model_slot& model = world.models[model_index];
                if (!model.occupied)
                {
                    continue;
                }

                const model_input_snapshot* model_snapshot =
                    find_model_input_snapshot(
                        input_snapshot, model_index, model.generation);
                if (model_snapshot == nullptr)
                {
                    return true;
                }
                const bool animation_active =
                    model_snapshot->has_animation_playback ||
                    model_snapshot->has_animation_blend;
                if (!model_snapshot->root_samples.empty() &&
                    !animation_active)
                {
                    // Root motion is currently evaluated by the GPU Skin
                    // Matrix path when an Animation Track supplies the
                    // immutable per-Segment Node / Skin mapping.
                    return true;
                }

                for (uint32_t mesh_index = 0u;
                    mesh_index < model.source_model.meshes.size(); ++mesh_index)
                {
                    const asset::physics_mode mode =
                        model.source_model.meshes[mesh_index].mode;
                    if (mode == asset::physics_mode::none)
                    {
                        continue;
                    }
                    const auto gpu_mesh_iterator = std::find_if(
                        model.gpu_soft_body_meshes.begin(),
                        model.gpu_soft_body_meshes.end(),
                        [mesh_index](const soft_body_gpu_mesh& value)
                        {
                            return value.mesh_index == mesh_index;
                        });
                    if (mode != asset::physics_mode::soft_body ||
                        gpu_mesh_iterator == model.gpu_soft_body_meshes.end())
                    {
                        return true;
                    }
                    if (animation_active)
                    {
                        const soft_body_gpu_mesh& gpu_mesh =
                            *gpu_mesh_iterator;
                        if (gpu_mesh.skin_node_index == asset::invalid_index ||
                            gpu_mesh.skin_index == asset::invalid_index ||
                            gpu_mesh.skin_index >= model.source_model.skins.size() ||
                            gpu_mesh.skin_node_index >= model.source_model.nodes.size() ||
                            gpu_mesh.skin_joint_count == 0u ||
                            gpu_mesh.skin_weights_srv == nullptr ||
                            gpu_mesh.animation_target_positions_srv == nullptr ||
                            gpu_mesh.animation_target_velocities_srv == nullptr ||
                            gpu_mesh.animation_target_positions_uav == nullptr ||
                            gpu_mesh.animation_target_velocities_uav == nullptr ||
                            gpu_mesh.animation_tracking_weights_srv == nullptr)
                        {
                            // Animation currently targets only registered,
                            // skinned Soft Body Meshes. Static / unskinned
                            // simulation remains available without Animation.
                            return true;
                        }
                    }
                }
            }
            return false;
        }


        bool append_soft_body_force_distribution_event(
            world_slot& world,
            std::vector<soft_body_mesh_work_state>& mesh_work,
            const target_filter& target,
            const gpu_soft_body_force_distribution_parameters& base_parameters,
            uint32_t segment_index,
            uint64_t event_id,
            bool is_impulse,
            force_distribution_global_scratch& scratch,
            simulation::d3d11_compute_graph& graph,
            std::vector<Microsoft::WRL::ComPtr<ID3D11Buffer>>&
                out_segment_parameter_buffers,
            std::string& diagnostic)
        {
            if (scratch.partial_sum_capacity == 0u ||
                scratch.partial_sum_srv == nullptr ||
                scratch.partial_sum_uav == nullptr ||
                scratch.normalized_sum_srv == nullptr ||
                scratch.normalized_sum_uav == nullptr ||
                soft_body_force_weight_shader == nullptr ||
                soft_body_force_reduce_shader == nullptr ||
                soft_body_force_finalize_shader == nullptr ||
                soft_body_force_apply_shader == nullptr)
            {
                diagnostic =
                    "Soft Body Force / Impulse scope reduction is missing a required shader or shared target resource.";
                return false;
            }

            struct target_mesh_binding
            {
                soft_body_mesh_work_state* work = nullptr;
                ID3D11Buffer* event_parameters = nullptr;
                uint32_t partial_sum_offset = 0u;
                uint32_t group_count = 0u;
            };

            std::vector<target_mesh_binding> target_meshes;
            target_meshes.reserve(mesh_work.size());
            uint64_t total_group_count = 0u;
            const std::string event_kind = is_impulse ? "impulse" : "force";
            const std::string scope_suffix =
                "_" + event_kind + "_scope_segment" +
                std::to_string(segment_index) + "_event" +
                std::to_string(event_id);

            for (soft_body_mesh_work_state& work : mesh_work)
            {
                if (work.model == nullptr || work.gpu_mesh == nullptr ||
                    !force_target_matches_mesh(
                        target, work.model_index, *work.model,
                        work.gpu_mesh->mesh_index))
                {
                    continue;
                }

                soft_body_gpu_mesh& gpu_mesh = *work.gpu_mesh;
                if (gpu_mesh.vertex_count == 0u ||
                    gpu_mesh.rest_lumped_mass_srv == nullptr ||
                    gpu_mesh.distribution_weight_srv == nullptr ||
                    gpu_mesh.distribution_weight_uav == nullptr ||
                    work.source_position_srv == nullptr ||
                    gpu_mesh.force_accumulator_srvs[0] == nullptr ||
                    gpu_mesh.force_accumulator_srvs[1] == nullptr ||
                    gpu_mesh.force_accumulator_uavs[0] == nullptr ||
                    gpu_mesh.force_accumulator_uavs[1] == nullptr ||
                    gpu_mesh.impulse_accumulator_srvs[0] == nullptr ||
                    gpu_mesh.impulse_accumulator_srvs[1] == nullptr ||
                    gpu_mesh.impulse_accumulator_uavs[0] == nullptr ||
                    gpu_mesh.impulse_accumulator_uavs[1] == nullptr)
                {
                    diagnostic =
                        "A Force / Impulse target Mesh is missing a required GPU distribution or accumulator resource.";
                    return false;
                }

                const uint32_t group_count =
                    (gpu_mesh.vertex_count - 1u) / 64u + 1u;
                if (total_group_count >
                        std::numeric_limits<uint32_t>::max() - group_count ||
                    total_group_count + group_count >
                        scratch.partial_sum_capacity)
                {
                    diagnostic =
                        "World Force / Impulse target-scope partial-sum count exceeds its checked GPU scratch capacity.";
                    return false;
                }

                gpu_soft_body_force_distribution_parameters event_parameters =
                    base_parameters;
                event_parameters.vertex_count = gpu_mesh.vertex_count;
                event_parameters.include_previous =
                    is_impulse && work.impulse_event_count == 0u ? 0u : 1u;

                Microsoft::WRL::ComPtr<ID3D11Buffer> event_parameter_buffer;
                if (!create_constant_buffer(
                        device.Get(),
                        static_cast<uint32_t>(sizeof(event_parameters)),
                        &event_parameters,
                        event_parameter_buffer.GetAddressOf()))
                {
                    diagnostic =
                        "Unable to allocate an immutable per-Mesh Force / Impulse Event Parameter Buffer.";
                    return false;
                }
                out_segment_parameter_buffers.push_back(
                    std::move(event_parameter_buffer));
                ID3D11Buffer* event_parameter_buffer_raw =
                    out_segment_parameter_buffers.back().Get();

                gpu_soft_body_distribution_reduce_parameters reduce_parameters;
                reduce_parameters.vertex_count = gpu_mesh.vertex_count;
                reduce_parameters.group_count = group_count;
                reduce_parameters.partial_sum_offset =
                    static_cast<uint32_t>(total_group_count);
                Microsoft::WRL::ComPtr<ID3D11Buffer> reduce_parameter_buffer;
                if (!create_constant_buffer(
                        device.Get(),
                        static_cast<uint32_t>(sizeof(reduce_parameters)),
                        &reduce_parameters,
                        reduce_parameter_buffer.GetAddressOf()))
                {
                    diagnostic =
                        "Unable to allocate an immutable per-Mesh target-scope Reduction Parameter Buffer.";
                    return false;
                }
                out_segment_parameter_buffers.push_back(
                    std::move(reduce_parameter_buffer));
                ID3D11Buffer* reduce_parameter_buffer_raw =
                    out_segment_parameter_buffers.back().Get();

                const std::string mesh_suffix =
                    "_m" + std::to_string(work.model_index) +
                    "_mesh" + std::to_string(gpu_mesh.mesh_index) +
                    scope_suffix;

                simulation::compute_pass_desc weight_pass;
                weight_pass.name = "soft_body" + mesh_suffix + "_weight";
                weight_pass.shader = soft_body_force_weight_shader.Get();
                weight_pass.shader_resources = {
                    { 0u, work.source_position_srv },
                    { 1u, gpu_mesh.rest_lumped_mass_srv.Get() }
                };
                weight_pass.unordered_access_views = {
                    { 0u, gpu_mesh.distribution_weight_uav.Get(), 0xffffffffu },
                    { 1u, world.commit_control_uav.Get(), 0xffffffffu }
                };
                weight_pass.constant_buffers = {
                    { 0u, event_parameter_buffer_raw }
                };
                weight_pass.group_count_x = group_count;
                if (!graph.add_pass(weight_pass, diagnostic))
                {
                    return false;
                }

                simulation::compute_pass_desc reduce_pass;
                reduce_pass.name = "soft_body" + mesh_suffix + "_reduce";
                reduce_pass.shader = soft_body_force_reduce_shader.Get();
                reduce_pass.shader_resources = {
                    { 2u, gpu_mesh.distribution_weight_srv.Get() }
                };
                reduce_pass.unordered_access_views = {
                    { 2u, scratch.partial_sum_uav.Get(), 0xffffffffu },
                    { 1u, world.commit_control_uav.Get(), 0xffffffffu }
                };
                reduce_pass.constant_buffers = {
                    { 1u, reduce_parameter_buffer_raw }
                };
                reduce_pass.group_count_x = group_count;
                if (!graph.add_pass(reduce_pass, diagnostic))
                {
                    return false;
                }

                target_mesh_binding binding;
                binding.work = &work;
                binding.event_parameters = event_parameter_buffer_raw;
                binding.partial_sum_offset =
                    static_cast<uint32_t>(total_group_count);
                binding.group_count = group_count;
                target_meshes.push_back(binding);
                total_group_count += group_count;
            }

            if (target_meshes.empty() || total_group_count == 0u ||
                total_group_count > std::numeric_limits<uint32_t>::max())
            {
                diagnostic =
                    "Force / Impulse target scope contains no registered Soft Body Mesh support.";
                return false;
            }

            gpu_soft_body_distribution_reduce_parameters finalize_parameters;
            finalize_parameters.vertex_count = 0u;
            finalize_parameters.group_count =
                static_cast<uint32_t>(total_group_count);
            finalize_parameters.partial_sum_offset = 0u;
            Microsoft::WRL::ComPtr<ID3D11Buffer> finalize_parameter_buffer;
            if (!create_constant_buffer(
                    device.Get(),
                    static_cast<uint32_t>(sizeof(finalize_parameters)),
                    &finalize_parameters,
                    finalize_parameter_buffer.GetAddressOf()))
            {
                diagnostic =
                    "Unable to allocate the immutable target-scope Force / Impulse Finalize Parameters.";
                return false;
            }
            out_segment_parameter_buffers.push_back(
                std::move(finalize_parameter_buffer));
            ID3D11Buffer* finalize_parameter_buffer_raw =
                out_segment_parameter_buffers.back().Get();

            simulation::compute_pass_desc finalize_pass;
            finalize_pass.name =
                "soft_body" + scope_suffix + "_finalize_global_denominator";
            finalize_pass.shader = soft_body_force_finalize_shader.Get();
            finalize_pass.shader_resources = {
                { 2u, scratch.partial_sum_srv.Get() }
            };
            finalize_pass.unordered_access_views = {
                { 3u, scratch.normalized_sum_uav.Get(), 0xffffffffu },
                { 1u, world.commit_control_uav.Get(), 0xffffffffu }
            };
            finalize_pass.constant_buffers = {
                { 1u, finalize_parameter_buffer_raw }
            };
            finalize_pass.group_count_x = 1u;
            if (!graph.add_pass(finalize_pass, diagnostic))
            {
                return false;
            }

            for (const target_mesh_binding& binding : target_meshes)
            {
                soft_body_mesh_work_state& work = *binding.work;
                soft_body_gpu_mesh& gpu_mesh = *work.gpu_mesh;

                ID3D11ShaderResourceView* previous_accumulator_srv = nullptr;
                ID3D11UnorderedAccessView* next_accumulator_uav = nullptr;
                uint32_t accumulator_index = 0u;
                if (is_impulse)
                {
                    accumulator_index = work.impulse_event_count & 1u;
                    previous_accumulator_srv =
                        work.impulse_event_count == 0u
                            ? gpu_mesh.vertex_force_srv.Get()
                            : gpu_mesh.impulse_accumulator_srvs[
                                (work.impulse_event_count - 1u) & 1u].Get();
                    next_accumulator_uav =
                        gpu_mesh.impulse_accumulator_uavs[accumulator_index].Get();
                }
                else
                {
                    accumulator_index = work.force_event_count & 1u;
                    previous_accumulator_srv =
                        work.force_event_count == 0u
                            ? gpu_mesh.vertex_force_srv.Get()
                            : gpu_mesh.force_accumulator_srvs[
                                (work.force_event_count - 1u) & 1u].Get();
                    next_accumulator_uav =
                        gpu_mesh.force_accumulator_uavs[accumulator_index].Get();
                }

                simulation::compute_pass_desc apply_pass;
                apply_pass.name =
                    std::string("soft_body") +
                    (is_impulse ? "_impulse" : "_force") +
                    "_m" + std::to_string(work.model_index) +
                    "_mesh" + std::to_string(gpu_mesh.mesh_index) +
                    scope_suffix + "_apply";
                apply_pass.shader = soft_body_force_apply_shader.Get();
                apply_pass.shader_resources = {
                    { 3u, gpu_mesh.distribution_weight_srv.Get() },
                    { 4u, scratch.normalized_sum_srv.Get() },
                    { 5u, previous_accumulator_srv }
                };
                apply_pass.unordered_access_views = {
                    { 4u, next_accumulator_uav, 0xffffffffu },
                    { 1u, world.commit_control_uav.Get(), 0xffffffffu }
                };
                apply_pass.constant_buffers = {
                    { 0u, binding.event_parameters }
                };
                apply_pass.group_count_x =
                    (gpu_mesh.vertex_count - 1u) / 64u + 1u;
                if (!graph.add_pass(apply_pass, diagnostic))
                {
                    return false;
                }

                ID3D11ShaderResourceView* applied_accumulator_srv =
                    is_impulse
                        ? gpu_mesh.impulse_accumulator_srvs[accumulator_index].Get()
                        : gpu_mesh.force_accumulator_srvs[accumulator_index].Get();
                if (is_impulse)
                {
                    work.integrated_impulse_srv = applied_accumulator_srv;
                    ++work.impulse_event_count;
                }
                else
                {
                    work.integrated_force_srv = applied_accumulator_srv;
                    ++work.force_event_count;
                }
            }

            return true;
        }

        bool append_soft_body_animation_skinning_pass(
            const model_slot& model,
            const model_input_snapshot& model_snapshot,
            const model_animation_tick_schedule& schedule,
            soft_body_mesh_work_state& work,
            double world_time,
            double segment_duration_seconds,
            uint64_t animation_input_version,
            std::vector<animation_packet_work_state>&
                out_animation_packet_resources,
            simulation::d3d11_compute_graph& graph,
            std::vector<Microsoft::WRL::ComPtr<ID3D11Buffer>>&
                out_segment_parameter_buffers,
            std::vector<Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>>&
                out_segment_shader_resource_views,
            std::vector<Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>>&
                out_segment_unordered_access_views,
            std::string& diagnostic)
        {
            diagnostic.clear();
            if (work.gpu_mesh == nullptr)
            {
                diagnostic = "Animation Skinning work item lost its GPU Mesh.";
                return false;
            }
            const soft_body_gpu_mesh& gpu_mesh = *work.gpu_mesh;
            if (soft_body_animation_skinning_shader == nullptr ||
                gpu_mesh.skin_index == asset::invalid_index ||
                gpu_mesh.skin_node_index == asset::invalid_index ||
                gpu_mesh.skin_joint_count == 0u ||
                gpu_mesh.skin_weights_srv == nullptr ||
                gpu_mesh.rest_positions_srv == nullptr ||
                gpu_mesh.animation_target_positions_uav == nullptr ||
                gpu_mesh.animation_target_velocities_uav == nullptr ||
                gpu_mesh.animation_target_status_uav == nullptr ||
                gpu_mesh.animation_target_status_buffer == nullptr ||
                !finite_double(world_time) ||
                !finite_double(segment_duration_seconds) ||
                segment_duration_seconds <= 0.0)
            {
                diagnostic =
                    "Animated Soft Body Mesh is missing its Skin / Joint mapping or GPU LBS target resources.";
                return false;
            }

            const asset::animation_clip* source_clip = nullptr;
            const asset::animation_clip* target_clip = nullptr;
            const physics::scheduled_animation_clip_span* playback_span = nullptr;
            const physics::animation_blend_clip_span* blend_span = nullptr;
            double span_local_u = 0.0;
            double span_duration = 0.0;
            uint32_t selected_span_record_index = asset::invalid_index;

            if (schedule.has_blend)
            {
                const auto iterator = std::find_if(
                    schedule.blend_spans.begin(),
                    schedule.blend_spans.end(),
                    [world_time](const physics::animation_blend_clip_span& value)
                    {
                        return world_time >= value.world_time_start &&
                            world_time <
                                value.world_time_start + value.world_time_duration;
                    });
                if (iterator == schedule.blend_spans.end() ||
                    model_snapshot.animation_blend.source_track.clip_index >=
                        model.source_model.animations.size() ||
                    model_snapshot.animation_blend.target_track.clip_index >=
                        model.source_model.animations.size())
                {
                    diagnostic =
                        "Animation Blend Segment start does not resolve one immutable synchronized Source / Target / Weight Span.";
                    return false;
                }
                blend_span = &(*iterator);
                selected_span_record_index = static_cast<uint32_t>(
                    std::distance(schedule.blend_spans.begin(), iterator));
                source_clip = &model.source_model.animations[
                    model_snapshot.animation_blend.source_track.clip_index];
                target_clip = &model.source_model.animations[
                    model_snapshot.animation_blend.target_track.clip_index];
                span_duration = blend_span->world_time_duration;
                span_local_u =
                    (world_time - blend_span->world_time_start) / span_duration;
            }
            else
            {
                const auto iterator = std::find_if(
                    schedule.playback_spans.begin(),
                    schedule.playback_spans.end(),
                    [world_time](const physics::scheduled_animation_clip_span& value)
                    {
                        return world_time >= value.world_time_start &&
                            world_time <
                                value.world_time_start + value.world_time_duration;
                    });
                if (iterator == schedule.playback_spans.end() ||
                    !model_snapshot.has_animation_playback ||
                    model_snapshot.animation_playback.track.clip_index >=
                        model.source_model.animations.size())
                {
                    diagnostic =
                        "Animation Playback Segment start does not resolve an immutable Key-bounded Clip Span.";
                    return false;
                }
                playback_span = &(*iterator);
                selected_span_record_index = static_cast<uint32_t>(
                    std::distance(schedule.playback_spans.begin(), iterator));
                source_clip = &model.source_model.animations[
                    model_snapshot.animation_playback.track.clip_index];
                span_duration = playback_span->world_time_duration;
                span_local_u =
                    (world_time - playback_span->world_time_start) / span_duration;
            }

            if (source_clip == nullptr ||
                (blend_span != nullptr && target_clip == nullptr) ||
                !finite_double(span_duration) ||
                span_duration <= 0.0 ||
                !finite_double(span_local_u) ||
                span_local_u < 0.0 || span_local_u > 1.0 ||
                selected_span_record_index == asset::invalid_index)
            {
                diagnostic =
                    "Animation Segment has an invalid Clip reference, World duration, normalized Span coordinate or selected Span index.";
                return false;
            }
            const double selected_span_world_time_start =
                blend_span != nullptr
                    ? blend_span->world_time_start
                    : playback_span->world_time_start;

            auto packet_cache_iterator = std::find_if(
                out_animation_packet_resources.begin(),
                out_animation_packet_resources.end(),
                [&schedule, &model](const animation_packet_work_state& value)
                {
                    return value.model_index == schedule.model_index &&
                        value.model_generation == model.generation;
                });

            if (packet_cache_iterator ==
                out_animation_packet_resources.end())
            {
                physics::animation_trajectory_packet_build_desc packet_desc;
                packet_desc.animation_input_version = animation_input_version;
                const std::size_t scheduled_span_count =
                    schedule.has_blend
                        ? schedule.blend_spans.size()
                        : schedule.playback_spans.size();
                if (scheduled_span_count == 0u ||
                    scheduled_span_count >
                        static_cast<std::size_t>(packet_desc.maximum_span_count))
                {
                    diagnostic =
                        "Animation Tick Schedule has an empty or over-capacity immutable Span table.";
                    return false;
                }
                packet_desc.spans.reserve(scheduled_span_count);

                if (schedule.has_blend)
                {
                    for (std::size_t span_index = 0u;
                        span_index < schedule.blend_spans.size();
                        ++span_index)
                    {
                        const physics::animation_blend_clip_span& scheduled =
                            schedule.blend_spans[span_index];
                        physics::animation_trajectory_span_input span_input;
                        span_input.span_id = static_cast<uint32_t>(span_index);
                        span_input.world_time_start = scheduled.world_time_start;
                        span_input.world_time_duration =
                            scheduled.world_time_duration;
                        span_input.bounds_are_certified = false;

                        physics::animation_trajectory_track_span_input source_input;
                        source_input.clip_index =
                            model_snapshot.animation_blend.source_track.clip_index;
                        source_input.scheduled = scheduled.source;
                        source_input.loop =
                            model_snapshot.animation_blend.source_track.loop;
                        source_input.reverse_playback =
                            scheduled.source.clip.clip_time_delta < 0.0;

                        physics::animation_trajectory_track_span_input target_input;
                        target_input.clip_index =
                            model_snapshot.animation_blend.target_track.clip_index;
                        target_input.scheduled = scheduled.target;
                        target_input.loop =
                            model_snapshot.animation_blend.target_track.loop;
                        target_input.reverse_playback =
                            scheduled.target.clip.clip_time_delta < 0.0;

                        span_input.tracks.push_back(std::move(source_input));
                        span_input.tracks.push_back(std::move(target_input));
                        span_input.has_blend = true;
                        span_input.source_track_index = 0u;
                        span_input.target_track_index = 1u;
                        span_input.blend_curve.blend_start_time =
                            model_snapshot.animation_blend.blend_start_time;
                        span_input.blend_curve.blend_end_time =
                            model_snapshot.animation_blend.blend_end_time;
                        span_input.blend_curve.weight_start =
                            model_snapshot.animation_blend.weight_start;
                        span_input.blend_curve.weight_end =
                            model_snapshot.animation_blend.weight_end;
                        span_input.blend_curve.tangent_start =
                            model_snapshot.animation_blend.tangent_start;
                        span_input.blend_curve.tangent_end =
                            model_snapshot.animation_blend.tangent_end;
                        packet_desc.spans.push_back(std::move(span_input));
                    }
                }
                else
                {
                    for (std::size_t span_index = 0u;
                        span_index < schedule.playback_spans.size();
                        ++span_index)
                    {
                        const physics::scheduled_animation_clip_span& scheduled =
                            schedule.playback_spans[span_index];
                        physics::animation_trajectory_span_input span_input;
                        span_input.span_id = static_cast<uint32_t>(span_index);
                        span_input.world_time_start = scheduled.world_time_start;
                        span_input.world_time_duration =
                            scheduled.world_time_duration;
                        span_input.bounds_are_certified = false;

                        physics::animation_trajectory_track_span_input playback_input;
                        playback_input.clip_index =
                            model_snapshot.animation_playback.track.clip_index;
                        playback_input.scheduled = scheduled;
                        playback_input.loop =
                            model_snapshot.animation_playback.track.loop;
                        playback_input.reverse_playback =
                            scheduled.clip.clip_time_delta < 0.0;
                        span_input.tracks.push_back(std::move(playback_input));
                        packet_desc.spans.push_back(std::move(span_input));
                    }
                }

                physics::animation_trajectory_packet_build_result packet_build =
                    physics::build_animation_trajectory_packet(
                        model.source_model, packet_desc);
                if (!packet_build.success)
                {
                    diagnostic =
                        "Immutable multi-Span GPU Animation Packet construction failed before Segment Submission: " +
                        packet_build.message;
                    return false;
                }

                physics::animation_span_cursor_gpu_record initial_cursor;
                if (!physics::initialize_animation_span_cursor(
                        packet_build.value,
                        selected_span_record_index,
                        world_time,
                        initial_cursor,
                        diagnostic))
                {
                    diagnostic =
                        "Unable to initialize the immutable Packet's initial Segment Cursor: " +
                        diagnostic;
                    return false;
                }

                out_animation_packet_resources.emplace_back();
                packet_cache_iterator =
                    std::prev(out_animation_packet_resources.end());
                packet_cache_iterator->model_index = schedule.model_index;
                packet_cache_iterator->model_generation = model.generation;
                packet_cache_iterator->packet =
                    std::move(packet_build.value);
                if (!physics::upload_animation_trajectory_packet(
                        device.Get(),
                        packet_cache_iterator->packet,
                        initial_cursor,
                        packet_cache_iterator->resources,
                        diagnostic))
                {
                    diagnostic =
                        "Unable to upload the immutable multi-Span Animation Packet for GPU evaluation: " +
                        diagnostic;
                    return false;
                }
            }
            else if (packet_cache_iterator->packet.animation_input_version !=
                animation_input_version)
            {
                diagnostic =
                    "Animation Packet cache reused a Model across incompatible immutable Input Snapshot Versions.";
                return false;
            }

            animation_packet_work_state& packet_cache =
                *packet_cache_iterator;
            const physics::animation_trajectory_packet& packet_data =
                packet_cache.packet;
            physics::animation_trajectory_gpu_resources& packet_resources =
                packet_cache.resources;
            if (selected_span_record_index >= packet_data.span_records.size())
            {
                diagnostic =
                    "Animation Segment selected a Span outside the cached immutable multi-Span Packet.";
                return false;
            }

            if (packet_resources.cursor_srv() == nullptr ||
                packet_resources.cursor_uav() == nullptr)
            {
                diagnostic =
                    "Immutable Model Animation Packet lost its shared mutable Span Cursor.";
                return false;
            }

            const physics::animation_span_gpu_record& packet_span =
                packet_data.span_records[selected_span_record_index];
            const uint32_t node_count = static_cast<uint32_t>(
                model.source_model.nodes.size());
            if (node_count == 0u ||
                packet_span.track_record_count == 0u ||
                packet_span.track_record_count > 2u ||
                packet_span.channel_record_count !=
                    node_count * packet_span.track_record_count ||
                (packet_span.track_record_count == 2u &&
                    packet_span.blend_record_count != 1u) ||
                (packet_span.track_record_count == 1u &&
                    packet_span.blend_record_count != 0u) ||
                packet_data.static_node_matrices.size() != node_count)
            {
                diagnostic =
                    std::string(
                        "GPU Animation Packet does not contain one dense topological Channel table for this Model. ") +
                    "node_count=" + std::to_string(node_count) +
                    " span_index=" + std::to_string(selected_span_record_index) +
                    " span_count=" + std::to_string(packet_data.span_records.size()) +
                    " track_count=" + std::to_string(packet_span.track_record_count) +
                    " channel_count=" + std::to_string(packet_span.channel_record_count) +
                    " blend_count=" + std::to_string(packet_span.blend_record_count) +
                    " static_matrix_count=" + std::to_string(packet_data.static_node_matrices.size()) +
                    " first_track=" + std::to_string(packet_span.first_track_record) +
                    " first_channel=" + std::to_string(packet_span.first_channel_record) +
                    " first_blend=" + std::to_string(packet_span.first_blend_record);
                return false;
            }

            struct alignas(16) channel_evaluation_parameters
            {
                uint32_t channel_count = 0u;
                uint32_t first_channel_record = 0u;
                uint32_t expected_span_record_index = 0u;
                uint32_t reserved = 0u;
            };
            struct alignas(16) blend_evaluation_parameters
            {
                uint32_t node_count = 0u;
                uint32_t expected_span_record_index = 0u;
                uint32_t blend_record_index = 0u;
                uint32_t reserved = 0u;
            };
            struct alignas(16) hierarchy_evaluation_parameters
            {
                uint32_t node_count = 0u;
                uint32_t static_node_matrix_count = 0u;
                uint32_t hierarchy_level = 0u;
                uint32_t reserved = 0u;
            };
            struct alignas(16) skin_matrix_evaluation_parameters
            {
                uint32_t skin_record_index = 0u;
                uint32_t skin_record_count = 0u;
                uint32_t track_channel_count = 0u;
                uint32_t skin_joint_record_count = 0u;
                uint32_t inverse_bind_matrix_count = 0u;
                uint32_t expected_mesh_node_index = asset::invalid_index;
                uint32_t reserved[2] = {};
                physics::gpu_matrix4x4 root_transform;
                physics::gpu_matrix4x4 root_transform_derivative;
            };
            static_assert(sizeof(channel_evaluation_parameters) == 16u);
            static_assert(sizeof(blend_evaluation_parameters) == 16u);
            static_assert(sizeof(hierarchy_evaluation_parameters) == 16u);
            static_assert(sizeof(skin_matrix_evaluation_parameters) == 160u);

            const double normalized_span_coordinate =
                (world_time - selected_span_world_time_start) / span_duration;
            if (!finite_double(normalized_span_coordinate) ||
                normalized_span_coordinate < 0.0 ||
                normalized_span_coordinate > 1.0 ||
                !finite_double(1.0 / span_duration))
            {
                diagnostic =
                    "GPU Animation Segment normalized coordinate or inverse duration is invalid.";
                return false;
            }
            const float gpu_span_local_u =
                static_cast<float>(normalized_span_coordinate);
            const float inverse_span_duration =
                static_cast<float>(1.0 / span_duration);
            if (!std::isfinite(gpu_span_local_u) ||
                !std::isfinite(inverse_span_duration) ||
                inverse_span_duration <= 0.0f)
            {
                diagnostic =
                    "GPU Animation Segment coordinate exceeds finite float range.";
                return false;
            }

            const auto create_rw_buffer =
                [this](
                    uint32_t element_count,
                    uint32_t element_stride,
                    const void* initial_data,
                    Microsoft::WRL::ComPtr<ID3D11Buffer>& out_buffer,
                    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>& out_srv,
                    Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>& out_uav)
            {
                if (element_count == 0u ||
                    element_stride == 0u ||
                    element_count >
                        std::numeric_limits<uint32_t>::max() / element_stride)
                {
                    return false;
                }
                const uint32_t byte_width =
                    element_count * element_stride;
                return create_structured_buffer(
                        device.Get(),
                        byte_width,
                        element_stride,
                        D3D11_BIND_SHADER_RESOURCE |
                            D3D11_BIND_UNORDERED_ACCESS,
                        initial_data,
                        out_buffer.GetAddressOf()) &&
                    create_structured_srv(
                        device.Get(),
                        out_buffer.Get(),
                        element_count,
                        out_srv.GetAddressOf()) &&
                    create_structured_uav(
                        device.Get(),
                        out_buffer.Get(),
                        element_count,
                        out_uav.GetAddressOf());
            };

            const uint32_t clear_status = 0u;
            Microsoft::WRL::ComPtr<ID3D11Buffer> segment_status_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> segment_status_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> segment_status_uav;
            if (!create_rw_buffer(
                    1u, static_cast<uint32_t>(sizeof(clear_status)),
                    &clear_status, segment_status_buffer,
                    segment_status_srv, segment_status_uav))
            {
                diagnostic =
                    "Unable to allocate the shared Segment Animation Evaluation Status.";
                return false;
            }

            std::vector<physics::animation_node_local_transform_gpu_record>
                initial_local_transforms(node_count);
            Microsoft::WRL::ComPtr<ID3D11Buffer> source_local_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> source_local_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> source_local_uav;
            if (!create_rw_buffer(
                    node_count,
                    static_cast<uint32_t>(sizeof(
                        physics::animation_node_local_transform_gpu_record)),
                    initial_local_transforms.data(),
                    source_local_buffer, source_local_srv, source_local_uav))
            {
                diagnostic =
                    "Unable to allocate the GPU Source Local Transform / Derivative output.";
                return false;
            }

            Microsoft::WRL::ComPtr<ID3D11Buffer> target_local_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> target_local_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> target_local_uav;
            if (packet_span.track_record_count == 2u &&
                !create_rw_buffer(
                    node_count,
                    static_cast<uint32_t>(sizeof(
                        physics::animation_node_local_transform_gpu_record)),
                    initial_local_transforms.data(),
                    target_local_buffer, target_local_srv, target_local_uav))
            {
                diagnostic =
                    "Unable to allocate the GPU Target Local Transform / Derivative output.";
                return false;
            }

            Microsoft::WRL::ComPtr<ID3D11Buffer> blended_local_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> blended_local_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> blended_local_uav;
            if (packet_span.track_record_count == 2u &&
                !create_rw_buffer(
                    node_count,
                    static_cast<uint32_t>(sizeof(
                        physics::animation_node_local_transform_gpu_record)),
                    initial_local_transforms.data(),
                    blended_local_buffer, blended_local_srv, blended_local_uav))
            {
                diagnostic =
                    "Unable to allocate the GPU Blended Local Transform / Derivative output.";
                return false;
            }

            for (uint32_t track_index = 0u;
                track_index < packet_span.track_record_count;
                ++track_index)
            {
                const physics::animation_track_gpu_record& track_record =
                    packet_data.track_records[
                        packet_span.first_track_record + track_index];
                if (track_record.channel_record_count != node_count ||
                    track_record.first_channel_record >
                        packet_data.channel_records.size() ||
                    track_record.channel_record_count >
                        packet_data.channel_records.size() -
                            track_record.first_channel_record)
                {
                    diagnostic =
                        "GPU Animation Track does not reference a complete dense Node Channel range.";
                    return false;
                }

                channel_evaluation_parameters parameters;
                parameters.channel_count = track_record.channel_record_count;
                parameters.first_channel_record =
                    track_record.first_channel_record;
                parameters.expected_span_record_index = selected_span_record_index;
                Microsoft::WRL::ComPtr<ID3D11Buffer> parameter_buffer;
                if (!create_constant_buffer(
                        device.Get(),
                        static_cast<uint32_t>(sizeof(parameters)),
                        &parameters,
                        parameter_buffer.GetAddressOf()))
                {
                    diagnostic =
                        "Unable to allocate GPU Animation Channel Evaluation Parameters.";
                    return false;
                }

                simulation::compute_pass_desc channel_pass;
                channel_pass.name =
                    "animation_channel_evaluate_m" +
                    std::to_string(schedule.model_index) + "_mesh" +
                    std::to_string(gpu_mesh.mesh_index) + "_track" +
                    std::to_string(track_index) + "_world_time" +
                    format_double_round_trip(world_time);
                channel_pass.shader = animation_channel_evaluate_shader.Get();
                channel_pass.shader_resources = {
                    { 0u, packet_resources.channel_srv() },
                    { 1u, packet_resources.cursor_srv() },
                    { 2u, packet_resources.span_srv() }
                };
                channel_pass.unordered_access_views = {
                    { 0u,
                        track_index == 0u
                            ? source_local_uav.Get()
                            : target_local_uav.Get(),
                        0xffffffffu },
                    { 1u, segment_status_uav.Get(), 0xffffffffu }
                };
                channel_pass.constant_buffers = {
                    { 0u, parameter_buffer.Get() }
                };
                channel_pass.group_count_x =
                    std::max(1u, (node_count + 63u) / 64u);
                if (!graph.add_pass(channel_pass, diagnostic))
                {
                    return false;
                }
                out_segment_parameter_buffers.push_back(
                    std::move(parameter_buffer));
            }

            ID3D11ShaderResourceView* local_transform_srv =
                source_local_srv.Get();
            if (packet_span.track_record_count == 2u)
            {
                blend_evaluation_parameters parameters;
                parameters.node_count = node_count;
                parameters.expected_span_record_index = selected_span_record_index;
                parameters.blend_record_index = packet_span.first_blend_record;
                Microsoft::WRL::ComPtr<ID3D11Buffer> parameter_buffer;
                if (!create_constant_buffer(
                        device.Get(),
                        static_cast<uint32_t>(sizeof(parameters)),
                        &parameters,
                        parameter_buffer.GetAddressOf()))
                {
                    diagnostic =
                        "Unable to allocate GPU Animation Blend Parameters.";
                    return false;
                }

                simulation::compute_pass_desc blend_pass;
                blend_pass.name =
                    "animation_blend_local_transform_m" +
                    std::to_string(schedule.model_index) + "_mesh" +
                    std::to_string(gpu_mesh.mesh_index) + "_world_time" +
                    format_double_round_trip(world_time);
                blend_pass.shader =
                    animation_blend_local_transform_shader.Get();
                blend_pass.shader_resources = {
                    { 0u, source_local_srv.Get() },
                    { 1u, target_local_srv.Get() },
                    { 2u, packet_resources.blend_srv() },
                    { 3u, packet_resources.cursor_srv() },
                    { 4u, packet_resources.span_srv() }
                };
                blend_pass.unordered_access_views = {
                    { 0u, blended_local_uav.Get(), 0xffffffffu },
                    { 1u, segment_status_uav.Get(), 0xffffffffu }
                };
                blend_pass.constant_buffers = {
                    { 0u, parameter_buffer.Get() }
                };
                blend_pass.group_count_x =
                    std::max(1u, (node_count + 63u) / 64u);
                if (!graph.add_pass(blend_pass, diagnostic))
                {
                    return false;
                }
                out_segment_parameter_buffers.push_back(
                    std::move(parameter_buffer));
                local_transform_srv = blended_local_srv.Get();
            }

            std::vector<physics::gpu_matrix4x4> initial_global_matrices(
                node_count);
            Microsoft::WRL::ComPtr<ID3D11Buffer> global_matrix_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> global_matrix_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> global_matrix_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> global_derivative_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> global_derivative_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> global_derivative_uav;
            if (!create_rw_buffer(
                    node_count,
                    static_cast<uint32_t>(sizeof(physics::gpu_matrix4x4)),
                    initial_global_matrices.data(),
                    global_matrix_buffer, global_matrix_srv, global_matrix_uav) ||
                !create_rw_buffer(
                    node_count,
                    static_cast<uint32_t>(sizeof(physics::gpu_matrix4x4)),
                    initial_global_matrices.data(),
                    global_derivative_buffer,
                    global_derivative_srv,
                    global_derivative_uav))
            {
                diagnostic =
                    "Unable to allocate GPU Animation Global Matrix / Derivative outputs.";
                return false;
            }

            uint32_t maximum_hierarchy_depth = 0u;
            for (uint32_t channel_index = 0u;
                channel_index < node_count;
                ++channel_index)
            {
                uint32_t depth = 0u;
                uint32_t parent_index =
                    packet_data.channel_records[
                        packet_span.first_channel_record + channel_index].parent_index;
                while (parent_index != asset::invalid_index)
                {
                    if (parent_index >= node_count || depth >= node_count)
                    {
                        diagnostic =
                            "GPU Animation Packet contains a non-topological Parent Channel reference.";
                        return false;
                    }
                    ++depth;
                    parent_index =
                        packet_data.channel_records[
                            packet_span.first_channel_record + parent_index].parent_index;
                }
                maximum_hierarchy_depth =
                    std::max(maximum_hierarchy_depth, depth);
            }

            for (uint32_t hierarchy_level = 0u;
                hierarchy_level <= maximum_hierarchy_depth;
                ++hierarchy_level)
            {
                hierarchy_evaluation_parameters parameters;
                parameters.node_count = node_count;
                parameters.static_node_matrix_count =
                    static_cast<uint32_t>(
                        packet_data.static_node_matrices.size());
                parameters.hierarchy_level = hierarchy_level;
                Microsoft::WRL::ComPtr<ID3D11Buffer> parameter_buffer;
                if (!create_constant_buffer(
                        device.Get(),
                        static_cast<uint32_t>(sizeof(parameters)),
                        &parameters,
                        parameter_buffer.GetAddressOf()))
                {
                    diagnostic =
                        "Unable to allocate GPU Animation Hierarchy Parameters.";
                    return false;
                }

                simulation::compute_pass_desc hierarchy_pass;
                hierarchy_pass.name =
                    "animation_node_hierarchy_evaluate_m" +
                    std::to_string(schedule.model_index) + "_mesh" +
                    std::to_string(gpu_mesh.mesh_index) + "_depth" +
                    std::to_string(hierarchy_level) + "_world_time" +
                    format_double_round_trip(world_time);
                hierarchy_pass.shader =
                    animation_node_hierarchy_evaluate_shader.Get();
                hierarchy_pass.shader_resources = {
                    { 0u, local_transform_srv },
                    { 1u, packet_resources.static_node_matrix_srv() }
                };
                hierarchy_pass.unordered_access_views = {
                    { 0u, global_matrix_uav.Get(), 0xffffffffu },
                    { 1u, global_derivative_uav.Get(), 0xffffffffu },
                    { 2u, segment_status_uav.Get(), 0xffffffffu }
                };
                hierarchy_pass.constant_buffers = {
                    { 0u, parameter_buffer.Get() }
                };
                hierarchy_pass.group_count_x =
                    std::max(1u, (node_count + 63u) / 64u);
                if (!graph.add_pass(hierarchy_pass, diagnostic))
                {
                    return false;
                }
                out_segment_parameter_buffers.push_back(
                    std::move(parameter_buffer));
            }

            uint32_t skin_record_index = asset::invalid_index;
            for (uint32_t skin_index = 0u;
                skin_index < packet_data.skin_records.size();
                ++skin_index)
            {
                const physics::animation_skin_gpu_record& skin_record =
                    packet_data.skin_records[skin_index];
                if (skin_record.skin_index != gpu_mesh.skin_index ||
                    skin_record.mesh_node_channel_offset >= node_count)
                {
                    continue;
                }
                const uint32_t track_channel_start =
                    packet_data.track_records[
                        packet_span.first_track_record].first_channel_record;
                const uint32_t mesh_channel_record =
                    track_channel_start +
                    skin_record.mesh_node_channel_offset;
                if (mesh_channel_record <
                        packet_data.channel_records.size() &&
                    packet_data.channel_records[
                        mesh_channel_record].node_index ==
                        gpu_mesh.skin_node_index)
                {
                    skin_record_index = skin_index;
                    break;
                }
            }
            if (skin_record_index == asset::invalid_index ||
                packet_data.skin_records[skin_record_index].joint_count !=
                    gpu_mesh.skin_joint_count)
            {
                diagnostic =
                    "Immutable GPU Animation Packet does not contain the exact registered Mesh Node / Skin Joint mapping.";
                return false;
            }

            const uint32_t skin_joint_count =
                gpu_mesh.skin_joint_count;
            std::vector<physics::gpu_matrix4x4> initial_skin_matrices(
                skin_joint_count);
            Microsoft::WRL::ComPtr<ID3D11Buffer> skin_matrix_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> skin_matrix_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> skin_matrix_uav;
            Microsoft::WRL::ComPtr<ID3D11Buffer> skin_derivative_buffer;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> skin_derivative_srv;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> skin_derivative_uav;
            if (!create_rw_buffer(
                    skin_joint_count,
                    static_cast<uint32_t>(sizeof(physics::gpu_matrix4x4)),
                    initial_skin_matrices.data(),
                    skin_matrix_buffer, skin_matrix_srv, skin_matrix_uav) ||
                !create_rw_buffer(
                    skin_joint_count,
                    static_cast<uint32_t>(sizeof(physics::gpu_matrix4x4)),
                    initial_skin_matrices.data(),
                    skin_derivative_buffer,
                    skin_derivative_srv,
                    skin_derivative_uav))
            {
                diagnostic =
                    "Unable to allocate GPU Animation Skin Matrix / Derivative outputs.";
                return false;
            }

            skin_matrix_evaluation_parameters skin_parameters;
            skin_parameters.skin_record_index = skin_record_index;
            skin_parameters.skin_record_count =
                static_cast<uint32_t>(
                    packet_data.skin_records.size());
            skin_parameters.track_channel_count = node_count;
            skin_parameters.skin_joint_record_count =
                static_cast<uint32_t>(
                    packet_data.skin_joint_records.size());
            skin_parameters.inverse_bind_matrix_count =
                static_cast<uint32_t>(
                    packet_data.inverse_bind_matrices.size());
            skin_parameters.expected_mesh_node_index =
                gpu_mesh.skin_node_index;
            transform segment_root_transform;
            asset::mat4 segment_root_derivative;
            if (!evaluate_root_transform_samples(
                    model_snapshot.root_samples,
                    model_snapshot.root_transform,
                    world_time,
                    segment_root_transform,
                    segment_root_derivative,
                    diagnostic))
            {
                diagnostic =
                    "GPU Animation Root Transform evaluation failed: " +
                    diagnostic;
                return false;
            }
            const asset::mat4 root_matrix =
                make_root_transform_matrix(segment_root_transform);
            skin_parameters.root_transform =
                to_gpu_matrix(root_matrix);
            skin_parameters.root_transform_derivative =
                to_gpu_matrix(segment_root_derivative);
            Microsoft::WRL::ComPtr<ID3D11Buffer> skin_parameter_buffer;
            if (!create_constant_buffer(
                    device.Get(),
                    static_cast<uint32_t>(sizeof(skin_parameters)),
                    &skin_parameters,
                    skin_parameter_buffer.GetAddressOf()))
            {
                diagnostic =
                    "Unable to allocate GPU Animation Skin Matrix Parameters.";
                return false;
            }

            simulation::compute_pass_desc skin_matrix_pass;
            skin_matrix_pass.name =
                "animation_skin_matrix_evaluate_m" +
                std::to_string(schedule.model_index) + "_mesh" +
                std::to_string(gpu_mesh.mesh_index) + "_world_time" +
                format_double_round_trip(world_time);
            skin_matrix_pass.shader =
                animation_skin_matrix_evaluate_shader.Get();
            skin_matrix_pass.shader_resources = {
                { 0u, global_matrix_srv.Get() },
                { 1u, global_derivative_srv.Get() },
                { 2u, packet_resources.skin_srv() },
                { 3u, packet_resources.skin_joint_srv() },
                { 4u, packet_resources.inverse_bind_matrix_srv() },
                { 5u, local_transform_srv }
            };
            skin_matrix_pass.unordered_access_views = {
                { 0u, skin_matrix_uav.Get(), 0xffffffffu },
                { 1u, skin_derivative_uav.Get(), 0xffffffffu },
                { 2u, segment_status_uav.Get(), 0xffffffffu }
            };
            skin_matrix_pass.constant_buffers = {
                { 0u, skin_parameter_buffer.Get() }
            };
            skin_matrix_pass.group_count_x =
                std::max(1u, (skin_joint_count + 63u) / 64u);
            if (!graph.add_pass(skin_matrix_pass, diagnostic))
            {
                return false;
            }

            gpu_soft_body_skinning_parameters skinning_parameters;
            skinning_parameters.vertex_count = gpu_mesh.vertex_count;
            skinning_parameters.joint_count = skin_joint_count;
            Microsoft::WRL::ComPtr<ID3D11Buffer> skinning_parameter_buffer;
            if (!create_constant_buffer(
                    device.Get(),
                    static_cast<uint32_t>(sizeof(skinning_parameters)),
                    &skinning_parameters,
                    skinning_parameter_buffer.GetAddressOf()))
            {
                diagnostic =
                    "Unable to allocate GPU Soft Body Skinning Parameters.";
                return false;
            }

            simulation::compute_pass_desc skinning_pass;
            skinning_pass.name =
                "soft_body_animation_skinning_m" +
                std::to_string(schedule.model_index) + "_mesh" +
                std::to_string(gpu_mesh.mesh_index) + "_world_time" +
                format_double_round_trip(world_time);
            skinning_pass.shader =
                soft_body_animation_skinning_shader.Get();
            skinning_pass.shader_resources = {
                { 0u, gpu_mesh.rest_positions_srv.Get() },
                { 1u, gpu_mesh.skin_weights_srv.Get() },
                { 2u, skin_matrix_srv.Get() },
                { 3u, skin_derivative_srv.Get() }
            };
            skinning_pass.unordered_access_views = {
                { 0u, gpu_mesh.animation_target_positions_uav.Get(), 0xffffffffu },
                { 1u, gpu_mesh.animation_target_velocities_uav.Get(), 0xffffffffu },
                { 2u, segment_status_uav.Get(), 0xffffffffu }
            };
            skinning_pass.constant_buffers = {
                { 0u, skinning_parameter_buffer.Get() }
            };
            skinning_pass.group_count_x =
                std::max(1u, (gpu_mesh.vertex_count + 63u) / 64u);
            if (!graph.add_pass(skinning_pass, diagnostic))
            {
                return false;
            }


            out_segment_parameter_buffers.push_back(
                std::move(skin_parameter_buffer));
            out_segment_parameter_buffers.push_back(
                std::move(skinning_parameter_buffer));
            out_segment_shader_resource_views.push_back(
                std::move(source_local_srv));
            if (target_local_srv)
            {
                out_segment_shader_resource_views.push_back(
                    std::move(target_local_srv));
            }
            if (blended_local_srv)
            {
                out_segment_shader_resource_views.push_back(
                    std::move(blended_local_srv));
            }
            out_segment_shader_resource_views.push_back(
                std::move(global_matrix_srv));
            out_segment_shader_resource_views.push_back(
                std::move(global_derivative_srv));
            out_segment_shader_resource_views.push_back(
                std::move(skin_matrix_srv));
            out_segment_shader_resource_views.push_back(
                std::move(skin_derivative_srv));
            out_segment_shader_resource_views.push_back(
                std::move(segment_status_srv));
            out_segment_unordered_access_views.push_back(
                std::move(source_local_uav));
            if (target_local_uav)
            {
                out_segment_unordered_access_views.push_back(
                    std::move(target_local_uav));
            }
            if (blended_local_uav)
            {
                out_segment_unordered_access_views.push_back(
                    std::move(blended_local_uav));
            }
            out_segment_unordered_access_views.push_back(
                std::move(global_matrix_uav));
            out_segment_unordered_access_views.push_back(
                std::move(global_derivative_uav));
            out_segment_unordered_access_views.push_back(
                std::move(skin_matrix_uav));
            out_segment_unordered_access_views.push_back(
                std::move(skin_derivative_uav));
            out_segment_unordered_access_views.push_back(
                std::move(segment_status_uav));
            work.animation_target_status_srv =
                out_segment_shader_resource_views.back().Get();
            work.animation_cursor_status_uav =
                out_segment_unordered_access_views.back().Get();
            return true;
        }

        bool append_soft_body_tick_passes(
            world_slot& world,
            const tick_input_snapshot& input_snapshot,
            uint32_t source_state_set,
            uint32_t destination_state_set,
            const tick_reservation& reservation,
            simulation::d3d11_compute_graph& graph,
            std::vector<Microsoft::WRL::ComPtr<ID3D11Buffer>>&
                out_segment_parameter_buffers,
            std::vector<Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>>&
                out_segment_shader_resource_views,
            std::vector<Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>>&
                out_segment_unordered_access_views,
            std::vector<animation_packet_work_state>&
                out_animation_packet_resources,
            force_distribution_global_scratch& out_force_distribution_scratch,
            std::string& diagnostic)
        {
            diagnostic.clear();
            out_segment_parameter_buffers.clear();
            out_segment_shader_resource_views.clear();
            out_segment_unordered_access_views.clear();
            out_force_distribution_scratch = {};
            if (source_state_set > 1u || destination_state_set > 1u ||
                source_state_set == destination_state_set ||
                reservation.planned_substeps == 0u ||
                reservation.planned_integration_segments == 0u ||
                reservation.integration_segments.size() !=
                    reservation.planned_integration_segments ||
                !world_has_soft_body_work(world))
            {
                diagnostic = "Soft Body Tick Passes require valid distinct State Banks and a non-empty Substep Schedule.";
                return false;
            }

            if (!finite_double(reservation.start_time) ||
                !finite_double(reservation.delta_time) ||
                reservation.delta_time <= 0.0 ||
                !finite_vec3(input_snapshot.description.gravity))
            {
                diagnostic = "Soft Body Tick time or World gravity is not representable as finite values.";
                return false;
            }

            std::vector<soft_body_mesh_work_state> mesh_work;
            uint64_t total_distribution_group_capacity = 0u;
            for (uint32_t model_index = 0u;
                model_index < world.models.size(); ++model_index)
            {
                model_slot& model = world.models[model_index];
                if (!model.occupied)
                {
                    continue;
                }

                const model_input_snapshot* model_snapshot =
                    find_model_input_snapshot(
                        input_snapshot, model_index, model.generation);
                if (model_snapshot == nullptr)
                {
                    diagnostic =
                        "Tick Input Snapshot lost an occupied Model before Soft Body dispatch.";
                    return false;
                }

                for (soft_body_gpu_mesh& gpu_mesh : model.gpu_soft_body_meshes)
                {
                    float damping_rate = 0.0f;
                    if (!update_soft_body_materials(
                            model, *model_snapshot, gpu_mesh, damping_rate, diagnostic))
                    {
                        return false;
                    }
                    if (gpu_mesh.vertex_count == 0u)
                    {
                        diagnostic =
                            "Soft Body GPU Mesh has no Physics Vertices for Segment dispatch.";
                        return false;
                    }

                    const uint32_t group_count =
                        (gpu_mesh.vertex_count - 1u) / 64u + 1u;
                    if (total_distribution_group_capacity >
                        std::numeric_limits<uint32_t>::max() - group_count)
                    {
                        diagnostic =
                            "World Soft Body target-scope reduction capacity exceeds the uint32 GPU ABI.";
                        return false;
                    }
                    total_distribution_group_capacity += group_count;

                    soft_body_mesh_work_state work;
                    work.model_index = model_index;
                    work.model = &model;
                    work.gpu_mesh = &gpu_mesh;
                    work.damping_rate = damping_rate;
                    const mesh_physics_parameters parameters =
                        model_snapshot->mesh_parameters_valid[gpu_mesh.mesh_index]
                            ? model_snapshot->mesh_parameters[gpu_mesh.mesh_index]
                            : mesh_physics_parameters{};
                    const std::size_t tracking_channel =
                        static_cast<std::size_t>(
                            asset::physics_attribute_channel::animation_tracking);
                    work.tracking_stiffness = parameters.tracking_stiffness;
                    work.tracking_damping = parameters.tracking_damping;
                    work.tracking_attribute_multiplier =
                        parameters.attribute_multipliers[tracking_channel];
                    work.source_bank = source_state_set;
                    mesh_work.push_back(work);
                }
            }

            if (mesh_work.empty())
            {
                diagnostic =
                    "Soft Body Tick Passes found no registered Soft Body Mesh resources.";
                return false;
            }

            if (!input_snapshot.forces.empty() ||
                !input_snapshot.impulses.empty())
            {
                if (total_distribution_group_capacity == 0u ||
                    total_distribution_group_capacity >
                        std::numeric_limits<uint32_t>::max())
                {
                    diagnostic =
                        "World Force / Impulse target-scope reduction capacity is invalid.";
                    return false;
                }

                const uint32_t partial_sum_capacity =
                    static_cast<uint32_t>(total_distribution_group_capacity);
                const std::vector<float> empty_partial_sums(
                    partial_sum_capacity, 0.0f);
                const std::vector<float> empty_normalized_sum = { 0.0f };
                const UINT scratch_bind_flags =
                    D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;

                out_force_distribution_scratch.partial_sum_capacity =
                    partial_sum_capacity;
                if (!create_structured_buffer_from_vector(
                        device.Get(),
                        empty_partial_sums,
                        scratch_bind_flags,
                        out_force_distribution_scratch.partial_sum_buffer.GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(),
                        out_force_distribution_scratch.partial_sum_buffer.Get(),
                        partial_sum_capacity,
                        out_force_distribution_scratch.partial_sum_srv.GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(),
                        out_force_distribution_scratch.partial_sum_buffer.Get(),
                        partial_sum_capacity,
                        out_force_distribution_scratch.partial_sum_uav.GetAddressOf()) ||
                    !create_structured_buffer_from_vector(
                        device.Get(),
                        empty_normalized_sum,
                        scratch_bind_flags,
                        out_force_distribution_scratch.normalized_sum_buffer.GetAddressOf()) ||
                    !create_structured_srv(
                        device.Get(),
                        out_force_distribution_scratch.normalized_sum_buffer.Get(),
                        1u,
                        out_force_distribution_scratch.normalized_sum_srv.GetAddressOf()) ||
                    !create_structured_uav(
                        device.Get(),
                        out_force_distribution_scratch.normalized_sum_buffer.Get(),
                        1u,
                        out_force_distribution_scratch.normalized_sum_uav.GetAddressOf()))
                {
                    diagnostic =
                        "Unable to allocate World-scoped Force / Impulse partial-sum and normalized-denominator resources.";
                    return false;
                }
            }

            struct alignas(16) cursor_advance_parameters
            {
                uint32_t span_count = 0u;
                float segment_duration_seconds = 0.0f;
                uint32_t reserved[2] = {};
            };
            static_assert(sizeof(cursor_advance_parameters) == 16u);

            for (uint32_t segment_index = 0u;
                segment_index < reservation.planned_integration_segments;
                ++segment_index)
            {
                const integration_segment& segment =
                    reservation.integration_segments[segment_index];
                const float delta_time =
                    static_cast<float>(segment.delta_time);
                if (!finite_double(segment.start_time) ||
                    !finite_double(segment.delta_time) ||
                    segment.delta_time <= 0.0 ||
                    !finite_float(delta_time) ||
                    delta_time <= 0.0f)
                {
                    diagnostic =
                        "World Integration Segment time cannot be represented as a positive finite GPU float.";
                    return false;
                }

                for (soft_body_mesh_work_state& work : mesh_work)
                {
                    soft_body_gpu_mesh& gpu_mesh = *work.gpu_mesh;
                    work.source_position_srv =
                        work.source_is_scratch
                            ? gpu_mesh.position_scratch_srv.Get()
                            : gpu_mesh.position_bank_srvs[work.source_bank].Get();
                    work.source_velocity_srv =
                        work.source_is_scratch
                            ? gpu_mesh.velocity_scratch_srv.Get()
                            : gpu_mesh.velocity_bank_srvs[work.source_bank].Get();

                    work.destination_is_scratch = (segment_index & 1u) == 0u;
                    work.destination_position_uav =
                        work.destination_is_scratch
                            ? gpu_mesh.position_scratch_uav.Get()
                            : gpu_mesh.position_bank_uavs[destination_state_set].Get();
                    work.destination_velocity_uav =
                        work.destination_is_scratch
                            ? gpu_mesh.velocity_scratch_uav.Get()
                            : gpu_mesh.velocity_bank_uavs[destination_state_set].Get();

                    if (work.source_position_srv == nullptr ||
                        work.source_velocity_srv == nullptr ||
                        work.destination_position_uav == nullptr ||
                        work.destination_velocity_uav == nullptr)
                    {
                        diagnostic =
                            "Soft Body Segment State Bank / Scratch resource resolution failed.";
                        return false;
                    }

                    simulation::compute_pass_desc fem_pass;
                    fem_pass.name = "soft_body_fem_m" +
                        std::to_string(work.model_index) + "_mesh" +
                        std::to_string(gpu_mesh.mesh_index) + "_segment" +
                        std::to_string(segment_index);
                    fem_pass.shader = soft_body_fem_shader.Get();
                    fem_pass.shader_resources = {
                        { 0u, work.source_position_srv },
                        { 1u, gpu_mesh.tetrahedron_srv.Get() }
                    };
                    fem_pass.unordered_access_views = {
                        { 0u, gpu_mesh.tetrahedron_output_uav.Get(), 0xffffffffu },
                        { 1u, world.commit_control_uav.Get(), 0xffffffffu }
                    };
                    fem_pass.constant_buffers = {
                        { 0u, gpu_mesh.fem_parameters.Get() }
                    };
                    fem_pass.group_count_x = std::max(
                        1u, (gpu_mesh.tetrahedron_count + 63u) / 64u);
                    if (!graph.add_pass(fem_pass, diagnostic))
                    {
                        return false;
                    }

                    simulation::compute_pass_desc gather_pass;
                    gather_pass.name = "soft_body_force_gather_m" +
                        std::to_string(work.model_index) + "_mesh" +
                        std::to_string(gpu_mesh.mesh_index) + "_segment" +
                        std::to_string(segment_index);
                    gather_pass.shader = soft_body_force_gather_shader.Get();
                    gather_pass.shader_resources = {
                        { 0u, gpu_mesh.tetrahedron_output_srv.Get() },
                        { 1u, gpu_mesh.tetrahedron_srv.Get() },
                        { 2u, gpu_mesh.vertex_tetrahedron_offsets_srv.Get() },
                        { 3u, gpu_mesh.vertex_tetrahedron_ids_srv.Get() }
                    };
                    gather_pass.unordered_access_views = {
                        { 0u, gpu_mesh.vertex_force_uav.Get(), 0xffffffffu },
                        { 1u, world.commit_control_uav.Get(), 0xffffffffu }
                    };
                    gather_pass.constant_buffers = {
                        { 0u, gpu_mesh.gather_parameters.Get() }
                    };
                    gather_pass.group_count_x = std::max(
                        1u, (gpu_mesh.vertex_count + 63u) / 64u);
                    if (!graph.add_pass(gather_pass, diagnostic))
                    {
                        return false;
                    }

                    work.integrated_force_srv = gpu_mesh.vertex_force_srv.Get();
                    work.integrated_impulse_srv = gpu_mesh.vertex_force_srv.Get();
                    work.force_event_count = 0u;
                    work.impulse_event_count = 0u;
                }

                std::vector<uint32_t> active_force_indices =
                    segment.active_force_indices;
                for (uint32_t event_index : active_force_indices)
                {
                    if (event_index >= input_snapshot.forces.size())
                    {
                        diagnostic =
                            "Integration Segment contains an invalid Force Event index.";
                        return false;
                    }
                }
                std::sort(
                    active_force_indices.begin(),
                    active_force_indices.end(),
                    [&input_snapshot](
                        uint32_t left_index, uint32_t right_index)
                    {
                        return input_snapshot.forces[left_index].event_id <
                            input_snapshot.forces[right_index].event_id;
                    });

                for (uint32_t event_index : active_force_indices)
                {
                    const force_desc& force =
                        input_snapshot.forces[event_index];
                    gpu_soft_body_force_distribution_parameters event_parameters;
                    event_parameters.action_position[0] = force.position.x;
                    event_parameters.action_position[1] = force.position.y;
                    event_parameters.action_position[2] = force.position.z;
                    event_parameters.radius = force.radius;
                    event_parameters.direction[0] = force.direction.x;
                    event_parameters.direction[1] = force.direction.y;
                    event_parameters.direction[2] = force.direction.z;
                    event_parameters.magnitude = force.magnitude;
                    if (!append_soft_body_force_distribution_event(
                            world,
                            mesh_work,
                            force.target,
                            event_parameters,
                            segment_index,
                            force.event_id,
                            false,
                            out_force_distribution_scratch,
                            graph,
                            out_segment_parameter_buffers,
                            diagnostic))
                    {
                        return false;
                    }
                }

                std::vector<uint32_t> impulse_indices =
                    segment.impulse_indices;
                for (uint32_t event_index : impulse_indices)
                {
                    if (event_index >= input_snapshot.impulses.size())
                    {
                        diagnostic =
                            "Integration Segment contains an invalid Impulse Event index.";
                        return false;
                    }
                }
                std::sort(
                    impulse_indices.begin(),
                    impulse_indices.end(),
                    [&input_snapshot](
                        uint32_t left_index, uint32_t right_index)
                    {
                        return input_snapshot.impulses[left_index].event_id <
                            input_snapshot.impulses[right_index].event_id;
                    });

                for (uint32_t event_index : impulse_indices)
                {
                    const impulse_desc& impulse =
                        input_snapshot.impulses[event_index];
                    gpu_soft_body_force_distribution_parameters event_parameters;
                    event_parameters.action_position[0] = impulse.position.x;
                    event_parameters.action_position[1] = impulse.position.y;
                    event_parameters.action_position[2] = impulse.position.z;
                    event_parameters.radius = impulse.radius;
                    event_parameters.direction[0] = impulse.direction.x;
                    event_parameters.direction[1] = impulse.direction.y;
                    event_parameters.direction[2] = impulse.direction.z;
                    event_parameters.magnitude = impulse.impulse_magnitude;
                    if (!append_soft_body_force_distribution_event(
                            world,
                            mesh_work,
                            impulse.target,
                            event_parameters,
                            segment_index,
                            impulse.event_id,
                            true,
                            out_force_distribution_scratch,
                            graph,
                            out_segment_parameter_buffers,
                            diagnostic))
                    {
                        return false;
                    }
                }

                for (soft_body_mesh_work_state& work : mesh_work)
                {
                    work.has_animation_target = false;
                    work.animation_target_status_srv = nullptr;
                    work.animation_cursor_status_uav = nullptr;

                    const auto animation_schedule_iterator = std::find_if(
                        reservation.animation_schedules.begin(),
                        reservation.animation_schedules.end(),
                        [&work](const model_animation_tick_schedule& value)
                        {
                            return value.model_index == work.model_index &&
                                value.model_generation == work.model->generation;
                        });
                    if (animation_schedule_iterator !=
                        reservation.animation_schedules.end())
                    {
                        const model_input_snapshot* model_snapshot =
                            find_model_input_snapshot(
                                input_snapshot,
                                work.model_index,
                                work.model->generation);
                        if (model_snapshot == nullptr ||
                            !append_soft_body_animation_skinning_pass(
                                *work.model,
                                *model_snapshot,
                                *animation_schedule_iterator,
                                work,
                                segment.start_time,
                                delta_time,
                                input_snapshot.version,
                                out_animation_packet_resources,
                                graph,
                                out_segment_parameter_buffers,
                                out_segment_shader_resource_views,
                                out_segment_unordered_access_views,
                                diagnostic))
                        {
                            if (diagnostic.empty())
                            {
                                diagnostic =
                                    "Animated Soft Body Mesh lost its immutable Model Snapshot before Skinning.";
                            }
                            return false;
                        }
                        work.has_animation_target = true;
                    }

                }

                // Every Skinned Mesh in one Model reads the same immutable
                // Packet Cursor for this Segment. Advance that Cursor once,
                // after all Mesh consumers have evaluated their Targets and
                // before any Mesh integrates the Segment.
                for (const model_animation_tick_schedule& schedule :
                    reservation.animation_schedules)
                {
                    const auto first_mesh_iterator = std::find_if(
                        mesh_work.begin(),
                        mesh_work.end(),
                        [&schedule](const soft_body_mesh_work_state& value)
                        {
                            return value.model_index == schedule.model_index &&
                                value.model != nullptr &&
                                value.model->generation == schedule.model_generation &&
                                value.has_animation_target;
                        });
                    if (first_mesh_iterator == mesh_work.end())
                    {
                        continue;
                    }
                    if (first_mesh_iterator->animation_cursor_status_uav == nullptr)
                    {
                        diagnostic =
                            "Model Animation Span Cursor has no Segment Status UAV for failure propagation.";
                        return false;
                    }

                    const auto packet_cache_iterator = std::find_if(
                        out_animation_packet_resources.begin(),
                        out_animation_packet_resources.end(),
                        [&schedule](const animation_packet_work_state& value)
                        {
                            return value.model_index == schedule.model_index &&
                                value.model_generation == schedule.model_generation;
                        });
                    if (packet_cache_iterator ==
                        out_animation_packet_resources.end() ||
                        packet_cache_iterator->resources.cursor_uav() == nullptr ||
                        packet_cache_iterator->resources.span_srv() == nullptr)
                    {
                        diagnostic =
                            "Model Animation Span Cursor lost its shared immutable Packet resources.";
                        return false;
                    }

                    animation_packet_work_state& packet_cache =
                        *packet_cache_iterator;
                    cursor_advance_parameters cursor_parameters;
                    cursor_parameters.span_count =
                        packet_cache.resources.counts().spans;
                    cursor_parameters.segment_duration_seconds = delta_time;
                    if (cursor_parameters.span_count == 0u ||
                        !std::isfinite(cursor_parameters.segment_duration_seconds) ||
                        cursor_parameters.segment_duration_seconds <= 0.0f)
                    {
                        diagnostic =
                            "GPU Model Animation Cursor Advance requires a valid immutable Span table and finite positive Integration Segment duration.";
                        return false;
                    }

                    Microsoft::WRL::ComPtr<ID3D11Buffer> cursor_parameter_buffer;
                    if (!create_constant_buffer(
                            device.Get(),
                            static_cast<uint32_t>(sizeof(cursor_parameters)),
                            &cursor_parameters,
                            cursor_parameter_buffer.GetAddressOf()))
                    {
                        diagnostic =
                            "Unable to allocate shared per-Model Animation Cursor Advance Parameters.";
                        return false;
                    }

                    simulation::compute_pass_desc cursor_advance_pass;
                    cursor_advance_pass.name =
                        "animation_span_cursor_advance_m" +
                        std::to_string(schedule.model_index) + "_segment" +
                        std::to_string(segment_index);
                    cursor_advance_pass.shader =
                        animation_span_cursor_advance_shader.Get();
                    cursor_advance_pass.shader_resources = {
                        { 0u, packet_cache.resources.span_srv() }
                    };
                    cursor_advance_pass.unordered_access_views = {
                        { 0u, packet_cache.resources.cursor_uav(), 0xffffffffu },
                        { 1u, first_mesh_iterator->animation_cursor_status_uav, 0xffffffffu }
                    };
                    cursor_advance_pass.constant_buffers = {
                        { 0u, cursor_parameter_buffer.Get() }
                    };
                    cursor_advance_pass.group_count_x = 1u;
                    if (!graph.add_pass(cursor_advance_pass, diagnostic))
                    {
                        return false;
                    }
                    out_segment_parameter_buffers.push_back(
                        std::move(cursor_parameter_buffer));
                }

                for (soft_body_mesh_work_state& work : mesh_work)
                {
                    soft_body_gpu_mesh& gpu_mesh = *work.gpu_mesh;
                    gpu_soft_body_integration_parameters integration_parameters;
                    integration_parameters.vertex_count = gpu_mesh.vertex_count;
                    integration_parameters.delta_time = delta_time;
                    integration_parameters.damping_rate = work.damping_rate;
                    integration_parameters.has_impulse_events =
                        work.impulse_event_count != 0u ? 1u : 0u;
                    integration_parameters.gravity[0] =
                        input_snapshot.description.gravity.x;
                    integration_parameters.gravity[1] =
                        input_snapshot.description.gravity.y;
                    integration_parameters.gravity[2] =
                        input_snapshot.description.gravity.z;
                    integration_parameters.tracking_stiffness =
                        work.tracking_stiffness;
                    integration_parameters.tracking_damping =
                        work.tracking_damping;
                    integration_parameters.tracking_attribute_multiplier =
                        work.tracking_attribute_multiplier;
                    integration_parameters.has_animation_target =
                        work.has_animation_target ? 1u : 0u;

                    Microsoft::WRL::ComPtr<ID3D11Buffer> segment_parameters;
                    if (!create_constant_buffer(
                            device.Get(),
                            static_cast<uint32_t>(sizeof(integration_parameters)),
                            &integration_parameters,
                            segment_parameters.GetAddressOf()))
                    {
                        diagnostic =
                            "Unable to allocate immutable per-Segment Integration Parameters.";
                        return false;
                    }
                    out_segment_parameter_buffers.push_back(
                        std::move(segment_parameters));

                    simulation::compute_pass_desc integrate_pass;
                    integrate_pass.name = "soft_body_integrate_m" +
                        std::to_string(work.model_index) + "_mesh" +
                        std::to_string(gpu_mesh.mesh_index) + "_segment" +
                        std::to_string(segment_index);
                    integrate_pass.shader = soft_body_integrate_shader.Get();
                    integrate_pass.shader_resources = {
                        { 0u, work.source_position_srv },
                        { 1u, work.source_velocity_srv },
                        { 2u, work.integrated_force_srv },
                        { 3u, gpu_mesh.inverse_mass_srv.Get() },
                        { 4u, gpu_mesh.vertex_damping_rate_srv.Get() },
                        { 5u, work.integrated_impulse_srv }
                    };
                    if (work.has_animation_target)
                    {
                        if (work.animation_target_status_srv == nullptr ||
                            gpu_mesh.animation_target_positions_srv == nullptr ||
                            gpu_mesh.animation_target_velocities_srv == nullptr)
                        {
                            diagnostic =
                                "Animation Skinning did not publish valid Target Position / Velocity / Status SRVs for Integration.";
                            return false;
                        }
                        integrate_pass.shader_resources.push_back(
                            { 6u, gpu_mesh.animation_target_positions_srv.Get() });
                        integrate_pass.shader_resources.push_back(
                            { 7u, gpu_mesh.animation_target_velocities_srv.Get() });
                        integrate_pass.shader_resources.push_back(
                            { 8u, work.animation_target_status_srv });
                    }
                    integrate_pass.shader_resources.push_back(
                        { 9u, gpu_mesh.animation_tracking_weights_srv.Get() });
                    integrate_pass.unordered_access_views = {
                        { 0u, work.destination_position_uav, 0xffffffffu },
                        { 1u, work.destination_velocity_uav, 0xffffffffu },
                        { 2u, world.commit_control_uav.Get(), 0xffffffffu }
                    };
                    integrate_pass.constant_buffers = {
                        { 0u, out_segment_parameter_buffers.back().Get() }
                    };
                    integrate_pass.group_count_x = std::max(
                        1u, (gpu_mesh.vertex_count + 63u) / 64u);
                    if (!graph.add_pass(integrate_pass, diagnostic))
                    {
                        return false;
                    }

                    work.source_is_scratch = work.destination_is_scratch;
                    if (!work.destination_is_scratch)
                    {
                        work.source_bank = destination_state_set;
                    }
                }
            }

            for (soft_body_mesh_work_state& work : mesh_work)
            {
                if (!work.source_is_scratch)
                {
                    continue;
                }

                soft_body_gpu_mesh& gpu_mesh = *work.gpu_mesh;
                simulation::compute_pass_desc copy_pass;
                copy_pass.name = "soft_body_publish_working_state_m" +
                    std::to_string(work.model_index) + "_mesh" +
                    std::to_string(gpu_mesh.mesh_index);
                copy_pass.shader = soft_body_copy_state_shader.Get();
                copy_pass.shader_resources = {
                    { 0u, gpu_mesh.position_scratch_srv.Get() },
                    { 1u, gpu_mesh.velocity_scratch_srv.Get() }
                };
                copy_pass.unordered_access_views = {
                    { 0u, gpu_mesh.position_bank_uavs[destination_state_set].Get(), 0xffffffffu },
                    { 1u, gpu_mesh.velocity_bank_uavs[destination_state_set].Get(), 0xffffffffu },
                    { 2u, world.commit_control_uav.Get(), 0xffffffffu }
                };
                copy_pass.constant_buffers = {
                    { 0u, gpu_mesh.integration_parameters.Get() }
                };
                copy_pass.group_count_x = std::max(
                    1u, (gpu_mesh.vertex_count + 63u) / 64u);
                if (!graph.add_pass(copy_pass, diagnostic))
                {
                    return false;
                }
            }
            return true;
        }

        bool append_soft_body_render_candidate_passes(
            world_slot& world,
            const tick_input_snapshot& input_snapshot,
            uint32_t destination_state_set,
            uint32_t render_slot_id,
            simulation::d3d11_compute_graph& graph,
            std::string& diagnostic)
        {
            diagnostic.clear();
            if (!world_has_soft_body_work(world) ||
                destination_state_set > 1u ||
                render_slot_id >= world.render_candidate_slots.size())
            {
                diagnostic =
                    "Render Candidate dispatch requires a Soft Body World, valid destination State Bank and admitted Ring Slot.";
                return false;
            }

            for (uint32_t model_index = 0u;
                model_index < world.models.size();
                ++model_index)
            {
                model_slot& model = world.models[model_index];
                if (!model.occupied)
                {
                    continue;
                }

                const model_input_snapshot* model_snapshot =
                    find_model_input_snapshot(
                        input_snapshot, model_index, model.generation);
                if (model_snapshot == nullptr)
                {
                    diagnostic =
                        "Tick Input Snapshot lost an occupied Model before Render Candidate generation.";
                    return false;
                }

                const transform& root = model_snapshot->root_transform;
                gpu_soft_body_render_parameters parameters;
                parameters.render_vertex_count = 0u;
                parameters.render_index_count = 0u;
                parameters.root_rotation[0] = root.rotation.x;
                parameters.root_rotation[1] = root.rotation.y;
                parameters.root_rotation[2] = root.rotation.z;
                parameters.root_rotation[3] = root.rotation.w;
                parameters.inverse_root_rotation[0] = -root.rotation.x;
                parameters.inverse_root_rotation[1] = -root.rotation.y;
                parameters.inverse_root_rotation[2] = -root.rotation.z;
                parameters.inverse_root_rotation[3] = root.rotation.w;

                const asset::vec3 negative_translation = {
                    -root.translation.x,
                    -root.translation.y,
                    -root.translation.z
                };
                const asset::vec3 inverse_quaternion = {
                    parameters.inverse_root_rotation[0],
                    parameters.inverse_root_rotation[1],
                    parameters.inverse_root_rotation[2]
                };
                const asset::vec3 first_cross = {
                    inverse_quaternion.y * negative_translation.z -
                        inverse_quaternion.z * negative_translation.y,
                    inverse_quaternion.z * negative_translation.x -
                        inverse_quaternion.x * negative_translation.z,
                    inverse_quaternion.x * negative_translation.y -
                        inverse_quaternion.y * negative_translation.x
                };
                const asset::vec3 second_cross = {
                    inverse_quaternion.y * first_cross.z -
                        inverse_quaternion.z * first_cross.y,
                    inverse_quaternion.z * first_cross.x -
                        inverse_quaternion.x * first_cross.z,
                    inverse_quaternion.x * first_cross.y -
                        inverse_quaternion.y * first_cross.x
                };
                parameters.inverse_root_translation[0] =
                    negative_translation.x + 2.0f *
                        (root.rotation.w * first_cross.x + second_cross.x);
                parameters.inverse_root_translation[1] =
                    negative_translation.y + 2.0f *
                        (root.rotation.w * first_cross.y + second_cross.y);
                parameters.inverse_root_translation[2] =
                    negative_translation.z + 2.0f *
                        (root.rotation.w * first_cross.z + second_cross.z);
                parameters.inverse_root_translation[3] = 0.0f;

                for (soft_body_gpu_mesh& gpu_mesh : model.gpu_soft_body_meshes)
                {
                    if (render_slot_id >= gpu_mesh.render_candidate_slots.size())
                    {
                        diagnostic =
                            "Soft Body Mesh Render Candidate Ring is smaller than the World Ring.";
                        return false;
                    }

                    render_candidate_mesh_slot& candidate =
                        gpu_mesh.render_candidate_slots[render_slot_id];
                    parameters.render_vertex_count = gpu_mesh.render_vertex_count;
                    parameters.render_index_count = gpu_mesh.render_index_count;
                    immediate_context->UpdateSubresource(
                        gpu_mesh.render_parameters.Get(),
                        0u,
                        nullptr,
                        &parameters,
                        0u,
                        0u);

                    simulation::compute_pass_desc feedback_pass;
                    feedback_pass.name = "soft_body_render_candidate_m" +
                        std::to_string(model_index) + "_mesh" +
                        std::to_string(gpu_mesh.mesh_index);
                    feedback_pass.shader = soft_body_render_feedback_shader.Get();
                    feedback_pass.shader_resources = {
                        { 0u, gpu_mesh.position_bank_srvs[destination_state_set].Get() },
                        { 1u, gpu_mesh.render_binding_srv.Get() },
                        { 2u, gpu_mesh.render_vertex_srv.Get() },
                        { 3u, gpu_mesh.render_index_srv.Get() },
                        { 4u, gpu_mesh.render_vertex_triangle_offsets_srv.Get() },
                        { 5u, gpu_mesh.render_vertex_triangle_ids_srv.Get() }
                    };
                    feedback_pass.unordered_access_views = {
                        { 0u, candidate.position_uav.Get(), 0xffffffffu },
                        { 1u, candidate.normal_uav.Get(), 0xffffffffu },
                        { 2u, candidate.tangent_uav.Get(), 0xffffffffu },
                        { 3u, world.commit_control_uav.Get(), 0xffffffffu }
                    };
                    feedback_pass.constant_buffers = {
                        { 0u, gpu_mesh.render_parameters.Get() }
                    };
                    feedback_pass.group_count_x = std::max(
                        1u, (gpu_mesh.render_vertex_count + 63u) / 64u);
                    if (!graph.add_pass(feedback_pass, diagnostic))
                    {
                        return false;
                    }

                    simulation::compute_pass_desc validate_pass;
                    validate_pass.name = "soft_body_render_candidate_validate_m" +
                        std::to_string(model_index) + "_mesh" +
                        std::to_string(gpu_mesh.mesh_index);
                    validate_pass.shader = soft_body_render_validate_shader.Get();
                    validate_pass.shader_resources = {
                        { 2u, gpu_mesh.render_vertex_srv.Get() },
                        { 6u, candidate.position_srv.Get() },
                        { 7u, candidate.normal_srv.Get() },
                        { 8u, candidate.tangent_srv.Get() }
                    };
                    validate_pass.unordered_access_views = {
                        { 3u, world.commit_control_uav.Get(), 0xffffffffu }
                    };
                    validate_pass.constant_buffers = {
                        { 0u, gpu_mesh.render_parameters.Get() }
                    };
                    validate_pass.group_count_x = std::max(
                        1u, (gpu_mesh.render_vertex_count + 63u) / 64u);
                    if (!graph.add_pass(validate_pass, diagnostic))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        bool build_model_animation_tick_schedules(
            const world_slot& world,
            const tick_input_snapshot& input_snapshot,
            const tick_reservation& reservation,
            std::vector<model_animation_tick_schedule>& out_schedules,
            std::string& diagnostic) const
        {
            out_schedules.clear();
            diagnostic.clear();
            if (reservation.input_snapshot == nullptr ||
                input_snapshot.version != reservation.input_snapshot->version ||
                !finite_double(reservation.start_time) ||
                !finite_double(reservation.delta_time) ||
                reservation.delta_time <= 0.0 ||
                !finite_double(reservation.start_time + reservation.delta_time))
            {
                diagnostic =
                    "Animation Scheduler requires the immutable Tick Snapshot and a finite positive Reservation interval.";
                return false;
            }

            const auto resolve_clip_start =
                [&reservation](
                    const animation_track_desc& track,
                    const asset::animation_clip& clip,
                    double world_time_origin,
                    double& out_clip_time,
                    std::string& out_error) -> bool
            {
                if (!finite_double(world_time_origin) ||
                    !finite_double(track.time) ||
                    !finite_double(track.speed))
                {
                    out_error =
                        "Animation Track Origin, Clip Time or Playback Speed is non-finite.";
                    return false;
                }
                const double elapsed_world_time =
                    reservation.start_time - world_time_origin;
                const double clip_offset =
                    elapsed_world_time * track.speed;
                const double unwrapped_clip_time = track.time + clip_offset;
                const double clip_duration =
                    static_cast<double>(clip.duration);
                if (!finite_double(elapsed_world_time) ||
                    !finite_double(clip_offset) ||
                    !finite_double(unwrapped_clip_time) ||
                    !finite_double(clip_duration) ||
                    clip_duration < 0.0)
                {
                    out_error =
                        "Animation Track World-to-Clip time mapping is not representable.";
                    return false;
                }

                if (track.loop)
                {
                    if (clip_duration <= 0.0)
                    {
                        if (track.speed != 0.0)
                        {
                            out_error =
                                "A non-zero-speed looping Animation Track requires a positive Clip duration.";
                            return false;
                        }
                        out_clip_time = 0.0;
                    }
                    else
                    {
                        out_clip_time = std::fmod(
                            unwrapped_clip_time, clip_duration);
                        if (out_clip_time < 0.0)
                        {
                            out_clip_time += clip_duration;
                        }
                        if (out_clip_time == clip_duration)
                        {
                            out_clip_time = 0.0;
                        }
                    }
                }
                else
                {
                    out_clip_time = std::max(
                        0.0, std::min(clip_duration, unwrapped_clip_time));
                }

                if (!finite_double(out_clip_time))
                {
                    out_error =
                        "Animation Track normalized Clip Time is not finite.";
                    return false;
                }
                return true;
            };

            try
            {
                out_schedules.reserve(input_snapshot.models.size());
                for (const model_input_snapshot& model_snapshot :
                    input_snapshot.models)
                {
                    if (!model_snapshot.has_animation_playback &&
                        !model_snapshot.has_animation_blend)
                    {
                        continue;
                    }
                    if (model_snapshot.has_animation_playback &&
                        model_snapshot.has_animation_blend)
                    {
                        diagnostic =
                            "One Model cannot submit Playback and two-Clip Blend as simultaneous independent Animation Inputs.";
                        return false;
                    }
                    if (model_snapshot.model_index >= world.models.size())
                    {
                        diagnostic =
                            "Animation Input Snapshot references a Model index outside the current World.";
                        return false;
                    }
                    const model_slot& model =
                        world.models[model_snapshot.model_index];
                    if (!model.occupied ||
                        model.generation != model_snapshot.generation)
                    {
                        diagnostic =
                            "Animation Input Snapshot references a stale Model generation.";
                        return false;
                    }

                    model_animation_tick_schedule schedule;
                    schedule.model_index = model_snapshot.model_index;
                    schedule.model_generation = model_snapshot.generation;
                    const uint32_t maximum_span_count =
                        std::max(1u, desc.maximum_integration_segments_per_tick_hard_limit);

                    if (model_snapshot.has_animation_playback)
                    {
                        const animation_track_desc& track =
                            model_snapshot.animation_playback.track;
                        if (track.clip_index >= model.source_model.animations.size())
                        {
                            diagnostic =
                                "Animation Playback Clip Index is outside the immutable Model Asset.";
                            return false;
                        }
                        const asset::animation_clip& clip =
                            model.source_model.animations[track.clip_index];
                        double clip_time_start = 0.0;
                        std::string time_diagnostic;
                        if (!resolve_clip_start(
                                track, clip,
                                model_snapshot.animation_world_time_origin,
                                clip_time_start, time_diagnostic))
                        {
                            diagnostic =
                                "Animation Playback time mapping failed: " +
                                time_diagnostic;
                            return false;
                        }
                        const physics::animation_clip_span_schedule_result playback =
                            physics::schedule_animation_clip_playback_spans(
                                clip,
                                static_cast<uint32_t>(
                                    model.source_model.nodes.size()),
                                clip_time_start,
                                track.speed,
                                track.loop,
                                reservation.start_time,
                                reservation.delta_time,
                                maximum_span_count);
                        if (!playback.success)
                        {
                            diagnostic =
                                "Animation Playback Span scheduling failed: " +
                                playback.message;
                            return false;
                        }
                        schedule.playback_spans = playback.values;
                    }
                    else
                    {
                        const animation_blend_desc& blend =
                            model_snapshot.animation_blend;
                        if (blend.source_track.clip_index >=
                                model.source_model.animations.size() ||
                            blend.target_track.clip_index >=
                                model.source_model.animations.size())
                        {
                            diagnostic =
                                "Animation Blend Source / Target Clip Index is outside the immutable Model Asset.";
                            return false;
                        }
                        const asset::animation_clip& source_clip =
                            model.source_model.animations[
                                blend.source_track.clip_index];
                        const asset::animation_clip& target_clip =
                            model.source_model.animations[
                                blend.target_track.clip_index];
                        double source_clip_time_start = 0.0;
                        double target_clip_time_start = 0.0;
                        std::string source_time_diagnostic;
                        std::string target_time_diagnostic;
                        if (!resolve_clip_start(
                                blend.source_track,
                                source_clip,
                                model_snapshot.animation_world_time_origin,
                                source_clip_time_start,
                                source_time_diagnostic) ||
                            !resolve_clip_start(
                                blend.target_track,
                                target_clip,
                                model_snapshot.animation_world_time_origin,
                                target_clip_time_start,
                                target_time_diagnostic))
                        {
                            diagnostic =
                                "Animation Blend Clip-Time mapping failed: Source=" +
                                source_time_diagnostic + " Target=" +
                                target_time_diagnostic;
                            return false;
                        }

                        physics::animation_blend_weight_curve_desc weight_curve;
                        weight_curve.blend_start_time = blend.blend_start_time;
                        weight_curve.blend_end_time = blend.blend_end_time;
                        weight_curve.weight_start = blend.weight_start;
                        weight_curve.weight_end = blend.weight_end;
                        weight_curve.tangent_start = blend.tangent_start;
                        weight_curve.tangent_end = blend.tangent_end;
                        const physics::animation_blend_clip_span_schedule_result blend_schedule =
                            physics::schedule_animation_blend_clip_spans(
                                source_clip,
                                static_cast<uint32_t>(
                                    model.source_model.nodes.size()),
                                source_clip_time_start,
                                blend.source_track.speed,
                                blend.source_track.loop,
                                target_clip,
                                static_cast<uint32_t>(
                                    model.source_model.nodes.size()),
                                target_clip_time_start,
                                blend.target_track.speed,
                                blend.target_track.loop,
                                weight_curve,
                                reservation.start_time,
                                reservation.delta_time,
                                maximum_span_count);
                        if (!blend_schedule.success)
                        {
                            diagnostic =
                                "Synchronized Animation Blend Span scheduling failed: " +
                                blend_schedule.message;
                            return false;
                        }
                        schedule.has_blend = true;
                        schedule.blend_spans = blend_schedule.values;
                    }

                    out_schedules.push_back(std::move(schedule));
                }
            }
            catch (...)
            {
                out_schedules.clear();
                diagnostic =
                    "Unable to allocate immutable per-Model Animation Span schedules.";
                return false;
            }
            return true;
        }

        bool build_world_integration_segment_schedule(
            const tick_reservation& reservation,
            const tick_input_snapshot& input_snapshot,
            uint32_t substep_count,
            std::vector<integration_segment>& out_segments,
            std::string& diagnostic) const
        {
            out_segments.clear();
            diagnostic.clear();
            if (substep_count == 0u ||
                reservation.input_snapshot == nullptr ||
                input_snapshot.version != reservation.input_snapshot->version ||
                !finite_double(reservation.start_time) ||
                !finite_double(reservation.delta_time) ||
                reservation.delta_time <= 0.0)
            {
                diagnostic =
                    "Integration Segment Schedule received an invalid Reservation or Input Snapshot.";
                return false;
            }

            const double tick_start = reservation.start_time;
            const double tick_end = tick_start + reservation.delta_time;
            if (!finite_double(tick_end) || tick_end <= tick_start)
            {
                diagnostic =
                    "Tick time interval is not representable as a positive finite World-time range.";
                return false;
            }

            const std::size_t maximum_size =
                std::numeric_limits<std::size_t>::max();
            std::size_t maximum_boundary_count =
                static_cast<std::size_t>(substep_count) + 1u;
            std::size_t root_sample_boundary_count = 0u;
            for (const model_input_snapshot& model_snapshot :
                input_snapshot.models)
            {
                if (!model_snapshot.has_animation_playback &&
                    !model_snapshot.has_animation_blend)
                {
                    continue;
                }
                if (model_snapshot.root_samples.size() >
                    maximum_size - root_sample_boundary_count)
                {
                    diagnostic =
                        "Root Transform Sample boundary count overflows size_t.";
                    return false;
                }
                root_sample_boundary_count +=
                    model_snapshot.root_samples.size();
            }
            if (root_sample_boundary_count >
                maximum_size - maximum_boundary_count)
            {
                diagnostic =
                    "Combined Root Transform / Substep boundary count overflows size_t.";
                return false;
            }
            maximum_boundary_count += root_sample_boundary_count;
            std::size_t animation_boundary_count = 0u;
            for (const model_animation_tick_schedule& schedule :
                reservation.animation_schedules)
            {
                const std::size_t span_count = schedule.has_blend
                    ? schedule.blend_spans.size()
                    : schedule.playback_spans.size();
                if (span_count >
                    (maximum_size - animation_boundary_count) / 2u)
                {
                    diagnostic =
                        "Animation Span boundary count overflows size_t.";
                    return false;
                }
                animation_boundary_count += span_count * 2u;
            }
            if (animation_boundary_count >
                maximum_size - maximum_boundary_count)
            {
                diagnostic =
                    "Combined Animation / Force / Impulse boundary count overflows size_t.";
                return false;
            }
            maximum_boundary_count += animation_boundary_count;
            if (input_snapshot.forces.size() >
                (maximum_size - maximum_boundary_count) / 2u)
            {
                diagnostic = "Force Event boundary count overflows size_t.";
                return false;
            }
            maximum_boundary_count += input_snapshot.forces.size() * 2u;
            if (input_snapshot.impulses.size() >
                maximum_size - maximum_boundary_count)
            {
                diagnostic = "Impulse Event boundary count overflows size_t.";
                return false;
            }
            maximum_boundary_count += input_snapshot.impulses.size();
            if (input_snapshot.forces.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) ||
                input_snapshot.impulses.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))
            {
                diagnostic =
                    "Force / Impulse Event count exceeds the uint32 Integration Segment ABI range.";
                return false;
            }

            std::vector<double> boundaries;
            try
            {
                boundaries.reserve(maximum_boundary_count);
                boundaries.push_back(tick_start);
                boundaries.push_back(tick_end);

                for (uint32_t substep = 1u; substep < substep_count; ++substep)
                {
                    const double normalized =
                        static_cast<double>(substep) /
                        static_cast<double>(substep_count);
                    const double boundary =
                        tick_start + reservation.delta_time * normalized;
                    if (!finite_double(boundary) ||
                        boundary <= tick_start ||
                        boundary >= tick_end)
                    {
                        diagnostic =
                            "A positive World Substep boundary collapsed in double-precision Tick time.";
                        return false;
                    }
                    boundaries.push_back(boundary);
                }

                for (const model_input_snapshot& model_snapshot :
                    input_snapshot.models)
                {
                    if (!model_snapshot.has_animation_playback &&
                        !model_snapshot.has_animation_blend)
                    {
                        continue;
                    }
                    for (std::size_t sample_index = 0u;
                        sample_index < model_snapshot.root_samples.size();
                        ++sample_index)
                    {
                        const transform_sample& sample =
                            model_snapshot.root_samples[sample_index];
                        if (!finite_double(sample.time) || sample.time < 0.0 ||
                            (sample_index > 0u &&
                                sample.time <=
                                    model_snapshot.root_samples[
                                        sample_index - 1u].time))
                        {
                            diagnostic =
                                "Immutable Root Transform Sample times must be finite and strictly increasing.";
                            return false;
                        }
                        if (sample.time > tick_start &&
                            sample.time < tick_end)
                        {
                            boundaries.push_back(sample.time);
                        }
                    }
                }

                for (const model_animation_tick_schedule& schedule :
                    reservation.animation_schedules)
                {
                    const auto append_animation_boundaries =
                        [&boundaries, &tick_start, &tick_end](
                            double world_start, double world_duration) -> bool
                    {
                        const double world_end = world_start + world_duration;
                        if (!finite_double(world_start) ||
                            !finite_double(world_duration) ||
                            world_duration <= 0.0 ||
                            !finite_double(world_end) ||
                            world_end <= world_start ||
                            world_start < tick_start ||
                            world_end > tick_end)
                        {
                            return false;
                        }
                        boundaries.push_back(world_start);
                        boundaries.push_back(world_end);
                        return true;
                    };

                    if (schedule.has_blend)
                    {
                        for (const physics::animation_blend_clip_span& span :
                            schedule.blend_spans)
                        {
                            if (!append_animation_boundaries(
                                    span.world_time_start,
                                    span.world_time_duration))
                            {
                                diagnostic =
                                    "Animation Blend Span boundary is outside its immutable Tick interval or has an invalid duration.";
                                return false;
                            }
                        }
                    }
                    else
                    {
                        for (const physics::scheduled_animation_clip_span& span :
                            schedule.playback_spans)
                        {
                            if (!append_animation_boundaries(
                                    span.world_time_start,
                                    span.world_time_duration))
                            {
                                diagnostic =
                                    "Animation Playback Span boundary is outside its immutable Tick interval or has an invalid duration.";
                                return false;
                            }
                        }
                    }
                }

                for (const force_desc& force : input_snapshot.forces)
                {
                    const double force_end_time =
                        force.start_time + force.duration;
                    if (!finite_double(force.start_time) ||
                        !finite_double(force.duration) ||
                        force.duration <= 0.0 ||
                        !finite_double(force_end_time) ||
                        force_end_time <= force.start_time)
                    {
                        diagnostic =
                            "Immutable Force Event contains an invalid or unrepresentable time interval.";
                        return false;
                    }

                    if (force.start_time > tick_start &&
                        force.start_time < tick_end)
                    {
                        boundaries.push_back(force.start_time);
                    }
                    if (force_end_time > tick_start &&
                        force_end_time < tick_end)
                    {
                        boundaries.push_back(force_end_time);
                    }
                }

                for (const impulse_desc& impulse : input_snapshot.impulses)
                {
                    if (!finite_double(impulse.event_time) ||
                        impulse.event_time < 0.0)
                    {
                        diagnostic =
                            "Immutable Impulse Event contains an invalid World time.";
                        return false;
                    }
                    if (impulse.event_time > tick_start &&
                        impulse.event_time < tick_end)
                    {
                        boundaries.push_back(impulse.event_time);
                    }
                }

                std::sort(boundaries.begin(), boundaries.end());
                boundaries.erase(
                    std::unique(boundaries.begin(), boundaries.end()),
                    boundaries.end());

                if (boundaries.size() < 2u ||
                    boundaries.size() - 1u >
                        static_cast<std::size_t>(
                            std::numeric_limits<uint32_t>::max()))
                {
                    diagnostic =
                        "Integration Segment count is empty or exceeds the uint32 Runtime ABI range.";
                    return false;
                }

                out_segments.reserve(boundaries.size() - 1u);
                for (std::size_t boundary_index = 0u;
                    boundary_index + 1u < boundaries.size();
                    ++boundary_index)
                {
                    integration_segment segment;
                    segment.start_time = boundaries[boundary_index];
                    segment.delta_time =
                        boundaries[boundary_index + 1u] -
                        boundaries[boundary_index];
                    if (!finite_double(segment.start_time) ||
                        !finite_double(segment.delta_time) ||
                        segment.delta_time <= 0.0)
                    {
                        diagnostic =
                            "Integration Segment Schedule contains a non-positive or non-finite interval.";
                        out_segments.clear();
                        return false;
                    }

                    for (uint32_t force_index = 0u;
                        force_index < input_snapshot.forces.size();
                        ++force_index)
                    {
                        const force_desc& force =
                            input_snapshot.forces[force_index];
                        const double force_end_time =
                            force.start_time + force.duration;
                        if (force.start_time <= segment.start_time &&
                            segment.start_time < force_end_time)
                        {
                            segment.active_force_indices.push_back(force_index);
                        }
                    }

                    for (uint32_t impulse_index = 0u;
                        impulse_index < input_snapshot.impulses.size();
                        ++impulse_index)
                    {
                        if (input_snapshot.impulses[impulse_index].event_time ==
                            segment.start_time)
                        {
                            segment.impulse_indices.push_back(impulse_index);
                        }
                    }

                    out_segments.push_back(std::move(segment));
                }
            }
            catch (...)
            {
                out_segments.clear();
                diagnostic =
                    "Unable to allocate the World Integration Segment Schedule.";
                return false;
            }
            return true;
        }

        void update_head_reservation_preflight(world_slot& world)
        {
            if (world.reservations.empty())
            {
                world.reserved_tick_time = 0.0;
                for (const in_flight_tick& tick : world.in_flight_ticks)
                {
                    world.reserved_tick_time += tick.reservation.delta_time;
                }
                world.planned_substeps = 0u;
                world.planned_integration_segments = 0u;
                world.cpu_preflight_block_status_observed = false;
                world.gpu_dynamic_toi_block_status_observed = false;
                if (world.in_flight_tick_count != 0u)
                {
                    world.state = tick_state::in_flight;
                    return;
                }
                world.state = tick_state::idle;
                world.last_tick_status = status_code::ok;
                world.budget_block_origin = work_budget_block_origin::none;
                world.budget_resolution = work_budget_resolution::none;
                world.work_budget_block_flags = 0u;
                world.diagnostic.clear();
                return;
            }

            tick_reservation& reservation = world.reservations.front();
            if (reservation.rejected)
            {
                world.state = tick_state::rejected;
                world.last_tick_status = status_code::step_rejected;
                return;
            }
            if (reservation.predecessor_blocked)
            {
                const bool predecessor_is_now_committed =
                    world.gpu_committed_sequence !=
                        std::numeric_limits<uint64_t>::max() &&
                    reservation.sequence ==
                        world.gpu_committed_sequence + 1u;
                if (!predecessor_is_now_committed)
                {
                    reservation.blocked = true;
                    world.state = tick_state::blocked;
                    world.last_tick_status = status_code::blocked_by_predecessor;
                    world.budget_block_origin = work_budget_block_origin::none;
                    world.budget_resolution = work_budget_resolution::retryable;
                    world.work_budget_block_flags = 0u;
                    world.cpu_preflight_block_status_observed = false;
                    world.gpu_dynamic_toi_block_status_observed = false;
                    world.diagnostic =
                        "Head Reservation remains blocked until its predecessor Commit is CPU-observed.";
                    return;
                }
                reservation.predecessor_blocked = false;
            }

            if (reservation.input_snapshot == nullptr ||
                reservation.input_snapshot->version == 0u)
            {
                reservation.blocked = false;
                reservation.rejected = true;
                world.last_tick_status = status_code::gpu_execution_failed;
                world.state = tick_state::failed;
                world.diagnostic =
                    "Head Tick Reservation does not own a valid immutable Input Snapshot.";
                return;
            }
            const tick_input_snapshot& input_snapshot =
                *reservation.input_snapshot;

            reservation.blocked = false;
            world.cpu_preflight_block_status_observed = false;
            world.gpu_dynamic_toi_block_status_observed = false;
            world.executed_integration_segments = 0u;
            world.minimum_required_integration_segments = 0u;
            double dt_limit = std::numeric_limits<double>::infinity();
            if (input_snapshot.description.world_max_substep_delta_time > 0.0)
            {
                dt_limit = std::min(
                    dt_limit,
                    input_snapshot.description.world_max_substep_delta_time);
            }
            if (input_snapshot.description.contact_max_substep_delta_time > 0.0)
            {
                dt_limit = std::min(
                    dt_limit,
                    input_snapshot.description.contact_max_substep_delta_time);
            }

            double soft_body_dt_limit = std::numeric_limits<double>::infinity();
            std::string cfl_diagnostic;
            if (!compute_soft_body_cfl_time_limit(
                    world, input_snapshot, soft_body_dt_limit, cfl_diagnostic))
            {
                reservation.blocked = false;
                reservation.rejected = false;
                reservation.planned_substeps = 0u;
                reservation.planned_integration_segments = 0u;
                world.planned_substeps = 0u;
                world.planned_integration_segments = 0u;
                world.work_budget_block_flags = 0u;
                world.budget_block_origin = work_budget_block_origin::none;
                world.budget_resolution = work_budget_resolution::none;
                world.last_tick_status = status_code::asset_invalid;
                world.state = tick_state::failed;
                world.diagnostic =
                    "Soft Body CFL Preflight could not derive a valid Stability Time Limit: " +
                    cfl_diagnostic;
                return;
            }
            if (soft_body_dt_limit > 0.0 &&
                std::isfinite(soft_body_dt_limit))
            {
                dt_limit = std::min(dt_limit, soft_body_dt_limit);
            }

            double required_substeps =
                input_snapshot.description.minimum_substeps;
            if (std::isfinite(dt_limit))
            {
                required_substeps = std::max(
                    required_substeps,
                    std::ceil(reservation.delta_time / dt_limit));
            }

            world.work_budget_block_flags = 0u;
            world.budget_block_origin = work_budget_block_origin::none;
            world.budget_resolution = work_budget_resolution::none;
            world.last_tick_status = status_code::ok;
            world.diagnostic.clear();

            if (!std::isfinite(required_substeps) ||
                required_substeps < 1.0 ||
                required_substeps >
                    static_cast<double>(std::numeric_limits<uint32_t>::max()))
            {
                reservation.planned_substeps = 0u;
                reservation.planned_integration_segments = 0u;
                reservation.blocked = false;
                reservation.rejected = false;
                world.planned_substeps = 0u;
                world.planned_integration_segments = 0u;
                world.work_budget_block_flags = 0u;
                world.budget_block_origin = work_budget_block_origin::none;
                world.budget_resolution = work_budget_resolution::none;
                world.last_tick_status = status_code::capacity_exceeded;
                world.state = tick_state::failed;
                world.diagnostic =
                    "CPU Preflight Work Count Overflow: required World Substep count is not representable as uint32.";
                return;
            }

            const uint32_t planned_substeps =
                static_cast<uint32_t>(required_substeps);
            std::vector<model_animation_tick_schedule> animation_schedules;
            std::string animation_schedule_diagnostic;
            if (!build_model_animation_tick_schedules(
                    world,
                    input_snapshot,
                    reservation,
                    animation_schedules,
                    animation_schedule_diagnostic))
            {
                reservation.planned_substeps = 0u;
                reservation.planned_integration_segments = 0u;
                reservation.animation_schedules.clear();
                reservation.integration_segments.clear();
                reservation.blocked = false;
                reservation.rejected = false;
                world.planned_substeps = 0u;
                world.planned_integration_segments = 0u;
                world.last_tick_status = status_code::asset_invalid;
                world.state = tick_state::failed;
                world.diagnostic =
                    "CPU Animation Span Preflight failed: " +
                    animation_schedule_diagnostic;
                return;
            }
            reservation.animation_schedules =
                std::move(animation_schedules);

            std::vector<integration_segment> scheduled_segments;
            std::string schedule_diagnostic;
            if (!build_world_integration_segment_schedule(
                    reservation,
                    input_snapshot,
                    planned_substeps,
                    scheduled_segments,
                    schedule_diagnostic))
            {
                reservation.planned_substeps = 0u;
                reservation.planned_integration_segments = 0u;
                reservation.integration_segments.clear();
                reservation.blocked = false;
                reservation.rejected = false;
                world.planned_substeps = 0u;
                world.planned_integration_segments = 0u;
                world.work_budget_block_flags = 0u;
                world.budget_block_origin = work_budget_block_origin::none;
                world.budget_resolution = work_budget_resolution::none;
                world.last_tick_status = status_code::capacity_exceeded;
                world.state = tick_state::failed;
                world.diagnostic =
                    "CPU Integration Segment Schedule could not be represented: " +
                    schedule_diagnostic;
                return;
            }

            const uint32_t planned_segments =
                static_cast<uint32_t>(scheduled_segments.size());
            reservation.planned_substeps = planned_substeps;
            reservation.planned_integration_segments = planned_segments;
            reservation.integration_segments = std::move(scheduled_segments);
            reservation.budget_revision = world.execution_budget_revision;
            world.planned_substeps = planned_substeps;
            world.planned_integration_segments = planned_segments;

            const bool substep_budget_exceeded =
                planned_substeps > world.execution_budget.maximum_substeps;
            const bool integration_budget_exceeded =
                planned_segments >
                    world.execution_budget.maximum_integration_segments_per_tick;
            const bool hard_limit_exceeded =
                planned_substeps > desc.maximum_world_substeps_hard_limit ||
                planned_segments >
                    desc.maximum_integration_segments_per_tick_hard_limit;

            if (substep_budget_exceeded)
            {
                reservation.blocked = true;
                world.work_budget_block_flags |= 0x01u;
            }
            if (integration_budget_exceeded)
            {
                reservation.blocked = true;
                world.work_budget_block_flags |= 0x02u;
            }
            if (hard_limit_exceeded)
            {
                reservation.blocked = true;
            }

            if (reservation.blocked)
            {
                world.cpu_preflight_block_status_observed = false;
                world.budget_block_origin = work_budget_block_origin::cpu_preflight;
                world.budget_resolution = hard_limit_exceeded
                    ? work_budget_resolution::hard_limit_reached
                    : work_budget_resolution::retryable;
                world.last_tick_status = hard_limit_exceeded
                    ? status_code::work_budget_unsatisfiable
                    : status_code::work_budget_exceeded;
                world.state = tick_state::blocked;
                world.diagnostic = hard_limit_exceeded
                    ? "Required CPU Preflight work exceeds the immutable Runtime Hard Limit."
                    : "Required CPU Preflight work exceeds the current World Execution Budget.";
                return;
            }

            reservation.blocked = false;
            world.cpu_preflight_block_status_observed = false;
            world.state = tick_state::pending;
        }

        bool reserve_next_tick(
            world_slot& world,
            bool from_single_step)
        {
            if (!world.reservations.empty())
            {
                return false;
            }
            if (from_single_step)
            {
                if (world.pending_single_step_requests == 0u)
                {
                    return false;
                }
            }
            else if (world.accumulator_seconds + time_epsilon <
                world.description.fixed_delta_time)
            {
                return false;
            }

            const double reservation_start_time =
                world.gpu_committed_simulation_time + world.reserved_tick_time;
            const double reservation_end_time =
                reservation_start_time + world.description.fixed_delta_time;
            if (world.next_tick_sequence == 0u ||
                world.next_tick_sequence == std::numeric_limits<uint64_t>::max() ||
                !finite_double(reservation_start_time) ||
                !finite_double(reservation_end_time) ||
                reservation_end_time <= reservation_start_time ||
                !finite_double(world.reserved_tick_time +
                    world.description.fixed_delta_time))
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::capacity_exceeded;
                world.diagnostic = "Tick Reservation Sequence or reserved time cannot advance without overflow.";
                return false;
            }

            if (world.next_input_snapshot_version == 0u ||
                world.next_input_snapshot_version ==
                    std::numeric_limits<uint64_t>::max())
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::capacity_exceeded;
                world.diagnostic =
                    "World Input Snapshot Version cannot advance without uint64 overflow.";
                return false;
            }

            std::shared_ptr<const tick_input_snapshot> input_snapshot;
            std::string snapshot_diagnostic;
            if (!build_tick_input_snapshot(
                    world,
                    world.next_input_snapshot_version,
                    input_snapshot,
                    snapshot_diagnostic))
            {
                world.state = tick_state::pending;
                world.last_tick_status = status_code::out_of_memory;
                world.diagnostic = snapshot_diagnostic;
                return false;
            }

            tick_reservation reservation;
            reservation.sequence = world.next_tick_sequence;
            reservation.start_time = reservation_start_time;
            reservation.delta_time = world.description.fixed_delta_time;
            reservation.input_snapshot = std::move(input_snapshot);
            reservation.from_single_step = from_single_step;
            reservation.budget_revision = world.execution_budget_revision;

            ++world.next_input_snapshot_version;
            ++world.next_tick_sequence;
            if (from_single_step)
            {
                --world.pending_single_step_requests;
            }
            else
            {
                world.accumulator_seconds = std::max(
                    0.0, world.accumulator_seconds -
                        world.description.fixed_delta_time);
            }

            world.reserved_tick_time += reservation.delta_time;
            world.reservations.push_back(reservation);
            update_head_reservation_preflight(world);
            return true;
        }

        bool initialize_world_commit_control(
            world_slot& world,
            std::string& diagnostic)
        {
            gpu_tick_commit_control initial_control;
            initial_control.state_generation_low = 1u;
            initial_control.active_state_set = 0u;
            initial_control.required_validation_mask = 1u;
            initial_control.render_slot_id = asset::invalid_index;
            initial_control.render_slot_generation = 0u;

            Microsoft::WRL::ComPtr<ID3D11Buffer> control_buffer;
            if (!create_structured_buffer(
                    device.Get(),
                    static_cast<uint32_t>(sizeof(initial_control)),
                    static_cast<uint32_t>(sizeof(initial_control)),
                    D3D11_BIND_UNORDERED_ACCESS,
                    &initial_control,
                    control_buffer.GetAddressOf()))
            {
                diagnostic = "Unable to allocate the World GPU Commit Control Buffer.";
                return false;
            }

            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> control_uav;
            if (!create_structured_uav(
                    device.Get(), control_buffer.Get(), 1u,
                    control_uav.GetAddressOf()))
            {
                diagnostic = "Unable to create the World GPU Commit Control UAV.";
                return false;
            }

            world.commit_control_buffer = std::move(control_buffer);
            world.commit_control_uav = std::move(control_uav);
            world.gpu_state_generation = 1u;
            world.active_state_set = 0u;
            world.scheduled_predecessor_sequence = 0u;
            world.scheduled_source_generation = 1u;
            world.scheduled_source_state_set = 0u;
            return true;
        }

        bool submit_head_reservation(
            world_slot& world,
            uint32_t world_index)
        {
            if (world.state == tick_state::failed ||
                world.reservations.empty() ||
                world.reservations.front().blocked ||
                world.reservations.front().rejected ||
                world.in_flight_tick_count >=
                    world.description.maximum_in_flight_ticks)
            {
                return false;
            }
            if (world.reservations.front().input_snapshot == nullptr ||
                world.reservations.front().input_snapshot->version == 0u)
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::gpu_execution_failed;
                world.diagnostic =
                    "GPU Submission requires the immutable Input Snapshot owned by its Tick Reservation.";
                return false;
            }
            const tick_input_snapshot& input_snapshot =
                *world.reservations.front().input_snapshot;
            if (world_has_unsupported_physics_work(world, input_snapshot))
            {
                // Keep the CPU Reservation pending and retryable. The Input
                // Block is diagnostic; no quality or time reduction is allowed.
                world.state = tick_state::blocked;
                world.last_tick_status = status_code::blocked_by_input;
                world.budget_block_origin = work_budget_block_origin::none;
                world.budget_resolution = work_budget_resolution::none;
                world.work_budget_block_flags = 0u;
                world.diagnostic =
                    "GPU submission is deferred because this World contains an unsupported Force / Impulse target, Root Transform Samples without an active Animation Track, Cloth or Collider input. Force / Impulse scopes now include one exact Mesh, every Soft Body Mesh in a Model, or every Soft Body Mesh in the World.";
                return false;
            }
            if (world.render_lease_active)
            {
                world.state = tick_state::pending;
                world.last_tick_status = status_code::render_binding_busy;
                world.diagnostic =
                    "GPU submission is waiting for the Host Render Lease to be released.";
                return false;
            }
            if (status_ring == nullptr ||
                status_ring->pending_count() >= status_ring->capacity())
            {
                world.state = tick_state::pending;
                world.last_tick_status = status_code::blocked_by_capacity;
                world.budget_block_origin = work_budget_block_origin::none;
                world.budget_resolution = work_budget_resolution::retryable;
                world.work_budget_block_flags = 0u;
                world.diagnostic =
                    "GPU Status Query Ring is full; the Tick Reservation remains pending.";
                return false;
            }
            if (next_dispatch_id == 0u ||
                next_dispatch_id == std::numeric_limits<uint64_t>::max())
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::capacity_exceeded;
                world.diagnostic = "Runtime Dispatch ID space is exhausted.";
                return false;
            }
            if (world.scheduled_source_generation ==
                std::numeric_limits<uint64_t>::max())
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::capacity_exceeded;
                world.diagnostic =
                    "Projected GPU State Generation cannot advance without uint64 overflow.";
                return false;
            }

            const tick_reservation reservation = world.reservations.front();
            if (reservation.input_snapshot == nullptr ||
                reservation.input_snapshot->version == 0u)
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::gpu_execution_failed;
                world.diagnostic =
                    "Submission Reservation has no valid Immutable Input Snapshot Version.";
                return false;
            }
            const uint64_t input_snapshot_version =
                reservation.input_snapshot->version;
            gpu_tick_dispatch_plan plan;
            plan.expected_predecessor_sequence_low =
                static_cast<uint32_t>(world.scheduled_predecessor_sequence);
            plan.expected_predecessor_sequence_high =
                static_cast<uint32_t>(world.scheduled_predecessor_sequence >> 32u);
            plan.expected_source_generation_low =
                static_cast<uint32_t>(world.scheduled_source_generation);
            plan.expected_source_generation_high =
                static_cast<uint32_t>(world.scheduled_source_generation >> 32u);
            plan.expected_source_state_set = world.scheduled_source_state_set;
            plan.destination_state_set = 1u - world.scheduled_source_state_set;
            plan.tick_sequence_low = static_cast<uint32_t>(reservation.sequence);
            plan.tick_sequence_high = static_cast<uint32_t>(reservation.sequence >> 32u);
            plan.dispatch_id_low = static_cast<uint32_t>(next_dispatch_id);
            plan.dispatch_id_high = static_cast<uint32_t>(next_dispatch_id >> 32u);
            plan.required_validation_mask = 1u;
            plan.flags = world_has_soft_body_work(world) ? 0u : 1u;
            plan.reserved[0] = 0xffffffffu;
            plan.reserved[1] = 0u;

            const uint64_t world_id = world_identity(world_index, world.generation);
            uint64_t reservation_token =
                (world_id * 0x9e3779b97f4a7c15ull) ^
                (reservation.sequence * 0xbf58476d1ce4e5b9ull) ^
                (input_snapshot_version * 0x94d049bb133111ebull) ^
                next_dispatch_id;
            if (reservation_token == 0u)
            {
                reservation_token = next_dispatch_id;
            }
            plan.reservation_token_low =
                static_cast<uint32_t>(reservation_token);
            plan.reservation_token_high =
                static_cast<uint32_t>(reservation_token >> 32u);

            Microsoft::WRL::ComPtr<ID3D11Buffer> plan_buffer;
            if (!create_structured_buffer(
                    device.Get(),
                    static_cast<uint32_t>(sizeof(plan)),
                    static_cast<uint32_t>(sizeof(plan)),
                    D3D11_BIND_SHADER_RESOURCE,
                    &plan,
                    plan_buffer.GetAddressOf()))
            {
                world.state = tick_state::blocked;
                world.last_tick_status = status_code::out_of_memory;
                world.budget_resolution = work_budget_resolution::retryable;
                world.diagnostic = "Unable to allocate an immutable GPU Tick Dispatch Plan Buffer.";
                return false;
            }

            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> plan_srv;
            if (!create_structured_srv(
                    device.Get(), plan_buffer.Get(), 1u,
                    plan_srv.GetAddressOf()))
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::gpu_execution_failed;
                world.diagnostic = "Unable to create the immutable GPU Tick Dispatch Plan SRV.";
                return false;
            }

            gpu_tick_status_record initial_status = {};
            Microsoft::WRL::ComPtr<ID3D11Buffer> status_buffer;
            if (!create_structured_buffer(
                    device.Get(),
                    static_cast<uint32_t>(sizeof(initial_status)),
                    static_cast<uint32_t>(sizeof(initial_status)),
                    D3D11_BIND_UNORDERED_ACCESS,
                    &initial_status,
                    status_buffer.GetAddressOf()))
            {
                world.state = tick_state::blocked;
                world.last_tick_status = status_code::out_of_memory;
                world.budget_resolution = work_budget_resolution::retryable;
                world.diagnostic = "Unable to allocate the GPU Tick Status Buffer.";
                return false;
            }

            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> status_uav;
            if (!create_structured_uav(
                    device.Get(), status_buffer.Get(), 1u,
                    status_uav.GetAddressOf()))
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::gpu_execution_failed;
                world.diagnostic = "Unable to create the GPU Tick Status UAV.";
                return false;
            }

            simulation::d3d11_compute_graph graph;
            std::vector<Microsoft::WRL::ComPtr<ID3D11Buffer>>
                segment_parameter_buffers;
            std::vector<Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>>
                segment_shader_resource_views;
            std::vector<Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>>
                segment_unordered_access_views;
            force_distribution_global_scratch force_distribution_scratch;
            std::vector<animation_packet_work_state>
                animation_packet_resources;
            std::string diagnostic;
            simulation::compute_pass_desc begin_pass;
            begin_pass.name = "tick_begin_control";
            begin_pass.shader = tick_begin_shader.Get();
            begin_pass.shader_resources = {
                { 0u, plan_srv.Get() }
            };
            begin_pass.unordered_access_views = {
                { 1u, world.commit_control_uav.Get(), 0xffffffffu },
                { 2u, status_uav.Get(), 0xffffffffu }
            };
            if (!graph.add_pass(begin_pass, diagnostic))
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::gpu_execution_failed;
                world.diagnostic = diagnostic;
                return false;
            }

            uint32_t render_slot_id = asset::invalid_index;
            uint32_t render_slot_generation = 0u;
            bool render_slot_reserved = false;

            if (world_has_soft_body_work(world))
            {
                if (!append_soft_body_tick_passes(
                        world,
                        input_snapshot,
                        plan.expected_source_state_set,
                        plan.destination_state_set,
                        reservation,
                        graph,
                        segment_parameter_buffers,
                        segment_shader_resource_views,
                        segment_unordered_access_views,
                        animation_packet_resources,
                        force_distribution_scratch,
                        diagnostic))
                {
                    world.state = tick_state::failed;
                    world.last_tick_status = status_code::gpu_execution_failed;
                    world.diagnostic =
                        "Unable to construct the gated Soft Body Tick Passes: " + diagnostic;
                    return false;
                }

                if (!reserve_render_candidate_slot(
                        world,
                        reservation.sequence,
                        render_slot_id,
                        render_slot_generation,
                        diagnostic))
                {
                    world.state = tick_state::pending;
                    world.last_tick_status = status_code::render_binding_busy;
                    world.budget_block_origin = work_budget_block_origin::none;
                    world.budget_resolution = work_budget_resolution::none;
                    world.work_budget_block_flags = 0u;
                    world.diagnostic = diagnostic;
                    return false;
                }
                render_slot_reserved =
                    render_slot_id != asset::invalid_index;
                plan.reserved[0] = render_slot_id;
                plan.reserved[1] = render_slot_generation;
                immediate_context->UpdateSubresource(
                    plan_buffer.Get(), 0u, nullptr, &plan, 0u, 0u);

                if (!append_soft_body_render_candidate_passes(
                        world,
                        input_snapshot,
                        plan.destination_state_set,
                        render_slot_id,
                        graph,
                        diagnostic))
                {
                    release_render_candidate_slot(
                        world, render_slot_id, render_slot_generation);
                    world.state = tick_state::failed;
                    world.last_tick_status = status_code::gpu_execution_failed;
                    world.diagnostic =
                        "Unable to construct the gated Render Candidate passes: " + diagnostic;
                    return false;
                }

                simulation::compute_pass_desc validate_pass;
                validate_pass.name = "tick_validate_soft_body";
                validate_pass.shader = tick_validate_soft_body_shader.Get();
                validate_pass.shader_resources = {
                    { 0u, plan_srv.Get() }
                };
                validate_pass.unordered_access_views = {
                    { 1u, world.commit_control_uav.Get(), 0xffffffffu },
                    { 2u, status_uav.Get(), 0xffffffffu }
                };
                if (!graph.add_pass(validate_pass, diagnostic))
                {
                    release_render_candidate_slot(
                        world, render_slot_id, render_slot_generation);
                    world.state = tick_state::failed;
                    world.last_tick_status = status_code::gpu_execution_failed;
                    world.diagnostic = diagnostic;
                    return false;
                }
            }
            else
            {
                simulation::compute_pass_desc validate_pass;
                validate_pass.name = "tick_validate_empty_world";
                validate_pass.shader = tick_validate_empty_world_shader.Get();
                validate_pass.shader_resources = {
                    { 0u, plan_srv.Get() }
                };
                validate_pass.unordered_access_views = {
                    { 1u, world.commit_control_uav.Get(), 0xffffffffu },
                    { 2u, status_uav.Get(), 0xffffffffu }
                };
                if (!graph.add_pass(validate_pass, diagnostic))
                {
                    world.state = tick_state::failed;
                    world.last_tick_status = status_code::gpu_execution_failed;
                    world.diagnostic = diagnostic;
                    return false;
                }
            }

            simulation::compute_pass_desc commit_pass;
            commit_pass.name = "tick_commit_control";
            commit_pass.shader = tick_commit_shader.Get();
            commit_pass.shader_resources = {
                { 0u, plan_srv.Get() }
            };
            commit_pass.unordered_access_views = {
                { 1u, world.commit_control_uav.Get(), 0xffffffffu },
                { 2u, status_uav.Get(), 0xffffffffu }
            };
            if (!graph.add_pass(commit_pass, diagnostic))
            {
                if (render_slot_reserved)
                {
                    release_render_candidate_slot(
                        world, render_slot_id, render_slot_generation);
                }
                world.state = tick_state::failed;
                world.last_tick_status = status_code::gpu_execution_failed;
                world.diagnostic = diagnostic;
                return false;
            }

            const simulation::compute_graph_result execution =
                graph.execute(immediate_context.Get());
            if (!execution.success)
            {
                if (render_slot_reserved)
                {
                    release_render_candidate_slot(
                        world, render_slot_id, render_slot_generation);
                }
                world.state = tick_state::failed;
                world.last_tick_status = status_code::gpu_execution_failed;
                world.diagnostic = execution.diagnostic;
                return false;
            }

            const simulation::d3d11_status_query_key query_key = {
                world_id,
                reservation.sequence,
                next_dispatch_id
            };
            const simulation::d3d11_status_ring_error enqueue_result =
                status_ring->enqueue(query_key, status_buffer.Get(), diagnostic);
            if (enqueue_result != simulation::d3d11_status_ring_error::none)
            {
                if (render_slot_reserved)
                {
                    release_render_candidate_slot(
                        world, render_slot_id, render_slot_generation);
                }
                world.state = tick_state::failed;
                world.last_tick_status =
                    enqueue_result == simulation::d3d11_status_ring_error::device_removed
                        ? status_code::device_removed
                        : status_code::gpu_execution_failed;
                world.diagnostic =
                    "GPU Tick was dispatched but Status Query enqueue failed: " + diagnostic;
                return false;
            }

            in_flight_tick in_flight;
            in_flight.reservation = reservation;
            in_flight.dispatch_id = next_dispatch_id;
            in_flight.reservation_token = reservation_token;
            in_flight.input_snapshot_version = input_snapshot_version;
            in_flight.segment_parameter_buffers =
                std::move(segment_parameter_buffers);
            in_flight.segment_shader_resource_views =
                std::move(segment_shader_resource_views);
            in_flight.segment_unordered_access_views =
                std::move(segment_unordered_access_views);
            in_flight.animation_packet_resources =
                std::move(animation_packet_resources);
            in_flight.force_distribution_scratch =
                std::move(force_distribution_scratch);
            in_flight.expected_predecessor_sequence =
                world.scheduled_predecessor_sequence;
            in_flight.expected_source_generation =
                world.scheduled_source_generation;
            in_flight.expected_source_state_set =
                world.scheduled_source_state_set;
            in_flight.expected_destination_state_set =
                plan.destination_state_set;
            in_flight.render_candidate_slot_id = render_slot_id;
            in_flight.render_candidate_slot_generation =
                render_slot_generation;
            in_flight.plan_buffer = std::move(plan_buffer);
            in_flight.status_buffer = std::move(status_buffer);
            world.in_flight_ticks.push_back(std::move(in_flight));
            world.reservations.pop_front();
            world.in_flight_tick_count = static_cast<uint32_t>(
                world.in_flight_ticks.size());
            world.submitted_sequence = reservation.sequence;
            world.scheduled_predecessor_sequence = reservation.sequence;
            ++world.scheduled_source_generation;
            world.scheduled_source_state_set = plan.destination_state_set;
            world.state = tick_state::in_flight;
            world.last_tick_status = status_code::ok;
            world.cpu_preflight_block_status_observed = false;
            world.gpu_dynamic_toi_block_status_observed = false;
            world.budget_block_origin = work_budget_block_origin::none;
            world.budget_resolution = work_budget_resolution::none;
            world.work_budget_block_flags = 0u;
            world.diagnostic.clear();
            ++next_dispatch_id;
            return true;
        }

        bool ensure_status_capacity(uint32_t required_capacity, std::string& diagnostic)
        {
            if (status_ring && status_ring->capacity() >= required_capacity)
            {
                return true;
            }
            if (status_ring && status_ring->pending_count() != 0u)
            {
                diagnostic = "Status Query Ring cannot grow while Query slots are in flight.";
                return false;
            }

            const uint32_t expanded_capacity =
                std::max(minimum_status_ring_capacity, next_power_of_two(required_capacity));
            if (expanded_capacity == 0u)
            {
                diagnostic = "Status Query Ring capacity exceeds the supported uint32 range.";
                return false;
            }

            std::unique_ptr<simulation::d3d11_status_ring> replacement(
                new simulation::d3d11_status_ring());
            if (!replacement->initialize(
                    device.Get(),
                    immediate_context.Get(),
                    expanded_capacity,
                    static_cast<uint32_t>(sizeof(gpu_tick_status_record)),
                    diagnostic))
            {
                return false;
            }
            status_ring = std::move(replacement);
            return true;
        }

        uint32_t total_in_flight_capacity() const
        {
            uint64_t total = 0u;
            for (const world_slot& world : worlds)
            {
                if (world.occupied)
                {
                    total += world.description.maximum_in_flight_ticks;
                }
            }
            if (total > std::numeric_limits<uint32_t>::max())
            {
                return 0u;
            }
            return static_cast<uint32_t>(total);
        }

        bool valid_target(world_slot& world, const target_filter& target) const
        {
            if (target.model.index == asset::invalid_index)
            {
                return target.mesh_index == asset::invalid_index;
            }
            model_slot* model = const_cast<implementation*>(this)->find_model(
                world, target.model);
            if (model == nullptr)
            {
                return false;
            }
            if (target.mesh_index != asset::invalid_index &&
                target.mesh_index >= model->source_model.meshes.size())
            {
                return false;
            }
            return true;
        }

        bool has_force_target(const world_slot& world, const target_filter& target) const
        {
            if (target.model.index != asset::invalid_index)
            {
                const model_slot* model = nullptr;
                if (target.model.index < world.models.size())
                {
                    const model_slot& item = world.models[target.model.index];
                    if (item.occupied && item.generation == target.model.generation)
                    {
                        model = &item;
                    }
                }
                if (model == nullptr)
                {
                    return false;
                }
                if (target.mesh_index != asset::invalid_index)
                {
                    return target.mesh_index < model->source_model.meshes.size() &&
                        model->source_model.meshes[target.mesh_index].mode != asset::physics_mode::none;
                }
                return std::any_of(
                    model->source_model.meshes.begin(),
                    model->source_model.meshes.end(),
                    [](const asset::mesh& mesh)
                    {
                        return mesh.mode != asset::physics_mode::none;
                    });
            }

            for (const model_slot& model : world.models)
            {
                if (!model.occupied)
                {
                    continue;
                }
                if (std::any_of(
                        model.source_model.meshes.begin(),
                        model.source_model.meshes.end(),
                        [](const asset::mesh& mesh)
                        {
                            return mesh.mode != asset::physics_mode::none;
                        }))
                {
                    return true;
                }
            }
            return false;
        }
    };

    physics_runtime::physics_runtime()
        : _implementation(new implementation())
    {
    }

    physics_runtime::~physics_runtime() = default;

    status physics_runtime::initialize(
        ID3D11Device* device,
        ID3D11DeviceContext* immediate_context,
        const runtime_desc& desc)
    {
        if (_implementation == nullptr)
        {
            return make_status(status_code::not_ready,
                "Runtime implementation is unavailable.");
        }
        implementation& runtime = *_implementation;
        if (runtime.initialized)
        {
            return make_status(status_code::invalid_state,
                "Runtime is already initialized.");
        }
        if (device == nullptr || immediate_context == nullptr ||
            !valid_runtime_desc(desc))
        {
            return make_status(status_code::invalid_argument,
                "Runtime initialization requires valid descriptors, a D3D11 Device and an Immediate Context.");
        }
        if (device->GetFeatureLevel() < D3D_FEATURE_LEVEL_11_0)
        {
            return make_status(status_code::unsupported_feature,
                "Skin Physics Runtime requires D3D11 Feature Level 11_0 or newer.");
        }
        if (immediate_context->GetType() != D3D11_DEVICE_CONTEXT_IMMEDIATE)
        {
            return make_status(status_code::invalid_argument,
                "Skin Physics Runtime accepts the Host-owned Immediate Context only.");
        }

        Microsoft::WRL::ComPtr<ID3D11Device> context_device;
        immediate_context->GetDevice(context_device.GetAddressOf());
        if (!same_com_identity(device, context_device.Get()))
        {
            return make_status(status_code::invalid_argument,
                "The supplied Immediate Context belongs to a different D3D11 Device.");
        }

        const HRESULT removed_reason = device->GetDeviceRemovedReason();
        if (FAILED(removed_reason))
        {
            return make_status(status_code::device_removed,
                "The supplied D3D11 Device is already removed.",
                static_cast<int32_t>(removed_reason));
        }

        std::string diagnostic;
        const std::filesystem::path shader_root =
            resolve_shader_package_root(desc.shader_package_root, diagnostic);
        if (shader_root.empty())
        {
            return make_status(status_code::invalid_argument, diagnostic);
        }
        const std::filesystem::path tick_shader_path =
            shader_root / "tick_transaction_control.cs.hlsl";
        const std::filesystem::path soft_body_animation_skinning_path =
            shader_root / "soft_body_animation_skinning.cs.hlsl";
        const std::filesystem::path animation_channel_evaluate_path =
            shader_root / "animation_channel_evaluate.cs.hlsl";
        const std::filesystem::path animation_blend_local_transform_path =
            shader_root / "animation_blend_local_transform.cs.hlsl";
        const std::filesystem::path animation_node_hierarchy_evaluate_path =
            shader_root / "animation_node_hierarchy_evaluate.cs.hlsl";
        const std::filesystem::path animation_skin_matrix_evaluate_path =
            shader_root / "animation_skin_matrix_evaluate.cs.hlsl";
        const std::filesystem::path animation_span_cursor_advance_path =
            shader_root / "animation_span_cursor_advance.cs.hlsl";
        const std::filesystem::path soft_body_fem_path =
            shader_root / "soft_body_fem.cs.hlsl";
        const std::filesystem::path soft_body_gather_path =
            shader_root / "soft_body_force_gather.cs.hlsl";
        const std::filesystem::path soft_body_integrate_path =
            shader_root / "soft_body_integrate.cs.hlsl";
        const std::filesystem::path soft_body_force_distribution_path =
            shader_root / "soft_body_force_distribution.cs.hlsl";
        const std::filesystem::path soft_body_render_feedback_path =
            shader_root / "soft_body_render_feedback.cs.hlsl";

        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_begin_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_validate_empty_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_validate_soft_body_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_commit_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_animation_skinning_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_channel_evaluate_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_blend_local_transform_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_node_hierarchy_evaluate_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_skin_matrix_evaluate_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> animation_span_cursor_advance_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_fem_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_gather_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_weight_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_reduce_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_finalize_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_apply_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_integrate_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_copy_state_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_render_feedback_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_render_validate_shader;
        if (!compile_runtime_compute_shader(
                device, tick_shader_path, "tick_begin_control",
                tick_begin_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, tick_shader_path, "tick_mark_empty_world_validated",
                tick_validate_empty_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, tick_shader_path, "tick_validate_soft_body",
                tick_validate_soft_body_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, tick_shader_path, "tick_commit_control",
                tick_commit_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_animation_skinning_path, "main",
                soft_body_animation_skinning_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, animation_channel_evaluate_path, "main",
                animation_channel_evaluate_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, animation_blend_local_transform_path, "main",
                animation_blend_local_transform_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, animation_node_hierarchy_evaluate_path, "main",
                animation_node_hierarchy_evaluate_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, animation_skin_matrix_evaluate_path, "main",
                animation_skin_matrix_evaluate_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, animation_span_cursor_advance_path, "main",
                animation_span_cursor_advance_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_fem_path, "soft_body_fem",
                soft_body_fem_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_gather_path, "soft_body_force_gather",
                soft_body_gather_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_force_distribution_path,
                "soft_body_force_distribution_weight",
                soft_body_force_weight_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_force_distribution_path,
                "soft_body_force_distribution_reduce",
                soft_body_force_reduce_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_force_distribution_path,
                "soft_body_force_distribution_finalize",
                soft_body_force_finalize_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_force_distribution_path,
                "soft_body_force_distribution_apply",
                soft_body_force_apply_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_integrate_path, "soft_body_integrate",
                soft_body_integrate_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_integrate_path, "soft_body_copy_state",
                soft_body_copy_state_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_render_feedback_path, "prepare_render_candidate",
                soft_body_render_feedback_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_render_feedback_path, "validate_render_candidate",
                soft_body_render_validate_shader.GetAddressOf(), diagnostic))
        {
            return make_status(status_code::gpu_execution_failed,
                "Runtime Tick / Soft Body shader initialization failed: " + diagnostic);
        }

        runtime.desc = desc;
        runtime.device = device;
        runtime.immediate_context = immediate_context;
        runtime.tick_begin_shader = std::move(tick_begin_shader);
        runtime.tick_validate_empty_world_shader = std::move(tick_validate_empty_shader);
        runtime.tick_validate_soft_body_shader = std::move(tick_validate_soft_body_shader);
        runtime.tick_commit_shader = std::move(tick_commit_shader);
        runtime.soft_body_animation_skinning_shader =
            std::move(soft_body_animation_skinning_shader);
        runtime.animation_channel_evaluate_shader =
            std::move(animation_channel_evaluate_shader);
        runtime.animation_blend_local_transform_shader =
            std::move(animation_blend_local_transform_shader);
        runtime.animation_node_hierarchy_evaluate_shader =
            std::move(animation_node_hierarchy_evaluate_shader);
        runtime.animation_skin_matrix_evaluate_shader =
            std::move(animation_skin_matrix_evaluate_shader);
        runtime.animation_span_cursor_advance_shader =
            std::move(animation_span_cursor_advance_shader);
        runtime.soft_body_fem_shader = std::move(soft_body_fem_shader);
        runtime.soft_body_force_gather_shader = std::move(soft_body_gather_shader);
        runtime.soft_body_force_weight_shader = std::move(soft_body_force_weight_shader);
        runtime.soft_body_force_reduce_shader = std::move(soft_body_force_reduce_shader);
        runtime.soft_body_force_finalize_shader = std::move(soft_body_force_finalize_shader);
        runtime.soft_body_force_apply_shader = std::move(soft_body_force_apply_shader);
        runtime.soft_body_integrate_shader = std::move(soft_body_integrate_shader);
        runtime.soft_body_copy_state_shader = std::move(soft_body_copy_state_shader);
        runtime.soft_body_render_feedback_shader =
            std::move(soft_body_render_feedback_shader);
        runtime.soft_body_render_validate_shader =
            std::move(soft_body_render_validate_shader);
        runtime.submission_thread = std::this_thread::get_id();

        if (!runtime.ensure_status_capacity(desc.query_ring_capacity, diagnostic))
        {
            runtime.immediate_context.Reset();
            runtime.device.Reset();
            return make_status(status_code::out_of_memory, diagnostic);
        }

        runtime.initialized = true;
        runtime.accepting_submissions = true;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::create_world(
        const world_desc& desc,
        world_handle* out_world)
    {
        if (_implementation == nullptr || !_implementation->initialized)
        {
            return make_status(status_code::not_ready,
                "Runtime must be initialized before creating a World.");
        }
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        if (out_world == nullptr || !valid_world_desc(desc, runtime.desc))
        {
            return make_status(status_code::invalid_argument,
                "World descriptor is invalid or the output handle is null.");
        }
        if (!runtime.accepting_submissions)
        {
            return make_status(status_code::busy,
                "Runtime is draining and does not accept new Worlds.");
        }

        const uint32_t current_capacity = runtime.total_in_flight_capacity();
        const uint64_t required64 =
            static_cast<uint64_t>(current_capacity) + desc.maximum_in_flight_ticks;
        if (required64 > std::numeric_limits<uint32_t>::max())
        {
            return make_status(status_code::capacity_exceeded,
                "Combined World in-flight capacity exceeds uint32.");
        }
        std::string diagnostic;
        if (!runtime.ensure_status_capacity(
                static_cast<uint32_t>(required64), diagnostic))
        {
            return make_status(status_code::busy, diagnostic);
        }

        uint32_t index = asset::invalid_index;
        for (uint32_t candidate = 0; candidate < runtime.worlds.size(); ++candidate)
        {
            if (!runtime.worlds[candidate].occupied &&
                !runtime.worlds[candidate].retired)
            {
                index = candidate;
                break;
            }
        }
        if (index == asset::invalid_index)
        {
            if (runtime.worlds.size() >= asset::invalid_index)
            {
                return make_status(status_code::capacity_exceeded,
                    "World Handle index space is exhausted.");
            }
            index = static_cast<uint32_t>(runtime.worlds.size());
            runtime.worlds.emplace_back();
        }

        implementation::world_slot& world = runtime.worlds[index];
        world.description = desc;
        world.execution_budget.maximum_substeps = desc.maximum_substeps;
        world.execution_budget.maximum_integration_segments_per_tick =
            desc.maximum_integration_segments_per_tick;
        world.execution_budget_revision = 1u;
        world.next_input_snapshot_version = 1u;
        world.cpu_preflight_block_status_observed = false;
        world.gpu_dynamic_toi_block_status_observed = false;
        world.playback = playback_state::stopped;
        world.accumulator_seconds = 0.0;
        world.requested_simulation_time = 0.0;
        world.cancelled_requested_time = 0.0;
        world.reserved_tick_time = 0.0;
        world.gpu_committed_simulation_time = 0.0;
        world.cpu_observed_completed_time = 0.0;
        world.submitted_sequence = 0u;
        world.gpu_committed_sequence = 0u;
        world.cpu_observed_sequence = 0u;
        world.next_tick_sequence = 1u;
        world.pending_single_step_requests = 0u;
        world.reservations.clear();
        world.in_flight_ticks.clear();
        world.in_flight_tick_count = 0u;
        world.commit_control_buffer.Reset();
        world.commit_control_uav.Reset();
        world.gpu_state_generation = 1u;
        world.active_state_set = 0u;
        world.scheduled_predecessor_sequence = 0u;
        world.scheduled_source_generation = 1u;
        world.scheduled_source_state_set = 0u;
        world.render_lease_active = false;
        // Keep the lease generation monotonic when a World slot is reused.
        world.leased_render_slot_id = asset::invalid_index;
        world.leased_render_slot_generation = 0u;
        world.render_candidate_slots.clear();
        world.render_candidate_slots.resize(
            static_cast<std::size_t>(desc.maximum_in_flight_ticks) + 1u);
        world.latest_render_slot_id = asset::invalid_index;
        world.latest_render_slot_generation = 0u;
        world.state = tick_state::idle;
        world.last_tick_status = status_code::ok;
        world.budget_block_origin = work_budget_block_origin::none;
        world.budget_resolution = work_budget_resolution::none;
        world.work_budget_block_flags = 0u;
        world.planned_substeps = 0u;
        world.planned_integration_segments = 0u;
        world.executed_integration_segments = 0u;
        world.minimum_required_integration_segments = 0u;
        world.diagnostic.clear();
        world.models.clear();
        world.colliders.clear();
        world.forces.clear();
        world.impulses.clear();
        world.event_ids.clear();
        world.observed_tick_sequences.clear();
        world.observed_tick_deltas.clear();
        if (!runtime.initialize_world_commit_control(world, diagnostic))
        {
            world.commit_control_buffer.Reset();
            world.commit_control_uav.Reset();
            return make_status(status_code::out_of_memory, diagnostic);
        }
        world.occupied = true;

        out_world->index = index;
        out_world->generation = world.generation;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::add_model(
        world_handle world_handle_value,
        const model_desc& desc,
        model_handle* out_model)
    {
        if (_implementation == nullptr || !_implementation->initialized)
        {
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        }
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (out_model == nullptr || !valid_transform(desc.world_transform))
        {
            return make_status(status_code::invalid_argument,
                "Model output Handle or Root Transform is invalid.");
        }
        transform root_transform = desc.world_transform;
        if (!normalize_transform_rotation(root_transform))
        {
            return make_status(status_code::invalid_argument,
                "Model Root Transform Quaternion could not be normalized.");
        }
        if (runtime.has_world_submission(*world) ||
            world->render_lease_active)
        {
            return make_status(status_code::busy,
                "Model lifecycle changes require an idle World with no Host Render Lease.");
        }

        const std::vector<asset::diagnostic> asset_diagnostics =
            asset::validate_model(desc.source_model);
        const bool has_asset_error = std::any_of(
            asset_diagnostics.begin(), asset_diagnostics.end(),
            [](const asset::diagnostic& diagnostic)
            {
                return diagnostic.severity == asset::diagnostic_severity::error;
            });
        if (has_asset_error)
        {
            return make_status(status_code::asset_invalid,
                "Source Model failed asset validation.");
        }

        std::vector<model_mesh_physics_asset> mesh_assets =
            desc.mesh_physics_assets;
        std::set<uint32_t> supplied_mesh_indices;
        for (model_mesh_physics_asset& mesh_asset : mesh_assets)
        {
            if (mesh_asset.mesh_index >= desc.source_model.meshes.size() ||
                !supplied_mesh_indices.insert(mesh_asset.mesh_index).second ||
                !valid_mesh_asset(mesh_asset.physics_mesh))
            {
                return make_status(status_code::asset_invalid,
                    "Model Physics Mesh Asset has a duplicate / invalid Mesh Index or malformed Physics Mesh.");
            }
            const asset::mesh& render_mesh =
                desc.source_model.meshes[mesh_asset.mesh_index];
            if (render_mesh.mode != asset::physics_mode::soft_body)
            {
                return make_status(status_code::invalid_argument,
                    "Physics Mesh Assets are accepted only for Soft Body Meshes.");
            }

            std::vector<uint32_t> mesh_node_indices;
            for (uint32_t node_index = 0u;
                node_index < desc.source_model.nodes.size(); ++node_index)
            {
                if (desc.source_model.nodes[node_index].mesh_index ==
                    mesh_asset.mesh_index)
                {
                    mesh_node_indices.push_back(node_index);
                }
            }
            if (mesh_node_indices.size() > 1u)
            {
                return make_status(status_code::unsupported_feature,
                    "A Soft Body Physics Mesh currently requires one unique Asset Mesh Node instance. Multiple Node instances need separate Physics State Banks.");
            }

            if (mesh_node_indices.empty())
            {
                if (mesh_asset.skin_node_index != asset::invalid_index ||
                    mesh_asset.skin_index != asset::invalid_index ||
                    mesh_asset.physics_mesh.has_skin_weights ||
                    !mesh_asset.physics_mesh.skin_weights.empty())
                {
                    return make_status(status_code::asset_invalid,
                        "Physics Skin Weights require a unique Skinned Mesh Node in the Source Model.");
                }
            }
            else
            {
                const uint32_t skin_node_index = mesh_node_indices.front();
                const asset::node& skin_node =
                    desc.source_model.nodes[skin_node_index];
                if (mesh_asset.skin_node_index != asset::invalid_index &&
                    mesh_asset.skin_node_index != skin_node_index)
                {
                    return make_status(status_code::asset_invalid,
                        "Physics Mesh Asset Skin Node Index disagrees with the Source Model Node hierarchy.");
                }
                mesh_asset.skin_node_index = skin_node_index;

                if (skin_node.skin_index == asset::invalid_index)
                {
                    if (mesh_asset.skin_index != asset::invalid_index ||
                        mesh_asset.physics_mesh.has_skin_weights ||
                        !mesh_asset.physics_mesh.skin_weights.empty())
                    {
                        return make_status(status_code::asset_invalid,
                            "An unskinned Mesh Node cannot own Physics Skin Weights or a Skin Index.");
                    }
                    mesh_asset.skin_index = asset::invalid_index;
                }
                else
                {
                    if (skin_node.skin_index >= desc.source_model.skins.size() ||
                        (mesh_asset.skin_index != asset::invalid_index &&
                            mesh_asset.skin_index != skin_node.skin_index))
                    {
                        return make_status(status_code::asset_invalid,
                            "Physics Mesh Asset Skin Index disagrees with the Source Model Skinned Mesh Node.");
                    }
                    mesh_asset.skin_index = skin_node.skin_index;

                    const asset::skin& source_skin =
                        desc.source_model.skins[mesh_asset.skin_index];
                    if (!mesh_asset.physics_mesh.has_skin_weights)
                    {
                        if (!mesh_asset.physics_mesh.skin_weights.empty())
                        {
                            return make_status(status_code::asset_invalid,
                                "Physics Skin Weight storage is non-empty while its validity flag is false.");
                        }

                        const physics::skin_weight_transfer_result skin_transfer =
                            physics::transfer_skin_weights(
                                render_mesh, source_skin, mesh_asset.physics_mesh);
                        if (!skin_transfer.success)
                        {
                            const std::string diagnostic =
                                skin_transfer.diagnostics.empty()
                                    ? "unknown skin transfer failure"
                                    : skin_transfer.diagnostics.front().message;
                            return make_status(status_code::asset_invalid,
                                "Physics Skin Weight transfer failed: " + diagnostic);
                        }
                    }

                    if (!mesh_asset.physics_mesh.has_skin_weights ||
                        mesh_asset.physics_mesh.skin_weights.size() !=
                            mesh_asset.physics_mesh.rest_positions.size())
                    {
                        return make_status(status_code::asset_invalid,
                            "Transferred Physics Skin Weight count must match the Physics Vertex count.");
                    }

                    for (std::size_t vertex_index = 0u;
                        vertex_index < mesh_asset.physics_mesh.skin_weights.size();
                        ++vertex_index)
                    {
                        const physics::physics_skin_weight& weights =
                            mesh_asset.physics_mesh.skin_weights[vertex_index];
                        double weight_sum = 0.0;
                        for (std::size_t influence_index = 0u;
                            influence_index < asset::max_skin_influences;
                            ++influence_index)
                        {
                            const float weight = weights.weights[influence_index];
                            if (!finite_float(weight) || weight < 0.0f ||
                                (weight > 0.0f &&
                                    weights.joints[influence_index] >=
                                        source_skin.joints.size()))
                            {
                                return make_status(status_code::asset_invalid,
                                    "Physics Skin Weight contains a non-finite weight or an influence outside the selected Skin Joint range.");
                            }
                            weight_sum += weight;
                        }
                        if (!std::isfinite(weight_sum) ||
                            std::abs(weight_sum - 1.0) > 1.0e-3)
                        {
                            return make_status(status_code::asset_invalid,
                                "Every Physics Vertex Skin Weight must be normalized to one.");
                        }
                    }
                }
            }

            if (mesh_asset.render_bindings.empty())
            {
                const physics::render_binding_build_result binding_result =
                    physics::build_render_vertex_bindings(
                        render_mesh, mesh_asset.physics_mesh);
                if (!binding_result.success)
                {
                    return make_status(status_code::asset_invalid,
                        "Soft Body Render Vertex Binding construction failed.");
                }
                mesh_asset.render_bindings = binding_result.bindings;
            }
            if (mesh_asset.render_bindings.size() != render_mesh.vertices.size())
            {
                return make_status(status_code::asset_invalid,
                    "Render Vertex Binding count must match the Render Mesh Vertex count.");
            }
            const physics::render_feedback_result binding_validation =
                physics::apply_render_feedback(
                    render_mesh, mesh_asset.physics_mesh,
                    mesh_asset.render_bindings);
            if (!binding_validation.success)
            {
                return make_status(status_code::asset_invalid,
                    "Supplied Render Vertex Bindings failed Physics Mesh validation.");
            }

            const physics::physics_attribute_transfer_result attribute_transfer =
                physics::transfer_physics_vertex_attribute_weights(
                    render_mesh, mesh_asset.physics_mesh);
            if (!attribute_transfer.success)
            {
                return make_status(status_code::asset_invalid,
                    "Physics Attribute Paint transfer failed: " +
                        attribute_transfer.diagnostic);
            }
            mesh_asset.physics_mesh.vertex_attribute_weights =
                attribute_transfer.vertex_weights;
        }

        for (uint32_t mesh_index = 0;
            mesh_index < desc.source_model.meshes.size(); ++mesh_index)
        {
            if (desc.source_model.meshes[mesh_index].mode ==
                    asset::physics_mode::soft_body &&
                supplied_mesh_indices.count(mesh_index) == 0u)
            {
                return make_status(status_code::asset_invalid,
                    "Every Soft Body Mesh requires a matching Physics Mesh Asset.");
            }
        }

        std::vector<implementation::soft_body_gpu_mesh> gpu_soft_body_meshes;
        gpu_soft_body_meshes.reserve(mesh_assets.size());
        for (const model_mesh_physics_asset& mesh_asset : mesh_assets)
        {
            implementation::soft_body_gpu_mesh gpu_mesh;
            std::string diagnostic;
            const asset::skin* source_skin =
                mesh_asset.skin_index == asset::invalid_index
                    ? nullptr
                    : &desc.source_model.skins[mesh_asset.skin_index];
            if (!runtime.initialize_soft_body_gpu_mesh(
                    mesh_asset.mesh_index,
                    mesh_asset,
                    source_skin,
                    desc.source_model.meshes[mesh_asset.mesh_index],
                    root_transform,
                    static_cast<uint32_t>(world->render_candidate_slots.size()),
                    gpu_mesh,
                    diagnostic))
            {
                return make_status(status_code::out_of_memory,
                    "Soft Body GPU State Bank creation failed: " + diagnostic);
            }
            gpu_soft_body_meshes.push_back(std::move(gpu_mesh));
        }

        uint32_t index = asset::invalid_index;
        for (uint32_t candidate = 0; candidate < world->models.size(); ++candidate)
        {
            if (!world->models[candidate].occupied &&
                !world->models[candidate].retired)
            {
                index = candidate;
                break;
            }
        }
        if (index == asset::invalid_index)
        {
            if (world->models.size() >= asset::invalid_index)
            {
                return make_status(status_code::capacity_exceeded,
                    "Model Handle index space is exhausted.");
            }
            index = static_cast<uint32_t>(world->models.size());
            world->models.emplace_back();
        }

        implementation::model_slot& model = world->models[index];
        model.source_model = desc.source_model;
        model.mesh_physics_assets = std::move(mesh_assets);
        model.gpu_soft_body_meshes = std::move(gpu_soft_body_meshes);
        model.root_transform = root_transform;
        model.root_samples.clear();
        model.has_animation_playback = false;
        model.has_animation_blend = false;
        model.animation_world_time_origin =
            world->gpu_committed_simulation_time;
        model.mesh_parameters.resize(model.source_model.meshes.size());
        model.mesh_parameters_valid.resize(model.source_model.meshes.size(), false);
        model.occupied = true;

        out_model->index = index;
        out_model->generation = model.generation;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::create_static_collider(
        world_handle world_handle_value,
        const collider_desc& desc,
        collider_handle* out_collider)
    {
        if (_implementation == nullptr || !_implementation->initialized)
        {
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        }
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (out_collider == nullptr || !valid_collider_desc(desc))
        {
            return make_status(status_code::invalid_argument,
                "Static Collider descriptor or output Handle is invalid.");
        }
        if (runtime.has_world_submission(*world) || world->render_lease_active)
        {
            return make_status(status_code::busy,
                "Collider lifecycle changes require an idle World with no Host Render Lease.");
        }

        uint32_t index = asset::invalid_index;
        for (uint32_t candidate = 0; candidate < world->colliders.size(); ++candidate)
        {
            if (!world->colliders[candidate].occupied &&
                !world->colliders[candidate].retired)
            {
                index = candidate;
                break;
            }
        }
        if (index == asset::invalid_index)
        {
            if (world->colliders.size() >= asset::invalid_index)
            {
                return make_status(status_code::capacity_exceeded,
                    "Collider Handle index space is exhausted.");
            }
            index = static_cast<uint32_t>(world->colliders.size());
            world->colliders.emplace_back();
        }

        implementation::collider_slot& collider = world->colliders[index];
        collider.description = desc;
        if (!normalize_transform_rotation(collider.description.initial_transform))
        {
            return make_status(status_code::invalid_argument,
                "Collider Initial Transform Quaternion could not be normalized.");
        }
        collider.kinematic = false;
        collider.pose_samples.clear();
        collider.occupied = true;
        out_collider->index = index;
        out_collider->generation = collider.generation;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::create_kinematic_collider(
        world_handle world_handle_value,
        const collider_desc& desc,
        collider_handle* out_collider)
    {
        status result = create_static_collider(world_handle_value, desc, out_collider);
        if (!result.success())
        {
            return result;
        }
        implementation::world_slot* world =
            _implementation->find_world(world_handle_value);
        if (world == nullptr || out_collider == nullptr)
        {
            return make_status(status_code::stale_handle,
                "World or Collider Handle became stale during creation.");
        }
        implementation::collider_slot* collider =
            _implementation->find_collider(*world, *out_collider);
        if (collider == nullptr)
        {
            return make_status(status_code::stale_handle,
                "New Collider Handle could not be resolved.");
        }
        collider->kinematic = true;
        collider->pose_samples.push_back({
            0.0, collider->description.initial_transform, false
        });
        return make_status(status_code::ok, {});
    }

    status physics_runtime::destroy_model(
        world_handle world_handle_value,
        model_handle model_handle_value)
    {
        if (_implementation == nullptr || !_implementation->initialized)
        {
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        }
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::model_slot* model =
            runtime.find_model(*world, model_handle_value);
        if (model == nullptr) return make_status(status_code::stale_handle, "Model Handle is stale.");
        if (runtime.has_world_submission(*world) || world->render_lease_active)
        {
            return make_status(status_code::busy,
                "Model cannot be destroyed while a Tick or Host Render Lease can reference it.");
        }
        model->occupied = false;
        model->source_model = {};
        model->mesh_physics_assets.clear();
        model->gpu_soft_body_meshes.clear();
        model->root_samples.clear();
        model->has_animation_playback = false;
        model->has_animation_blend = false;
        model->mesh_parameters.clear();
        model->mesh_parameters_valid.clear();
        if (model->generation == std::numeric_limits<uint32_t>::max())
        {
            model->retired = true;
        }
        else
        {
            ++model->generation;
        }
        return make_status(status_code::ok, {});
    }

    status physics_runtime::destroy_collider(
        world_handle world_handle_value,
        collider_handle collider_handle_value)
    {
        if (_implementation == nullptr || !_implementation->initialized)
        {
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        }
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::collider_slot* collider =
            runtime.find_collider(*world, collider_handle_value);
        if (collider == nullptr) return make_status(status_code::stale_handle, "Collider Handle is stale.");
        if (runtime.has_world_submission(*world) || world->render_lease_active)
        {
            return make_status(status_code::busy,
                "Collider cannot be destroyed while a Tick or Host Render Lease can reference it.");
        }
        collider->occupied = false;
        collider->description = {};
        collider->pose_samples.clear();
        if (collider->generation == std::numeric_limits<uint32_t>::max())
        {
            collider->retired = true;
        }
        else
        {
            ++collider->generation;
        }
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_playback_state(
        world_handle world_handle_value,
        playback_state state)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (state != playback_state::playing &&
            state != playback_state::paused &&
            state != playback_state::stopped)
        {
            return make_status(status_code::invalid_argument, "Playback State is invalid.");
        }
        if (state == playback_state::stopped &&
            world->pending_single_step_requests != 0u)
        {
            const double cancelled = static_cast<double>(
                world->pending_single_step_requests) * world->description.fixed_delta_time;
            if (!finite_double(world->cancelled_requested_time + cancelled))
            {
                return make_status(status_code::capacity_exceeded,
                    "Cancelled requested time would overflow the World timeline.");
            }
            world->cancelled_requested_time += cancelled;
            world->pending_single_step_requests = 0u;
        }
        world->playback = state;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_fixed_delta_time(
        world_handle world_handle_value,
        double fixed_delta_time)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (!finite_double(fixed_delta_time) || fixed_delta_time <= 0.0)
        {
            return make_status(status_code::invalid_argument,
                "Fixed Delta Time must be finite and positive.");
        }
        if (world->playback != playback_state::paused ||
            world->in_flight_tick_count != 0u ||
            world->reserved_tick_time > time_epsilon ||
            count_pending_ticks(world->accumulator_seconds,
                world->description.fixed_delta_time) != 0u ||
            world->pending_single_step_requests != 0u)
        {
            return make_status(status_code::busy,
                "Fixed Delta Time can change only while Paused with no whole pending Tick, Step Request or in-flight Submission.");
        }
        world->description.fixed_delta_time = fixed_delta_time;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_time_scale(
        world_handle world_handle_value,
        double time_scale)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (!finite_double(time_scale) || time_scale < 0.0)
        {
            return make_status(status_code::invalid_argument,
                "Time Scale must be finite and non-negative.");
        }
        world->description.time_scale = time_scale;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_world_execution_budget(
        world_handle world_handle_value,
        const world_execution_budget& budget)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (world->state == tick_state::failed ||
            world->state == tick_state::rejected)
        {
            return make_status(status_code::invalid_state,
                "A terminal World cannot change its Execution Budget; reset_world is required.");
        }
        if (budget.maximum_substeps < world->description.minimum_substeps ||
            budget.maximum_substeps > runtime.desc.maximum_world_substeps_hard_limit ||
            budget.maximum_integration_segments_per_tick == 0u ||
            budget.maximum_integration_segments_per_tick >
                runtime.desc.maximum_integration_segments_per_tick_hard_limit)
        {
            return make_status(status_code::invalid_argument,
                "World Execution Budget is outside the Runtime Hard Limits or below the minimum substep count.");
        }
        if (world->execution_budget_revision == std::numeric_limits<uint64_t>::max())
        {
            return make_status(status_code::capacity_exceeded,
                "World Execution Budget Revision cannot advance without uint64 overflow.");
        }

        const bool has_head_reservation = !world->reservations.empty();
        const bool head_is_oldest_uncommitted_sequence =
            has_head_reservation &&
            world->gpu_committed_sequence !=
                std::numeric_limits<uint64_t>::max() &&
            world->reservations.front().sequence ==
                world->gpu_committed_sequence + 1u;
        const bool has_observed_retryable_budget_block =
            head_is_oldest_uncommitted_sequence &&
            world->reservations.front().blocked &&
            !world->reservations.front().rejected &&
            !world->reservations.front().predecessor_blocked &&
            world->state == tick_state::blocked &&
            world->last_tick_status == status_code::work_budget_exceeded &&
            world->budget_resolution == work_budget_resolution::retryable &&
            (world->budget_block_origin ==
                    work_budget_block_origin::cpu_preflight ||
                world->budget_block_origin ==
                    work_budget_block_origin::gpu_dynamic_toi);

        if (has_head_reservation)
        {
            if (world->budget_resolution ==
                work_budget_resolution::hard_limit_reached)
            {
                return make_status(status_code::invalid_state,
                    "The blocked Reservation exceeds an immutable Runtime Hard Limit; reset_world is required.");
            }

            // No plan may be rebuilt while any earlier/later Submission or
            // Event Query from this World is still unobserved.
            if (!has_observed_retryable_budget_block ||
                world->in_flight_tick_count != 0u ||
                !world->in_flight_ticks.empty())
            {
                return make_status(status_code::busy,
                    "Budget Retry requires the oldest CPU-observed retryable Work Budget Reservation and zero in-flight Ticks.");
            }
            if (world->budget_block_origin ==
                    work_budget_block_origin::cpu_preflight &&
                !world->cpu_preflight_block_status_observed)
            {
                return make_status(status_code::busy,
                    "CPU Preflight Status must be returned by poll() before changing the World Execution Budget.");
            }
            if (world->budget_block_origin ==
                    work_budget_block_origin::gpu_dynamic_toi &&
                !world->gpu_dynamic_toi_block_status_observed)
            {
                return make_status(status_code::busy,
                    "GPU Dynamic TOI Status must be returned by poll(world) before changing the World Execution Budget.");
            }

            const uint32_t required_segments =
                world->budget_block_origin ==
                    work_budget_block_origin::gpu_dynamic_toi
                    ? world->minimum_required_integration_segments
                    : world->planned_integration_segments;
            if (world->planned_substeps == 0u || required_segments == 0u)
            {
                return make_status(status_code::invalid_state,
                    "The blocked Work Budget Status does not contain a valid retryable Substep / Segment lower bound.");
            }
            if (budget.maximum_substeps <
                    world->execution_budget.maximum_substeps ||
                budget.maximum_integration_segments_per_tick <
                    world->execution_budget.maximum_integration_segments_per_tick)
            {
                return make_status(status_code::invalid_argument,
                    "A blocked Reservation may only increase or preserve each Admission Limit.");
            }
            if (budget.maximum_substeps < world->planned_substeps ||
                budget.maximum_integration_segments_per_tick < required_segments)
            {
                return make_status(status_code::invalid_argument,
                    "The new Admission Limits must satisfy the blocked Reservation's exact Substep and Segment lower bounds.");
            }
        }
        else
        {
            const uint32_t pending_whole_ticks = count_pending_ticks(
                world->accumulator_seconds,
                world->description.fixed_delta_time);
            if (world->in_flight_tick_count != 0u ||
                !world->in_flight_ticks.empty() ||
                world->reserved_tick_time > time_epsilon ||
                world->pending_single_step_requests != 0u ||
                pending_whole_ticks != 0u)
            {
                return make_status(status_code::busy,
                    "An unblocked World may change its Execution Budget only without pending whole Ticks, Step Requests, Reservations or in-flight Submissions.");
            }
        }

        // Commit the two Admission Limits and Revision only after every
        // observation, lifecycle, monotonicity and required-work check succeeds.
        world->execution_budget = budget;
        world->description.maximum_substeps = budget.maximum_substeps;
        world->description.maximum_integration_segments_per_tick =
            budget.maximum_integration_segments_per_tick;
        ++world->execution_budget_revision;

        if (has_head_reservation)
        {
            implementation::tick_reservation& reservation = world->reservations.front();
            reservation.budget_revision = world->execution_budget_revision;
            if (world->budget_block_origin ==
                work_budget_block_origin::gpu_dynamic_toi)
            {
                // CPU Preflight was already satisfied by this fixed Reservation.
                // GPU TOI's discovered Segment lower bound is the additional
                // admission requirement; do not replace it with the base plan.
                reservation.blocked = false;
                reservation.predecessor_blocked = false;
                world->gpu_dynamic_toi_block_status_observed = false;
                world->cpu_preflight_block_status_observed = false;
                world->state = tick_state::pending;
                world->last_tick_status = status_code::ok;
                world->budget_block_origin = work_budget_block_origin::none;
                world->budget_resolution = work_budget_resolution::none;
                world->work_budget_block_flags = 0u;
                world->executed_integration_segments = 0u;
                world->minimum_required_integration_segments = 0u;
                world->diagnostic.clear();
            }
            else
            {
                world->cpu_preflight_block_status_observed = false;
                runtime.update_head_reservation_preflight(*world);
            }
        }
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_gravity(
        world_handle world_handle_value,
        const asset::vec3& gravity)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;
        if (!finite_vec3(gravity))
        {
            return make_status(status_code::invalid_argument,
                "Gravity must contain finite SI acceleration values.");
        }
        world->description.gravity = gravity;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::enqueue_force(
        world_handle world_handle_value,
        const force_desc& force)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;
        force_desc normalized_force = force;
        const double force_end_time = force.start_time + force.duration;
        if (force.event_id == 0u ||
            !finite_vec3(force.position) ||
            !normalize_direction(normalized_force.direction) ||
            !finite_float(force.magnitude) ||
            force.magnitude < 0.0f ||
            !finite_float(force.radius) ||
            force.radius <= 0.0f ||
            !finite_double(force.duration) ||
            force.duration <= 0.0 ||
            !finite_double(force.start_time) ||
            force.start_time < 0.0 ||
            !finite_double(force_end_time) ||
            force_end_time <= force.start_time ||
            !runtime.valid_target(*world, force.target))
        {
            return make_status(status_code::invalid_argument,
                "Force Event descriptor contains an invalid ID, Target, vector, magnitude, radius or time.");
        }
        if (!runtime.has_force_target(*world, force.target))
        {
            return make_status(status_code::no_force_target,
                "Force Target does not resolve to an active Cloth or Soft Body Mesh.");
        }
        if (!world->event_ids.insert(force.event_id).second)
        {
            return make_status(status_code::invalid_argument,
                "Force / Impulse Event ID must be unique within the World.");
        }
        world->forces.push_back(std::move(normalized_force));
        return make_status(status_code::ok, {});
    }

    status physics_runtime::enqueue_impulse(
        world_handle world_handle_value,
        const impulse_desc& impulse)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;
        impulse_desc normalized_impulse = impulse;
        if (impulse.event_id == 0u ||
            !finite_vec3(impulse.position) ||
            !normalize_direction(normalized_impulse.direction) ||
            !finite_float(impulse.impulse_magnitude) ||
            impulse.impulse_magnitude < 0.0f ||
            !finite_float(impulse.radius) ||
            impulse.radius <= 0.0f ||
            !finite_double(impulse.event_time) ||
            impulse.event_time < 0.0 ||
            !runtime.valid_target(*world, impulse.target))
        {
            return make_status(status_code::invalid_argument,
                "Impulse Event descriptor contains an invalid ID, Target, vector, magnitude, radius or time.");
        }
        if (!runtime.has_force_target(*world, impulse.target))
        {
            return make_status(status_code::no_force_target,
                "Impulse Target does not resolve to an active Cloth or Soft Body Mesh.");
        }
        if (!world->event_ids.insert(impulse.event_id).second)
        {
            return make_status(status_code::invalid_argument,
                "Force / Impulse Event ID must be unique within the World.");
        }
        world->impulses.push_back(std::move(normalized_impulse));
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_animation_playback(
        world_handle world_handle_value,
        model_handle model_handle_value,
        const animation_playback_desc& desc)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::model_slot* model = runtime.find_model(*world, model_handle_value);
        if (model == nullptr) return make_status(status_code::stale_handle, "Model Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;
        if (!valid_animation_track(model->source_model, desc.track) ||
            !finite_float(desc.w_event) ||
            desc.w_event < 0.0f || desc.w_event > 1.0f)
        {
            return make_status(status_code::invalid_argument,
                "Animation Playback descriptor contains an invalid Clip, Time, Speed or Event Weight.");
        }
        model->animation_playback = desc;
        model->animation_world_time_origin =
            world->gpu_committed_simulation_time;
        model->has_animation_playback = true;
        model->has_animation_blend = false;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_animation_blend(
        world_handle world_handle_value,
        model_handle model_handle_value,
        const animation_blend_desc& desc)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::model_slot* model = runtime.find_model(*world, model_handle_value);
        if (model == nullptr) return make_status(status_code::stale_handle, "Model Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;

        if (!valid_animation_track(model->source_model, desc.source_track) ||
            !valid_animation_track(model->source_model, desc.target_track) ||
            !finite_double(desc.blend_start_time) ||
            !finite_double(desc.blend_end_time) ||
            desc.blend_end_time <= desc.blend_start_time ||
            !finite_float(desc.weight_start) ||
            !finite_float(desc.weight_end) ||
            !finite_float(desc.tangent_start) ||
            !finite_float(desc.tangent_end) ||
            !finite_float(desc.w_event) ||
            desc.w_event < 0.0f || desc.w_event > 1.0f ||
            !valid_blend_weight_curve(desc))
        {
            return make_status(status_code::invalid_argument,
                "Animation Blend descriptor is invalid or its Hermite Weight Curve leaves [0, 1].");
        }
        model->animation_blend = desc;
        model->animation_world_time_origin =
            world->gpu_committed_simulation_time;
        model->has_animation_blend = true;
        model->has_animation_playback = false;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_model_transform_sample(
        world_handle world_handle_value,
        model_handle model_handle_value,
        double sample_time,
        const transform& world_transform,
        bool is_teleport)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::model_slot* model = runtime.find_model(*world, model_handle_value);
        if (model == nullptr) return make_status(status_code::stale_handle, "Model Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;
        if (!finite_double(sample_time) || sample_time < 0.0 ||
            !valid_transform(world_transform))
        {
            return make_status(status_code::invalid_argument,
                "Model Transform Sample must have a finite non-negative time and valid rigid transform.");
        }
        transform normalized_world_transform = world_transform;
        if (!normalize_transform_rotation(normalized_world_transform))
        {
            return make_status(status_code::invalid_argument,
                "Model Transform Sample Quaternion could not be normalized.");
        }
        if (!model->root_samples.empty() &&
            sample_time < model->root_samples.back().time)
        {
            return make_status(status_code::invalid_argument,
                "Model Transform Samples must be submitted in non-decreasing time order.");
        }
        if (!model->root_samples.empty() &&
            sample_time == model->root_samples.back().time)
        {
            model->root_samples.back() = { sample_time, normalized_world_transform, is_teleport };
        }
        else
        {
            model->root_samples.push_back({ sample_time, normalized_world_transform, is_teleport });
        }
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_mesh_physics_parameters(
        world_handle world_handle_value,
        model_handle model_handle_value,
        uint32_t mesh_index,
        const mesh_physics_parameters& parameters)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::model_slot* model = runtime.find_model(*world, model_handle_value);
        if (model == nullptr) return make_status(status_code::stale_handle, "Model Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;
        if (mesh_index >= model->source_model.meshes.size() ||
            model->source_model.meshes[mesh_index].mode == asset::physics_mode::none ||
            !valid_mesh_physics_parameters(parameters))
        {
            return make_status(status_code::invalid_argument,
                "Mesh Physics Parameters are invalid or target a non-physical Mesh.");
        }
        model->mesh_parameters[mesh_index] = parameters;
        model->mesh_parameters_valid[mesh_index] = true;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::apply_attribute_edit(
        world_handle world_handle_value,
        model_handle model_handle_value,
        uint32_t mesh_index,
        const attribute_edit_desc& edit)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::model_slot* model = runtime.find_model(*world, model_handle_value);
        if (model == nullptr) return make_status(status_code::stale_handle, "Model Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;
        const std::size_t channel = static_cast<std::size_t>(edit.channel);
        if (mesh_index >= model->source_model.meshes.size() ||
            model->source_model.meshes[mesh_index].mode == asset::physics_mode::none ||
            channel >= static_cast<std::size_t>(asset::physics_attribute_channel::count) ||
            !finite_float(edit.target) ||
            !finite_float(edit.strength) ||
            edit.strength < 0.0f || edit.strength > 1.0f)
        {
            return make_status(status_code::invalid_argument,
                "Attribute Edit has an invalid Mesh, Channel, Target or Strength.");
        }
        asset::mesh& mesh = model->source_model.meshes[mesh_index];
        for (uint32_t vertex_index : edit.vertex_indices)
        {
            if (vertex_index >= mesh.vertices.size())
            {
                return make_status(status_code::invalid_argument,
                    "Attribute Edit contains a Vertex Index outside the target Mesh.");
            }
        }
        if (edit.vertex_indices.empty())
        {
            return make_status(status_code::invalid_argument,
                "Attribute Edit requires at least one explicit Vertex Index.");
        }

        std::vector<std::set<uint32_t>> adjacency(mesh.vertices.size());
        for (std::size_t index = 0; index + 2u < mesh.indices.size(); index += 3u)
        {
            const uint32_t a = mesh.indices[index];
            const uint32_t b = mesh.indices[index + 1u];
            const uint32_t c = mesh.indices[index + 2u];
            if (a >= mesh.vertices.size() || b >= mesh.vertices.size() ||
                c >= mesh.vertices.size())
            {
                return make_status(status_code::asset_invalid,
                    "Target Mesh topology became invalid before Attribute Edit.");
            }
            adjacency[a].insert(b);
            adjacency[a].insert(c);
            adjacency[b].insert(a);
            adjacency[b].insert(c);
            adjacency[c].insert(a);
            adjacency[c].insert(b);
        }

        std::vector<float> updated(mesh.vertices.size());
        for (std::size_t index = 0; index < mesh.vertices.size(); ++index)
        {
            updated[index] = mesh.vertices[index].physics_weights[channel];
        }
        for (uint32_t vertex_index : edit.vertex_indices)
        {
            float target = edit.target;
            if (edit.operation == attribute_edit_operation::erase)
            {
                target = 1.0f;
            }
            else if (edit.operation == attribute_edit_operation::smooth ||
                edit.operation == attribute_edit_operation::average)
            {
                if (adjacency[vertex_index].empty())
                {
                    continue;
                }
                double total = 0.0;
                for (uint32_t neighbour : adjacency[vertex_index])
                {
                    total += mesh.vertices[neighbour].physics_weights[channel];
                }
                target = static_cast<float>(
                    total / static_cast<double>(adjacency[vertex_index].size()));
            }
            else if (edit.operation != attribute_edit_operation::paint)
            {
                return make_status(status_code::invalid_argument,
                    "Attribute Edit operation is unsupported.");
            }

            const float current = mesh.vertices[vertex_index].physics_weights[channel];
            updated[vertex_index] = std::clamp(
                current + (target - current) * edit.strength, 0.0f, 1.0f);
        }
        asset::mesh updated_mesh = mesh;
        for (uint32_t vertex_index : edit.vertex_indices)
        {
            updated_mesh.vertices[vertex_index].physics_weights[channel] =
                updated[vertex_index];
        }

        const auto physics_asset_iterator = std::find_if(
            model->mesh_physics_assets.begin(),
            model->mesh_physics_assets.end(),
            [mesh_index](const model_mesh_physics_asset& value)
            {
                return value.mesh_index == mesh_index;
            });
        if (physics_asset_iterator != model->mesh_physics_assets.end())
        {
            const physics::physics_attribute_transfer_result transferred =
                physics::transfer_physics_vertex_attribute_weights(
                    updated_mesh, physics_asset_iterator->physics_mesh);
            if (!transferred.success)
            {
                return make_status(status_code::asset_invalid,
                    "Paint edit could not be transferred to the Physics Mesh: " +
                        transferred.diagnostic);
            }

            // Commit both Render Mesh paint and the Physics Mesh transfer only after
            // the complete boundary projection / harmonic solve has succeeded.
            physics_asset_iterator->physics_mesh.vertex_attribute_weights =
                transferred.vertex_weights;
        }

        mesh = std::move(updated_mesh);
        return make_status(status_code::ok, {});
    }

    status physics_runtime::set_kinematic_pose_sample(
        world_handle world_handle_value,
        collider_handle collider_handle_value,
        double sample_time,
        const transform& pose)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::collider_slot* collider =
            runtime.find_collider(*world, collider_handle_value);
        if (collider == nullptr) return make_status(status_code::stale_handle, "Collider Handle is stale.");
        const status input_status = runtime.validate_input_mutation(*world);
        if (!input_status.success()) return input_status;
        if (!collider->kinematic)
        {
            return make_status(status_code::invalid_state,
                "Kinematic Pose Samples can be submitted only to Kinematic Colliders.");
        }
        if (!finite_double(sample_time) || sample_time < 0.0 ||
            !valid_transform(pose))
        {
            return make_status(status_code::invalid_argument,
                "Kinematic Pose Sample has an invalid time or transform.");
        }
        transform normalized_pose = pose;
        if (!normalize_transform_rotation(normalized_pose))
        {
            return make_status(status_code::invalid_argument,
                "Kinematic Pose Sample Quaternion could not be normalized.");
        }
        if (!collider->pose_samples.empty() &&
            sample_time < collider->pose_samples.back().time)
        {
            return make_status(status_code::invalid_argument,
                "Kinematic Pose Samples must be submitted in non-decreasing time order.");
        }
        if (!collider->pose_samples.empty() &&
            sample_time == collider->pose_samples.back().time)
        {
            collider->pose_samples.back() = { sample_time, normalized_pose, false };
        }
        else
        {
            collider->pose_samples.push_back({ sample_time, normalized_pose, false });
        }
        return make_status(status_code::ok, {});
    }

    status physics_runtime::advance(
        world_handle world_handle_value,
        double frame_delta_time,
        advance_report* out_report)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (world->state == tick_state::failed)
        {
            return make_status(status_code::invalid_state,
                "World is in a terminal FAILED state; reset_world is required before further Tick requests.");
        }
        if (out_report == nullptr || !finite_double(frame_delta_time) ||
            frame_delta_time < 0.0)
        {
            return make_status(status_code::invalid_argument,
                "Advance requires a report output and finite non-negative frame delta.");
        }

        *out_report = {};
        if (!runtime.accepting_submissions)
        {
            return make_status(status_code::busy,
                "Runtime is draining and does not accept new Tick submissions.");
        }
        if (world->playback == playback_state::playing)
        {
            const double requested_delta =
                frame_delta_time * world->description.time_scale;
            if (!finite_double(requested_delta) ||
                !finite_double(world->requested_simulation_time + requested_delta) ||
                !finite_double(world->accumulator_seconds + requested_delta))
            {
                return make_status(status_code::invalid_argument,
                    "Advance would overflow the World requested-time accumulator.");
            }
            world->requested_simulation_time += requested_delta;
            world->accumulator_seconds += requested_delta;
        }

        const uint64_t previous_submitted_sequence = world->submitted_sequence;
        uint32_t submission_attempts = 0u;
        while (submission_attempts < world->description.maximum_steps_per_advance &&
            world->in_flight_tick_count <
                world->description.maximum_in_flight_ticks)
        {
            if (world->reservations.empty())
            {
                if (world->pending_single_step_requests != 0u)
                {
                    if (!runtime.reserve_next_tick(*world, true))
                    {
                        break;
                    }
                }
                else if (world->playback == playback_state::playing)
                {
                    if (!runtime.reserve_next_tick(*world, false))
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }

            if (world->reservations.front().blocked ||
                world->reservations.front().rejected)
            {
                break;
            }
            if (!runtime.submit_head_reservation(
                    *world, world_handle_value.index))
            {
                break;
            }
            ++submission_attempts;
        }
        out_report->submitted_tick_count = static_cast<uint32_t>(
            world->submitted_sequence - previous_submitted_sequence);

        const uint64_t pending64 =
            static_cast<uint64_t>(count_pending_ticks(
                world->accumulator_seconds, world->description.fixed_delta_time)) +
            world->pending_single_step_requests +
            world->reservations.size();
        out_report->pending_tick_count = static_cast<uint32_t>(
            std::min<uint64_t>(pending64, std::numeric_limits<uint32_t>::max()));
        out_report->in_flight_tick_count = world->in_flight_tick_count;
        out_report->accumulator_seconds = world->accumulator_seconds;
        const bool has_pending_reservation = !world->reservations.empty();
        out_report->render_binding_backpressure =
            has_pending_reservation &&
            world->last_tick_status == status_code::render_binding_busy;
        out_report->capacity_blocked =
            world->last_tick_status == status_code::blocked_by_capacity ||
            world->last_tick_status == status_code::out_of_memory;
        if (!world->reservations.empty() && world->reservations.front().blocked)
        {
            out_report->cpu_preflight_blocked_tick_count = 1u;
            out_report->first_cpu_preflight_blocked_tick_sequence =
                world->reservations.front().sequence;
        }
        return make_status(status_code::ok, {});
    }

    status physics_runtime::poll(
        world_handle world_handle_value,
        world_status* out_status)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (out_status == nullptr)
        {
            return make_status(status_code::invalid_argument,
                "Poll requires a non-null World Status output.");
        }

        const simulation::d3d11_status_ring_poll_result polled =
            runtime.status_ring->poll();
        if (!polled.success())
        {
            const status_code code =
                polled.error == simulation::d3d11_status_ring_error::device_removed
                    ? status_code::device_removed
                    : status_code::gpu_execution_failed;
            return make_status(code, polled.diagnostic);
        }

        for (const simulation::d3d11_status_readback& readback : polled.completed)
        {
            if (readback.bytes.size() != sizeof(gpu_tick_status_record))
            {
                return make_status(status_code::gpu_execution_failed,
                    "Status Ring returned a buffer with an unexpected Tick Status ABI size.");
            }

            const uint32_t world_index =
                static_cast<uint32_t>(readback.key.world_id & 0xffffffffull);
            const uint32_t world_generation =
                static_cast<uint32_t>(readback.key.world_id >> 32u);
            if (world_index >= runtime.worlds.size())
            {
                continue;
            }

            implementation::world_slot& result_world = runtime.worlds[world_index];
            if (!result_world.occupied ||
                result_world.generation != world_generation)
            {
                continue;
            }

            gpu_tick_status_record gpu_status = {};
            std::memcpy(&gpu_status, readback.bytes.data(), sizeof(gpu_status));
            const uint64_t sequence = readback.key.tick_sequence;
            const uint64_t dispatch_id = readback.key.dispatch_id;
            const auto in_flight_iterator = std::find_if(
                result_world.in_flight_ticks.begin(),
                result_world.in_flight_ticks.end(),
                [&](const implementation::in_flight_tick& tick)
                {
                    return tick.reservation.sequence == sequence &&
                        tick.dispatch_id == dispatch_id;
                });
            if (in_flight_iterator == result_world.in_flight_ticks.end())
            {
                // A completed query with no live owner is stale bookkeeping. It
                // must never publish a committed prefix for a recycled World.
                continue;
            }

            implementation::in_flight_tick completed_tick =
                std::move(*in_flight_iterator);
            result_world.in_flight_ticks.erase(in_flight_iterator);
            result_world.in_flight_tick_count = static_cast<uint32_t>(
                result_world.in_flight_ticks.size());

            // A prior, older Submission may already have made the World terminal.
            // Later in-flight Submissions can only report predecessor BLOCKED (or,
            // in a corrupted command stream, an unexpected Commit). They must not
            // downgrade the first terminal result or publish a later State Bank.
            if (result_world.state == tick_state::failed ||
                result_world.state == tick_state::rejected)
            {
                runtime.release_render_candidate_slot(
                    result_world,
                    completed_tick.render_candidate_slot_id,
                    completed_tick.render_candidate_slot_generation);
                completed_tick.reservation.blocked = true;
                runtime.restore_reservation_in_sequence_order(
                    result_world, std::move(completed_tick.reservation));
                continue;
            }

            const uint64_t status_dispatch_id = gpu_status.dispatch_id;
            const uint64_t status_reservation_token = gpu_status.reservation_token;
            const bool matching_snapshot_identity =
                completed_tick.input_snapshot_version != 0u &&
                completed_tick.reservation.input_snapshot != nullptr &&
                completed_tick.reservation.input_snapshot->version ==
                    completed_tick.input_snapshot_version;
            const bool matching_status_identity =
                status_dispatch_id == dispatch_id &&
                status_reservation_token == completed_tick.reservation_token &&
                matching_snapshot_identity;
            if (!matching_status_identity)
            {
                runtime.release_render_candidate_slot(
                    result_world,
                    completed_tick.render_candidate_slot_id,
                    completed_tick.render_candidate_slot_generation);
                completed_tick.reservation.rejected = true;
                completed_tick.reservation.blocked = true;
                runtime.restore_reservation_in_sequence_order(
                    result_world, std::move(completed_tick.reservation));
                result_world.state = tick_state::failed;
                result_world.last_tick_status = status_code::gpu_execution_failed;
                result_world.diagnostic =
                    "GPU Tick Status Dispatch ID / Reservation Token / Input Snapshot Version did not match its immutable Submission Record.";
                continue;
            }

            const uint64_t status_committed_sequence =
                gpu_status.committed_tick_sequence;
            const uint64_t status_state_generation =
                gpu_status.committed_state_generation;

            const bool committed_generation_valid =
                completed_tick.expected_source_generation !=
                    std::numeric_limits<uint64_t>::max() &&
                status_state_generation ==
                    completed_tick.expected_source_generation + 1u;
            const bool committed_bank_valid =
                gpu_status.active_state_set ==
                    completed_tick.expected_destination_state_set;
            const bool committed_sequence_valid =
                status_committed_sequence == sequence;

            // A later in-flight Tick may be gated after an earlier Tick failed.
            // In that case the actual GPU state is an older committed prefix than
            // this immutable Plan's expected predecessor. Status Query records are
            // consumed in Immediate-Context order, so the current CPU-observed
            // committed prefix is the authoritative non-Commit state to preserve.
            const uint32_t expected_published_render_slot_id =
                result_world.latest_render_slot_id;
            const uint32_t expected_published_render_slot_generation =
                result_world.latest_render_slot_id == asset::invalid_index
                    ? 0u
                    : result_world.latest_render_slot_generation;
            const bool non_commit_render_state_unchanged =
                gpu_status.reserved[0] == expected_published_render_slot_id &&
                gpu_status.reserved[1] == expected_published_render_slot_generation;

            const bool non_commit_state_unchanged =
                status_committed_sequence ==
                    result_world.gpu_committed_sequence &&
                status_state_generation ==
                    result_world.gpu_state_generation &&
                gpu_status.active_state_set ==
                    result_world.active_state_set &&
                non_commit_render_state_unchanged;

            const bool committed_render_slot_valid =
                completed_tick.render_candidate_slot_id == asset::invalid_index
                    ? non_commit_render_state_unchanged
                    : gpu_status.reserved[0] ==
                            completed_tick.render_candidate_slot_id &&
                        gpu_status.reserved[1] ==
                            completed_tick.render_candidate_slot_generation &&
                        completed_tick.render_candidate_slot_id <
                            result_world.render_candidate_slots.size() &&
                        result_world.render_candidate_slots[
                            completed_tick.render_candidate_slot_id].reserved &&
                        result_world.render_candidate_slots[
                            completed_tick.render_candidate_slot_id].generation ==
                            completed_tick.render_candidate_slot_generation &&
                        result_world.render_candidate_slots[
                            completed_tick.render_candidate_slot_id].committed_tick_sequence ==
                            sequence;

            if (gpu_status.terminal_state != 6u &&
                !non_commit_state_unchanged)
            {
                runtime.release_render_candidate_slot(
                    result_world,
                    completed_tick.render_candidate_slot_id,
                    completed_tick.render_candidate_slot_generation);
                completed_tick.reservation.blocked = true;
                completed_tick.reservation.rejected = true;
                runtime.restore_reservation_in_sequence_order(
                    result_world, std::move(completed_tick.reservation));
                result_world.state = tick_state::failed;
                result_world.last_tick_status = status_code::gpu_execution_failed;
                result_world.diagnostic =
                    "Non-COMMITTED GPU Tick Status modified the predecessor Sequence, State Generation or active State Bank.";
                continue;
            }

            if (gpu_status.terminal_state == 6u &&
                (!committed_generation_valid ||
                    !committed_bank_valid ||
                    !committed_sequence_valid ||
                    !committed_render_slot_valid))
            {
                runtime.release_render_candidate_slot(
                    result_world,
                    completed_tick.render_candidate_slot_id,
                    completed_tick.render_candidate_slot_generation);
                completed_tick.reservation.blocked = true;
                completed_tick.reservation.rejected = true;
                runtime.restore_reservation_in_sequence_order(
                    result_world, std::move(completed_tick.reservation));
                result_world.state = tick_state::failed;
                result_world.last_tick_status = status_code::gpu_execution_failed;
                result_world.diagnostic =
                    "GPU COMMITTED Status violated the State Bank / Generation / Render Candidate publication contract.";
                continue;
            }

            if (gpu_status.terminal_state == 6u &&
                gpu_status.terminal_failure == 0u &&
                committed_sequence_valid &&
                committed_generation_valid &&
                committed_bank_valid &&
                committed_render_slot_valid)
            {
                if (sequence != result_world.gpu_committed_sequence + 1u)
                {
                    runtime.release_render_candidate_slot(
                        result_world,
                        completed_tick.render_candidate_slot_id,
                        completed_tick.render_candidate_slot_generation);
                    completed_tick.reservation.rejected = true;
                    completed_tick.reservation.blocked = true;
                    runtime.restore_reservation_in_sequence_order(
                        result_world, std::move(completed_tick.reservation));
                    result_world.state = tick_state::failed;
                    result_world.last_tick_status = status_code::gpu_execution_failed;
                    result_world.diagnostic =
                        "GPU Commit Status skipped a Logical Tick Sequence.";
                    continue;
                }

                if (!runtime.commit_render_candidate_slot(
                        result_world,
                        completed_tick.render_candidate_slot_id,
                        completed_tick.render_candidate_slot_generation,
                        sequence))
                {
                    runtime.release_render_candidate_slot(
                        result_world,
                        completed_tick.render_candidate_slot_id,
                        completed_tick.render_candidate_slot_generation);
                    result_world.state = tick_state::failed;
                    result_world.last_tick_status = status_code::gpu_execution_failed;
                    result_world.diagnostic =
                        "GPU Commit succeeded but its reserved Render Candidate Slot could not be published.";
                    continue;
                }

                result_world.gpu_committed_sequence = sequence;
                result_world.gpu_state_generation = status_state_generation;
                result_world.active_state_set = gpu_status.active_state_set;
                result_world.gpu_committed_simulation_time +=
                    completed_tick.reservation.delta_time;
                result_world.observed_tick_deltas[sequence] =
                    completed_tick.reservation.delta_time;

                while (true)
                {
                    const uint64_t next_observed_sequence =
                        result_world.cpu_observed_sequence + 1u;
                    const auto observed_delta =
                        result_world.observed_tick_deltas.find(next_observed_sequence);
                    if (observed_delta ==
                        result_world.observed_tick_deltas.end())
                    {
                        break;
                    }
                    result_world.cpu_observed_sequence = next_observed_sequence;
                    result_world.cpu_observed_completed_time += observed_delta->second;
                    result_world.observed_tick_deltas.erase(observed_delta);
                }

                result_world.reserved_tick_time = std::max(
                    0.0,
                    result_world.reserved_tick_time -
                        completed_tick.reservation.delta_time);

                const bool blocked_head_owns_world_status =
                    !result_world.reservations.empty() &&
                    result_world.reservations.front().blocked &&
                    !result_world.reservations.front().rejected &&
                    !result_world.reservations.front().predecessor_blocked &&
                    result_world.state == tick_state::blocked &&
                    result_world.last_tick_status ==
                        status_code::work_budget_exceeded &&
                    result_world.budget_resolution ==
                        work_budget_resolution::retryable &&
                    (result_world.budget_block_origin ==
                            work_budget_block_origin::cpu_preflight ||
                        result_world.budget_block_origin ==
                            work_budget_block_origin::gpu_dynamic_toi);

                // An older in-flight Tick may Commit while a newer CPU Preflight
                // Reservation is already BLOCKED. Publish the older State Bank and
                // advance its committed time, but retain the newer Head's own
                // Work-Budget Status until its admission limit is repaired.
                if (!blocked_head_owns_world_status)
                {
                    result_world.state = tick_state::committed;
                    result_world.last_tick_status = status_code::ok;
                    result_world.budget_block_origin =
                        work_budget_block_origin::none;
                    result_world.budget_resolution =
                        work_budget_resolution::none;
                    result_world.work_budget_block_flags = 0u;
                    result_world.executed_integration_segments =
                        gpu_status.executed_integration_segments;
                    result_world.minimum_required_integration_segments =
                        gpu_status.minimum_required_integration_segments;
                    result_world.diagnostic.clear();
                }
            }
            else
            {
                runtime.release_render_candidate_slot(
                    result_world,
                    completed_tick.render_candidate_slot_id,
                    completed_tick.render_candidate_slot_generation);
                const uint64_t completed_sequence =
                    completed_tick.reservation.sequence;
                const work_budget_block_origin status_block_origin =
                    static_cast<work_budget_block_origin>(
                        std::min(gpu_status.work_budget_block_origin, 2u));
                const bool gpu_dynamic_toi_block =
                    gpu_status.terminal_state == 3u &&
                    status_block_origin ==
                        work_budget_block_origin::gpu_dynamic_toi;
                const bool predecessor_block =
                    gpu_status.terminal_state == 3u &&
                    !gpu_dynamic_toi_block;

                completed_tick.reservation.blocked = true;
                completed_tick.reservation.rejected =
                    gpu_status.terminal_state == 4u ||
                    gpu_status.terminal_state == 5u;
                completed_tick.reservation.predecessor_blocked =
                    predecessor_block;
                runtime.restore_reservation_in_sequence_order(
                    result_world, std::move(completed_tick.reservation));

                // Only the oldest uncommitted Reservation owns the World-level
                // recovery state. Later GPU submissions are gated by this head and
                // must not replace its original BLOCKED / REJECTED / FAILED reason.
                const bool completed_tick_is_head =
                    !result_world.reservations.empty() &&
                    result_world.reservations.front().sequence == completed_sequence;
                if (completed_tick_is_head)
                {
                    result_world.work_budget_block_flags =
                        static_cast<uint8_t>(
                            gpu_status.work_budget_block_flags & 0xffu);
                    result_world.budget_block_origin = status_block_origin;
                    result_world.executed_integration_segments =
                        gpu_status.executed_integration_segments;
                    result_world.minimum_required_integration_segments =
                        gpu_status.minimum_required_integration_segments;

                    if (gpu_dynamic_toi_block)
                    {
                        const bool dynamic_work_unsatisfiable =
                            gpu_status.minimum_required_integration_segments == 0u ||
                            gpu_status.minimum_required_integration_segments >
                                runtime.desc.maximum_integration_segments_per_tick_hard_limit;
                        result_world.state = tick_state::blocked;
                        result_world.last_tick_status =
                            dynamic_work_unsatisfiable
                                ? status_code::work_budget_unsatisfiable
                                : status_code::work_budget_exceeded;
                        result_world.budget_resolution =
                            dynamic_work_unsatisfiable
                                ? work_budget_resolution::hard_limit_reached
                                : work_budget_resolution::retryable;
                        result_world.gpu_dynamic_toi_block_status_observed = false;
                    }
                    else if (predecessor_block)
                    {
                        result_world.state = tick_state::blocked;
                        result_world.last_tick_status = status_code::blocked_by_predecessor;
                        result_world.budget_block_origin = work_budget_block_origin::none;
                        result_world.budget_resolution = work_budget_resolution::retryable;
                        result_world.gpu_dynamic_toi_block_status_observed = false;
                        result_world.cpu_preflight_block_status_observed = false;
                    }
                    else if (gpu_status.terminal_state == 4u &&
                        gpu_status.diagnostic_code == 7u)
                    {
                        result_world.state = tick_state::rejected;
                        result_world.last_tick_status = status_code::no_force_target;
                        result_world.budget_resolution = work_budget_resolution::none;
                        result_world.diagnostic =
                            "NO_FORCE_TARGET: the selected Soft Body Mesh has no positive mass-weighted Wendland C2 support for this Segment-start action position.";
                    }
                    else if (gpu_status.terminal_state == 4u)
                    {
                        result_world.state = tick_state::rejected;
                        result_world.last_tick_status = status_code::step_rejected;
                        result_world.budget_resolution = work_budget_resolution::none;
                        result_world.diagnostic =
                            "GPU Tick was rejected by a solver or input contract gate.";
                    }
                    else
                    {
                        result_world.state = tick_state::failed;
                        result_world.last_tick_status = status_code::gpu_execution_failed;
                        result_world.budget_resolution = work_budget_resolution::none;
                        result_world.diagnostic =
                            std::string(
                                "GPU Tick failed before Commit; the active State Bank and reservation time were retained. ") +
                            "terminal_state=" +
                            std::to_string(gpu_status.terminal_state) +
                            " terminal_failure=" +
                            std::to_string(gpu_status.terminal_failure) +
                            " diagnostic_code=" +
                            std::to_string(gpu_status.diagnostic_code) +
                            " validation_mask=" +
                            std::to_string(gpu_status.validation_mask) +
                            " required_validation_mask=" +
                            std::to_string(gpu_status.required_validation_mask) +
                            " executed_segments=" +
                            std::to_string(gpu_status.executed_integration_segments) +
                            " minimum_required_segments=" +
                            std::to_string(gpu_status.minimum_required_integration_segments) +
                            " work_budget_block_flags=" +
                            std::to_string(gpu_status.work_budget_block_flags) +
                            " work_budget_block_origin=" +
                            std::to_string(gpu_status.work_budget_block_origin);
                    }
                }
            }
        }

        for (implementation::world_slot& result_world : runtime.worlds)
        {
            if (!result_world.occupied ||
                result_world.in_flight_tick_count != 0u)
            {
                continue;
            }

            runtime.synchronize_scheduled_state_with_committed(result_world);
            if (result_world.state != tick_state::failed &&
                result_world.state != tick_state::rejected &&
                !result_world.reservations.empty() &&
                result_world.reservations.front().predecessor_blocked &&
                result_world.gpu_committed_sequence !=
                    std::numeric_limits<uint64_t>::max() &&
                result_world.reservations.front().sequence ==
                    result_world.gpu_committed_sequence + 1u)
            {
                // The predecessor's Query has now been observed. Rebuild the
                // dependent Reservation's CPU Preflight against the newly committed
                // source; it will receive a fresh immutable GPU Plan on admission.
                runtime.update_head_reservation_preflight(result_world);
            }
        }

        if (world->state == tick_state::blocked &&
            world->last_tick_status == status_code::work_budget_exceeded &&
            !world->reservations.empty() &&
            world->reservations.front().blocked &&
            !world->reservations.front().predecessor_blocked)
        {
            if (world->budget_block_origin ==
                work_budget_block_origin::cpu_preflight)
            {
                // A CPU-only Preflight Block is observable synchronously through
                // this returned world_status; it does not require an Event Query.
                world->cpu_preflight_block_status_observed = true;
            }
            else if (world->budget_block_origin ==
                    work_budget_block_origin::gpu_dynamic_toi &&
                world->in_flight_tick_count == 0u)
            {
                // Only poll(world) returns this Work Budget Status to the Host.
                // Polling another World may have consumed the shared Query Ring
                // entry internally, but does not authorize a Budget Retry here.
                world->gpu_dynamic_toi_block_status_observed = true;
            }
        }

        world_status result;
        result.playback = world->playback;
        result.state = world->state;
        result.last_tick_status = world->last_tick_status;
        result.budget_block_origin = world->budget_block_origin;
        result.budget_resolution = world->budget_resolution;
        result.work_budget_block_flags = world->work_budget_block_flags;
        result.execution_budget_revision = world->execution_budget_revision;
        result.submitted_sequence = world->submitted_sequence;
        result.gpu_committed_sequence = world->gpu_committed_sequence;
        result.cpu_observed_sequence = world->cpu_observed_sequence;
        result.head_reserved_tick_sequence = world->reservations.empty()
            ? 0u : world->reservations.front().sequence;
        result.head_input_snapshot_version =
            world->reservations.empty() ||
            world->reservations.front().input_snapshot == nullptr
                ? 0u
                : world->reservations.front().input_snapshot->version;
        result.planned_substeps = world->planned_substeps;
        result.planned_integration_segments = world->planned_integration_segments;
        result.executed_integration_segments = world->executed_integration_segments;
        result.minimum_required_integration_segments =
            world->minimum_required_integration_segments;
        result.requested_simulation_time = world->requested_simulation_time;
        result.cancelled_requested_time = world->cancelled_requested_time;
        result.effective_requested_time =
            std::max(0.0, world->requested_simulation_time -
                world->cancelled_requested_time);
        result.reserved_tick_time = world->reserved_tick_time;
        result.gpu_committed_simulation_time = world->gpu_committed_simulation_time;
        result.cpu_observed_completed_time = world->cpu_observed_completed_time;
        const uint64_t pending_tick_count =
            static_cast<uint64_t>(count_pending_ticks(
                world->accumulator_seconds, world->description.fixed_delta_time)) +
            world->pending_single_step_requests +
            world->reservations.size();
        result.pending_tick_count = static_cast<uint32_t>(
            std::min<uint64_t>(pending_tick_count,
                std::numeric_limits<uint32_t>::max()));
        result.in_flight_tick_count = world->in_flight_tick_count;
        result.simulation_backlog = result.effective_requested_time -
            result.gpu_committed_simulation_time -
            result.reserved_tick_time;
        result.diagnostic = world->diagnostic;
        *out_status = std::move(result);
        return make_status(status_code::ok, {});
    }

    status physics_runtime::single_step(
        world_handle world_handle_value,
        advance_report* out_report)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (out_report == nullptr)
        {
            return make_status(status_code::invalid_argument,
                "Single Step requires a report output.");
        }
        *out_report = {};
        if (!runtime.accepting_submissions)
        {
            return make_status(status_code::busy,
                "Runtime is draining and does not accept new Step Requests.");
        }
        if (world->playback == playback_state::playing)
        {
            return make_status(status_code::invalid_state,
                "Single Step is accepted only while Paused or Stopped.");
        }
        if (world->pending_single_step_requests ==
            std::numeric_limits<uint32_t>::max())
        {
            return make_status(status_code::capacity_exceeded,
                "Single Step Request Queue is full.");
        }
        const double new_requested_time =
            world->requested_simulation_time + world->description.fixed_delta_time;
        if (!finite_double(new_requested_time))
        {
            return make_status(status_code::capacity_exceeded,
                "Requested Simulation Time would overflow.");
        }
        world->requested_simulation_time = new_requested_time;
        ++world->pending_single_step_requests;
        const uint64_t pending_tick_count =
            static_cast<uint64_t>(count_pending_ticks(
                world->accumulator_seconds, world->description.fixed_delta_time)) +
            world->pending_single_step_requests +
            world->reservations.size();
        out_report->pending_tick_count = static_cast<uint32_t>(
            std::min<uint64_t>(pending_tick_count,
                std::numeric_limits<uint32_t>::max()));
        out_report->in_flight_tick_count = world->in_flight_tick_count;
        out_report->accumulator_seconds = world->accumulator_seconds;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::get_render_binding(
        world_handle world_handle_value,
        model_handle model_handle_value,
        uint32_t mesh_index,
        render_binding* out_binding)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        implementation::model_slot* model = runtime.find_model(*world, model_handle_value);
        if (model == nullptr) return make_status(status_code::stale_handle, "Model Handle is stale.");
        if (out_binding == nullptr || mesh_index >= model->source_model.meshes.size())
        {
            return make_status(status_code::invalid_argument,
                "Render Binding output or Mesh Index is invalid.");
        }

        *out_binding = {};
        if (model->source_model.meshes[mesh_index].mode !=
            asset::physics_mode::soft_body)
        {
            return make_status(status_code::unsupported_feature,
                "GPU Render Candidates are currently provided for Soft Body Meshes only.");
        }

        const auto gpu_mesh_iterator = std::find_if(
            model->gpu_soft_body_meshes.begin(),
            model->gpu_soft_body_meshes.end(),
            [mesh_index](const implementation::soft_body_gpu_mesh& value)
            {
                return value.mesh_index == mesh_index;
            });
        if (gpu_mesh_iterator == model->gpu_soft_body_meshes.end())
        {
            return make_status(status_code::not_ready,
                "Soft Body Mesh has no GPU Render Candidate resources.");
        }

        const uint32_t slot_id = world->render_lease_active
            ? world->leased_render_slot_id
            : world->latest_render_slot_id;
        const uint32_t slot_generation = world->render_lease_active
            ? world->leased_render_slot_generation
            : world->latest_render_slot_generation;
        if (slot_id == asset::invalid_index ||
            slot_id >= world->render_candidate_slots.size())
        {
            return make_status(status_code::not_ready,
                "No CPU-observed committed Render Candidate is available for this World.");
        }

        const implementation::render_candidate_slot_state& slot_state =
            world->render_candidate_slots[slot_id];
        if (!slot_state.committed ||
            slot_state.reserved ||
            slot_state.generation != slot_generation)
        {
            return make_status(status_code::not_ready,
                "Latest Render Candidate Ring Slot is not committed and reusable.");
        }

        const implementation::render_candidate_mesh_slot& mesh_slot =
            gpu_mesh_iterator->render_candidate_slots[slot_id];
        if (mesh_slot.committed_generation != slot_generation ||
            mesh_slot.committed_tick_sequence != slot_state.committed_tick_sequence)
        {
            return make_status(status_code::not_ready,
                "This Model / Mesh did not participate in the latest committed Render Candidate.");
        }

        if (!world->render_lease_active)
        {
            if (world->render_lease_generation ==
                std::numeric_limits<uint64_t>::max())
            {
                return make_status(status_code::capacity_exceeded,
                    "Host Render Lease Generation cannot advance without uint64 overflow.");
            }
            ++world->render_lease_generation;
            world->render_lease_active = true;
            world->leased_render_slot_id = slot_id;
            world->leased_render_slot_generation = slot_generation;
        }

        const asset::mesh& source_mesh = model->source_model.meshes[mesh_index];
        const bool has_normals = std::any_of(
            source_mesh.vertices.begin(),
            source_mesh.vertices.end(),
            [](const asset::vertex& vertex)
            {
                return (vertex.attributes & asset::attribute_normal) != 0u;
            });
        const bool has_tangents = std::any_of(
            source_mesh.vertices.begin(),
            source_mesh.vertices.end(),
            [](const asset::vertex& vertex)
            {
                return (vertex.attributes & asset::attribute_tangent) != 0u;
            });

        out_binding->position_srv = mesh_slot.position_srv.Get();
        out_binding->normal_srv = has_normals ? mesh_slot.normal_srv.Get() : nullptr;
        out_binding->tangent_srv = has_tangents ? mesh_slot.tangent_srv.Get() : nullptr;
        out_binding->format = DXGI_FORMAT_R32G32B32A32_FLOAT;
        out_binding->stride = sizeof(gpu_float4);
        out_binding->vertex_count = gpu_mesh_iterator->render_vertex_count;
        out_binding->index_count = gpu_mesh_iterator->render_index_count;
        out_binding->resource_generation =
            world->render_lease_generation;
        out_binding->device_generation = runtime.device_generation;
        out_binding->committed_tick_sequence =
            slot_state.committed_tick_sequence;
        out_binding->render_slot_id = slot_id;
        out_binding->render_slot_generation = slot_generation;
        out_binding->lease_generation = world->render_lease_generation;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::release_render_bindings(
        world_handle world_handle_value,
        uint64_t lease_generation)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (!world->render_lease_active ||
            world->render_lease_generation != lease_generation)
        {
            return make_status(status_code::stale_handle,
                "Render Lease Generation is stale or already released.");
        }
        world->render_lease_active = false;
        world->leased_render_slot_id = asset::invalid_index;
        world->leased_render_slot_generation = 0u;
        return make_status(status_code::ok, {});
    }

    status physics_runtime::reset_world(world_handle world_handle_value)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");
        if (world->in_flight_tick_count != 0u ||
            world->render_lease_active ||
            runtime.status_ring->pending_count() != 0u)
        {
            return make_status(status_code::busy,
                "Reset requires all GPU Submissions / Status Queries and Host Render Leases to be released.");
        }

        world->playback = playback_state::stopped;
        world->accumulator_seconds = 0.0;
        world->requested_simulation_time = 0.0;
        world->cancelled_requested_time = 0.0;
        world->reserved_tick_time = 0.0;
        world->gpu_committed_simulation_time = 0.0;
        world->cpu_observed_completed_time = 0.0;
        world->submitted_sequence = 0u;
        world->gpu_committed_sequence = 0u;
        world->cpu_observed_sequence = 0u;
        world->next_tick_sequence = 1u;
        world->pending_single_step_requests = 0u;
        world->cpu_preflight_block_status_observed = false;
        world->gpu_dynamic_toi_block_status_observed = false;
        world->reservations.clear();
        world->in_flight_ticks.clear();
        world->in_flight_tick_count = 0u;
        world->gpu_state_generation = 1u;
        world->active_state_set = 0u;
        world->scheduled_predecessor_sequence = 0u;
        world->latest_render_slot_id = asset::invalid_index;
        world->latest_render_slot_generation = 0u;
        world->leased_render_slot_id = asset::invalid_index;
        world->leased_render_slot_generation = 0u;
        world->render_lease_active = false;
        for (implementation::render_candidate_slot_state& slot :
            world->render_candidate_slots)
        {
            slot.reserved = false;
            slot.committed = false;
            slot.committed_tick_sequence = 0u;
        }
        world->scheduled_source_generation = 1u;
        world->scheduled_source_state_set = 0u;
        gpu_tick_commit_control initial_control;
        initial_control.state_generation_low = 1u;
        initial_control.active_state_set = 0u;
        initial_control.required_validation_mask = 1u;
        initial_control.render_slot_id = asset::invalid_index;
        initial_control.render_slot_generation = 0u;
        runtime.immediate_context->UpdateSubresource(
            world->commit_control_buffer.Get(), 0u, nullptr,
            &initial_control, 0u, 0u);
        world->state = tick_state::idle;
        world->last_tick_status = status_code::ok;
        world->budget_block_origin = work_budget_block_origin::none;
        world->budget_resolution = work_budget_resolution::none;
        world->work_budget_block_flags = 0u;
        world->planned_substeps = 0u;
        world->planned_integration_segments = 0u;
        world->executed_integration_segments = 0u;
        world->minimum_required_integration_segments = 0u;
        world->diagnostic.clear();
        world->forces.clear();
        world->impulses.clear();
        world->event_ids.clear();
        world->observed_tick_sequences.clear();
        world->observed_tick_deltas.clear();
        for (implementation::model_slot& model : world->models)
        {
            model.root_samples.clear();
            // Reset restarts the World Simulation Time epoch. Keep retained
            // Playback / Blend descriptors at their authored Clip Time by
            // rebasing their immutable input origin to the new World time.
            model.animation_world_time_origin =
                world->gpu_committed_simulation_time;
            for (implementation::soft_body_gpu_mesh& gpu_mesh :
                model.gpu_soft_body_meshes)
            {
                runtime.immediate_context->UpdateSubresource(
                    gpu_mesh.position_banks[0].Get(), 0u, nullptr,
                    gpu_mesh.initial_positions.data(), 0u, 0u);
                runtime.immediate_context->UpdateSubresource(
                    gpu_mesh.position_banks[1].Get(), 0u, nullptr,
                    gpu_mesh.initial_positions.data(), 0u, 0u);
                runtime.immediate_context->UpdateSubresource(
                    gpu_mesh.velocity_banks[0].Get(), 0u, nullptr,
                    gpu_mesh.initial_velocities.data(), 0u, 0u);
                runtime.immediate_context->UpdateSubresource(
                    gpu_mesh.velocity_banks[1].Get(), 0u, nullptr,
                    gpu_mesh.initial_velocities.data(), 0u, 0u);
                runtime.immediate_context->UpdateSubresource(
                    gpu_mesh.position_scratch.Get(), 0u, nullptr,
                    gpu_mesh.initial_positions.data(), 0u, 0u);
                runtime.immediate_context->UpdateSubresource(
                    gpu_mesh.velocity_scratch.Get(), 0u, nullptr,
                    gpu_mesh.initial_velocities.data(), 0u, 0u);
            }
        }
        for (implementation::collider_slot& collider : world->colliders)
        {
            collider.pose_samples.clear();
            if (collider.occupied && collider.kinematic)
            {
                // Reset returns each Kinematic Collider to its immutable initial
                // pose and re-establishes the t=0 sample used to bracket its
                // first post-reset trajectory interval.
                collider.pose_samples.push_back({
                    0.0, collider.description.initial_transform, false
                });
            }
        }
        return make_status(status_code::ok, {});
    }

    status physics_runtime::destroy_world(world_handle world_handle_value)
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        implementation::world_slot* world = runtime.find_world(world_handle_value);
        if (world == nullptr) return make_status(status_code::stale_handle, "World Handle is stale.");

        const bool has_pending_whole_tick =
            count_pending_ticks(world->accumulator_seconds,
                world->description.fixed_delta_time) != 0u;
        if (world->playback == playback_state::playing ||
            world->in_flight_tick_count != 0u ||
            world->reserved_tick_time > time_epsilon ||
            world->pending_single_step_requests != 0u ||
            !world->reservations.empty() ||
            has_pending_whole_tick ||
            world->accumulator_seconds > time_epsilon ||
            world->render_lease_active)
        {
            return make_status(status_code::busy,
                "World Destroy requires a stopped / paused idle World with no pending time, Reservation, Step Request, GPU Query or Host Render Lease.");
        }

        world->occupied = false;
        world->models.clear();
        world->colliders.clear();
        world->forces.clear();
        world->impulses.clear();
        world->event_ids.clear();
        world->observed_tick_sequences.clear();
        world->observed_tick_deltas.clear();
        world->in_flight_ticks.clear();
        world->commit_control_uav.Reset();
        world->commit_control_buffer.Reset();
        if (world->generation == std::numeric_limits<uint32_t>::max())
        {
            world->retired = true;
        }
        else
        {
            ++world->generation;
        }
        return make_status(status_code::ok, {});
    }

    status physics_runtime::drain()
    {
        if (_implementation == nullptr || !_implementation->initialized)
            return make_status(status_code::not_ready, "Runtime is not initialized.");
        implementation& runtime = *_implementation;
        const status thread = runtime.thread_status();
        if (!thread.success()) return thread;
        runtime.accepting_submissions = false;
        if (runtime.status_ring->pending_count() != 0u)
        {
            return make_status(status_code::busy,
                "Drain is waiting for non-blocking Status Query polling; call poll() until the Query Ring is empty.");
        }
        for (const implementation::world_slot& world : runtime.worlds)
        {
            if (!world.occupied) continue;
            if (world.in_flight_tick_count != 0u ||
                world.reserved_tick_time > time_epsilon ||
                count_pending_ticks(world.accumulator_seconds,
                    world.description.fixed_delta_time) != 0u ||
                world.pending_single_step_requests != 0u ||
                world.render_lease_active)
            {
                return make_status(status_code::busy,
                    "Drain cannot complete while a World has pending Ticks, Requests or Host Render Leases.");
            }
        }
        return make_status(status_code::ok, {});
    }

    status_code physics_runtime::shutdown() noexcept
    {
        if (_implementation == nullptr)
        {
            return status_code::ok;
        }
        implementation& runtime = *_implementation;
        if (!runtime.initialized)
        {
            _implementation.reset();
            return status_code::ok;
        }
        if (!runtime.on_submission_thread())
        {
            return status_code::submission_thread_violation;
        }
        if (runtime.status_ring && runtime.status_ring->pending_count() != 0u)
        {
            return status_code::busy;
        }
        for (const implementation::world_slot& world : runtime.worlds)
        {
            if (world.occupied)
            {
                return status_code::busy;
            }
        }
        runtime.worlds.clear();
        runtime.status_ring.reset();
        runtime.immediate_context.Reset();
        runtime.device.Reset();
        runtime.initialized = false;
        _implementation.reset();
        return status_code::ok;
    }

    bool physics_runtime::initialized() const
    {
        return _implementation != nullptr && _implementation->initialized;
    }

    const runtime_desc* physics_runtime::description() const
    {
        if (_implementation == nullptr || !_implementation->initialized)
        {
            return nullptr;
        }
        return &_implementation->desc;
    }
}
