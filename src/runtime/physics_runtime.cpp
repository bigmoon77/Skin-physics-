#include "skin_physics/runtime/physics_runtime.hpp"

#include "skin_physics/asset/importer.hpp"
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
            float reserved0 = 0.0f;
            float gravity[3] = {};
            float reserved1 = 0.0f;
        };

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
        static_assert(sizeof(gpu_soft_body_integration_parameters) == 32,
            "Soft Body Integration Parameters ABI must remain 32 bytes.");

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
            const double duration = desc.blend_end_time - desc.blend_start_time;
            const double w0 = desc.weight_start;
            const double w1 = desc.weight_end;
            const double m0 = static_cast<double>(desc.tangent_start) * duration;
            const double m1 = static_cast<double>(desc.tangent_end) * duration;

            // w(u) = a*u^3 + b*u^2 + c*u + d, u in [0, 1].
            const double a = 2.0 * w0 - 2.0 * w1 + m0 + m1;
            const double b = -3.0 * w0 + 3.0 * w1 - 2.0 * m0 - m1;
            const double c = m0;
            const double d = w0;

            const auto inside_unit_interval = [](double value)
            {
                return value > 0.0 && value < 1.0;
            };
            const auto valid_weight = [&](double u)
            {
                const double value = ((a * u + b) * u + c) * u + d;
                return std::isfinite(value) && value >= -1.0e-6 && value <= 1.0 + 1.0e-6;
            };

            if (!valid_weight(0.0) || !valid_weight(1.0))
            {
                return false;
            }

            const double qa = 3.0 * a;
            const double qb = 2.0 * b;
            const double qc = c;
            if (std::abs(qa) <= 1.0e-14)
            {
                if (std::abs(qb) <= 1.0e-14)
                {
                    return true;
                }
                const double root = -qc / qb;
                return !inside_unit_interval(root) || valid_weight(root);
            }

            const double discriminant = qb * qb - 4.0 * qa * qc;
            if (discriminant < 0.0)
            {
                return true;
            }
            const double sqrt_discriminant = std::sqrt(discriminant);
            const double root0 = (-qb - sqrt_discriminant) / (2.0 * qa);
            const double root1 = (-qb + sqrt_discriminant) / (2.0 * qa);
            return (!inside_unit_interval(root0) || valid_weight(root0)) &&
                (!inside_unit_interval(root1) || valid_weight(root1));
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
                parameters.damping < 0.0f)
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
            uint32_t vertex_count = 0u;
            uint32_t tetrahedron_count = 0u;

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

        struct tick_reservation
        {
            uint64_t sequence = 0u;
            double delta_time = 0.0;
            bool from_single_step = false;
            bool blocked = false;
            bool rejected = false;
            uint64_t budget_revision = 0u;
            uint32_t planned_substeps = 0u;
            uint32_t planned_integration_segments = 0u;
        };

        struct in_flight_tick
        {
            tick_reservation reservation;
            uint64_t dispatch_id = 0u;
            uint64_t reservation_token = 0u;
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
            uint32_t generation = 1u;
            bool occupied = false;
            bool retired = false;
            world_desc description;
            world_execution_budget execution_budget;
            uint64_t execution_budget_revision = 1u;
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
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_fem_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_force_gather_shader;
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
                mesh.vertex_attribute_weights.size() != mesh.rest_positions.size())
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
                vertex_index < mesh.vertex_attribute_weights.size();
                ++vertex_index)
            {
                const float weight =
                    mesh.vertex_attribute_weights[vertex_index][damping_channel];
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
                    if (vertex_index >= mesh.vertex_attribute_weights.size())
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
                        mesh.vertex_attribute_weights[vertex_index][shape_channel] * 0.25f;
                    average_volume_retention +=
                        mesh.vertex_attribute_weights[vertex_index][volume_channel] * 0.25f;
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
            candidate.vertex_count = static_cast<uint32_t>(mesh.rest_positions.size());
            candidate.tetrahedron_count = static_cast<uint32_t>(mesh.tetrahedra.size());
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
                    mesh, default_parameters, tet_inputs, damping_rate,
                    initial_vertex_damping_rates, diagnostic))
            {
                return false;
            }

            std::vector<gpu_soft_body_tet_output> tet_output(
                mesh.tetrahedra.size(), gpu_soft_body_tet_output{});
            std::vector<gpu_float4> vertex_force(
                mesh.rest_positions.size(), gpu_float4{});

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
            if (asset_iterator == model.mesh_physics_assets.end() ||
                gpu_mesh.mesh_index >= model.mesh_parameters.size() ||
                gpu_mesh.mesh_index >= model.mesh_parameters_valid.size())
            {
                diagnostic = "Soft Body GPU Mesh lost its immutable Physics Mesh Asset owner.";
                return false;
            }

            mesh_physics_parameters parameters =
                model.mesh_parameters_valid[gpu_mesh.mesh_index]
                    ? model.mesh_parameters[gpu_mesh.mesh_index]
                    : mesh_physics_parameters{};
            if (!model.mesh_parameters_valid[gpu_mesh.mesh_index])
            {
                parameters.density = static_cast<float>(gpu_mesh.rest_density);
            }

            std::vector<gpu_soft_body_tet_input> tet_inputs;
            std::vector<float> vertex_damping_rates;
            if (!build_soft_body_tet_inputs(
                    asset_iterator->physics_mesh,
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

            if (vertex_damping_rates.size() != gpu_mesh.vertex_count)
            {
                diagnostic = "Soft Body vertex damping-rate count changed after GPU resource creation.";
                return false;
            }

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
            double& out_dt_limit,
            std::string& diagnostic) const
        {
            constexpr double cfl_factor = 0.2;
            out_dt_limit = std::numeric_limits<double>::infinity();
            diagnostic.clear();

            for (const model_slot& model : world.models)
            {
                if (!model.occupied)
                {
                    continue;
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
                    if (mesh.vertex_attribute_weights.size() !=
                            mesh.rest_positions.size() ||
                        mesh.mass.size() != mesh.rest_positions.size() ||
                        mesh.tetrahedra.empty())
                    {
                        diagnostic =
                            "Soft Body CFL Preflight requires transferred per-vertex paint, lumped mass and Tet data.";
                        return false;
                    }

                    mesh_physics_parameters parameters =
                        mesh_asset.mesh_index < model.mesh_parameters.size() &&
                        mesh_asset.mesh_index < model.mesh_parameters_valid.size() &&
                        model.mesh_parameters_valid[mesh_asset.mesh_index]
                            ? model.mesh_parameters[mesh_asset.mesh_index]
                            : mesh_physics_parameters{};

                    if (!(mesh_asset.mesh_index < model.mesh_parameters_valid.size() &&
                        model.mesh_parameters_valid[mesh_asset.mesh_index]))
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
                                mesh.vertex_attribute_weights[vertex_index];
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

        bool world_has_unsupported_physics_work(const world_slot& world) const
        {
            if (!world.forces.empty() || !world.impulses.empty())
            {
                return true;
            }

            for (const collider_slot& collider : world.colliders)
            {
                if (collider.occupied)
                {
                    return true;
                }
            }

            for (const model_slot& model : world.models)
            {
                if (!model.occupied)
                {
                    continue;
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
                    const bool has_soft_body_resources =
                        std::any_of(
                            model.gpu_soft_body_meshes.begin(),
                            model.gpu_soft_body_meshes.end(),
                            [mesh_index](const soft_body_gpu_mesh& value)
                            {
                                return value.mesh_index == mesh_index;
                            });
                    if (mode != asset::physics_mode::soft_body ||
                        !has_soft_body_resources)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        bool append_soft_body_tick_passes(
            world_slot& world,
            uint32_t source_state_set,
            uint32_t destination_state_set,
            const tick_reservation& reservation,
            simulation::d3d11_compute_graph& graph,
            std::string& diagnostic)
        {
            diagnostic.clear();
            if (source_state_set > 1u || destination_state_set > 1u ||
                source_state_set == destination_state_set ||
                reservation.planned_substeps == 0u ||
                !world_has_soft_body_work(world))
            {
                diagnostic = "Soft Body Tick Passes require valid distinct State Banks and a non-empty Substep Schedule.";
                return false;
            }

            const double delta_time_double =
                reservation.delta_time /
                static_cast<double>(reservation.planned_substeps);
            const float delta_time = static_cast<float>(delta_time_double);
            if (!finite_double(delta_time_double) ||
                delta_time_double <= 0.0 ||
                !finite_float(delta_time) ||
                delta_time <= 0.0f ||
                !finite_vec3(world.description.gravity))
            {
                diagnostic = "Soft Body Integration Segment time or World gravity is not representable as finite GPU float values.";
                return false;
            }

            for (uint32_t model_index = 0u;
                model_index < world.models.size(); ++model_index)
            {
                model_slot& model = world.models[model_index];
                if (!model.occupied)
                {
                    continue;
                }

                for (soft_body_gpu_mesh& gpu_mesh : model.gpu_soft_body_meshes)
                {
                    float damping_rate = 0.0f;
                    if (!update_soft_body_materials(
                            model, gpu_mesh, damping_rate, diagnostic))
                    {
                        return false;
                    }

                    gpu_soft_body_integration_parameters integration_parameters;
                    integration_parameters.vertex_count = gpu_mesh.vertex_count;
                    integration_parameters.delta_time = delta_time;
                    integration_parameters.damping_rate = damping_rate;
                    integration_parameters.gravity[0] = world.description.gravity.x;
                    integration_parameters.gravity[1] = world.description.gravity.y;
                    integration_parameters.gravity[2] = world.description.gravity.z;
                    immediate_context->UpdateSubresource(
                        gpu_mesh.integration_parameters.Get(),
                        0u,
                        nullptr,
                        &integration_parameters,
                        0u,
                        0u);

                    bool source_is_scratch = false;
                    uint32_t source_bank = source_state_set;
                    for (uint32_t substep_index = 0u;
                        substep_index < reservation.planned_substeps;
                        ++substep_index)
                    {
                        ID3D11ShaderResourceView* source_position_srv =
                            source_is_scratch
                                ? gpu_mesh.position_scratch_srv.Get()
                                : gpu_mesh.position_bank_srvs[source_bank].Get();
                        ID3D11ShaderResourceView* source_velocity_srv =
                            source_is_scratch
                                ? gpu_mesh.velocity_scratch_srv.Get()
                                : gpu_mesh.velocity_bank_srvs[source_bank].Get();

                        const bool destination_is_scratch =
                            (substep_index & 1u) == 0u;
                        const uint32_t destination_bank = destination_state_set;
                        ID3D11UnorderedAccessView* destination_position_uav =
                            destination_is_scratch
                                ? gpu_mesh.position_scratch_uav.Get()
                                : gpu_mesh.position_bank_uavs[destination_bank].Get();
                        ID3D11UnorderedAccessView* destination_velocity_uav =
                            destination_is_scratch
                                ? gpu_mesh.velocity_scratch_uav.Get()
                                : gpu_mesh.velocity_bank_uavs[destination_bank].Get();

                        simulation::compute_pass_desc fem_pass;
                        fem_pass.name = "soft_body_fem_m" +
                            std::to_string(model_index) + "_mesh" +
                            std::to_string(gpu_mesh.mesh_index) + "_substep" +
                            std::to_string(substep_index);
                        fem_pass.shader = soft_body_fem_shader.Get();
                        fem_pass.shader_resources = {
                            { 0u, source_position_srv },
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
                            std::to_string(model_index) + "_mesh" +
                            std::to_string(gpu_mesh.mesh_index) + "_substep" +
                            std::to_string(substep_index);
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

                        simulation::compute_pass_desc integrate_pass;
                        integrate_pass.name = "soft_body_integrate_m" +
                            std::to_string(model_index) + "_mesh" +
                            std::to_string(gpu_mesh.mesh_index) + "_substep" +
                            std::to_string(substep_index);
                        integrate_pass.shader = soft_body_integrate_shader.Get();
                        integrate_pass.shader_resources = {
                            { 0u, source_position_srv },
                            { 1u, source_velocity_srv },
                            { 2u, gpu_mesh.vertex_force_srv.Get() },
                            { 3u, gpu_mesh.inverse_mass_srv.Get() },
                            { 4u, gpu_mesh.vertex_damping_rate_srv.Get() }
                        };
                        integrate_pass.unordered_access_views = {
                            { 0u, destination_position_uav, 0xffffffffu },
                            { 1u, destination_velocity_uav, 0xffffffffu },
                            { 2u, world.commit_control_uav.Get(), 0xffffffffu }
                        };
                        integrate_pass.constant_buffers = {
                            { 0u, gpu_mesh.integration_parameters.Get() }
                        };
                        integrate_pass.group_count_x = std::max(
                            1u, (gpu_mesh.vertex_count + 63u) / 64u);
                        if (!graph.add_pass(integrate_pass, diagnostic))
                        {
                            return false;
                        }

                        source_is_scratch = destination_is_scratch;
                        if (!destination_is_scratch)
                        {
                            source_bank = destination_bank;
                        }
                    }

                    if (source_is_scratch)
                    {
                        simulation::compute_pass_desc copy_pass;
                        copy_pass.name = "soft_body_publish_working_state_m" +
                            std::to_string(model_index) + "_mesh" +
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
                }
            }
            return true;
        }

        bool append_soft_body_render_candidate_passes(
            world_slot& world,
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

                const transform& root = model.root_transform;
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
            reservation.blocked = false;
            double dt_limit = std::numeric_limits<double>::infinity();
            if (world.description.world_max_substep_delta_time > 0.0)
            {
                dt_limit = std::min(
                    dt_limit, world.description.world_max_substep_delta_time);
            }
            if (world.description.contact_max_substep_delta_time > 0.0)
            {
                dt_limit = std::min(
                    dt_limit, world.description.contact_max_substep_delta_time);
            }

            double soft_body_dt_limit = std::numeric_limits<double>::infinity();
            std::string cfl_diagnostic;
            if (!compute_soft_body_cfl_time_limit(
                    world, soft_body_dt_limit, cfl_diagnostic))
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

            double required_substeps = world.description.minimum_substeps;
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
            // Until R1-R5 are connected, the base integration-segment lower bound
            // is the World Substep count. Animation / Force / Impulse / TOI boundaries
            // are added by the later scheduler integration phases.
            const uint32_t planned_segments = planned_substeps;
            reservation.planned_substeps = planned_substeps;
            reservation.planned_integration_segments = planned_segments;
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

            if (world.next_tick_sequence == 0u ||
                world.next_tick_sequence == std::numeric_limits<uint64_t>::max() ||
                !finite_double(world.reserved_tick_time +
                    world.description.fixed_delta_time))
            {
                world.state = tick_state::failed;
                world.last_tick_status = status_code::capacity_exceeded;
                world.diagnostic = "Tick Reservation Sequence or reserved time cannot advance without overflow.";
                return false;
            }

            tick_reservation reservation;
            reservation.sequence = world.next_tick_sequence;
            reservation.delta_time = world.description.fixed_delta_time;
            reservation.from_single_step = from_single_step;
            reservation.budget_revision = world.execution_budget_revision;

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
            if (world_has_unsupported_physics_work(world))
            {
                world.state = tick_state::pending;
                world.diagnostic =
                    "GPU submission is deferred because this World contains Cloth or Collider work that is not yet connected to the Runtime scheduler.";
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
                        plan.expected_source_state_set,
                        plan.destination_state_set,
                        reservation,
                        graph,
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
        const std::filesystem::path soft_body_fem_path =
            shader_root / "soft_body_fem.cs.hlsl";
        const std::filesystem::path soft_body_gather_path =
            shader_root / "soft_body_force_gather.cs.hlsl";
        const std::filesystem::path soft_body_integrate_path =
            shader_root / "soft_body_integrate.cs.hlsl";
        const std::filesystem::path soft_body_render_feedback_path =
            shader_root / "soft_body_render_feedback.cs.hlsl";

        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_begin_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_validate_empty_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_validate_soft_body_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> tick_commit_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_fem_shader;
        Microsoft::WRL::ComPtr<ID3D11ComputeShader> soft_body_gather_shader;
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
                device, soft_body_fem_path, "soft_body_fem",
                soft_body_fem_shader.GetAddressOf(), diagnostic) ||
            !compile_runtime_compute_shader(
                device, soft_body_gather_path, "soft_body_force_gather",
                soft_body_gather_shader.GetAddressOf(), diagnostic) ||
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
        runtime.soft_body_fem_shader = std::move(soft_body_fem_shader);
        runtime.soft_body_force_gather_shader = std::move(soft_body_gather_shader);
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
            if (!runtime.initialize_soft_body_gpu_mesh(
                    mesh_asset.mesh_index,
                    mesh_asset,
                    desc.source_model.meshes[mesh_asset.mesh_index],
                    desc.world_transform,
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
        model.root_transform = desc.world_transform;
        model.root_samples.clear();
        model.has_animation_playback = false;
        model.has_animation_blend = false;
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
        world->execution_budget = budget;
        world->description.maximum_substeps = budget.maximum_substeps;
        world->description.maximum_integration_segments_per_tick =
            budget.maximum_integration_segments_per_tick;
        ++world->execution_budget_revision;
        if (!world->reservations.empty())
        {
            runtime.update_head_reservation_preflight(*world);
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
        if (force.event_id == 0u ||
            !finite_vec3(force.position) ||
            !finite_vec3(force.direction) ||
            !finite_float(force.magnitude) ||
            force.magnitude < 0.0f ||
            !finite_float(force.radius) ||
            force.radius <= 0.0f ||
            !finite_double(force.duration) ||
            force.duration <= 0.0 ||
            !finite_double(force.start_time) ||
            force.start_time < 0.0 ||
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
        world->forces.push_back(force);
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
        if (impulse.event_id == 0u ||
            !finite_vec3(impulse.position) ||
            !finite_vec3(impulse.direction) ||
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
        world->impulses.push_back(impulse);
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
        if (!model->root_samples.empty() &&
            sample_time < model->root_samples.back().time)
        {
            return make_status(status_code::invalid_argument,
                "Model Transform Samples must be submitted in non-decreasing time order.");
        }
        if (!model->root_samples.empty() &&
            sample_time == model->root_samples.back().time)
        {
            model->root_samples.back() = { sample_time, world_transform, is_teleport };
        }
        else
        {
            model->root_samples.push_back({ sample_time, world_transform, is_teleport });
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
        if (!collider->pose_samples.empty() &&
            sample_time < collider->pose_samples.back().time)
        {
            return make_status(status_code::invalid_argument,
                "Kinematic Pose Samples must be submitted in non-decreasing time order.");
        }
        if (!collider->pose_samples.empty() &&
            sample_time == collider->pose_samples.back().time)
        {
            collider->pose_samples.back() = { sample_time, pose, false };
        }
        else
        {
            collider->pose_samples.push_back({ sample_time, pose, false });
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

            const uint64_t status_dispatch_id = gpu_status.dispatch_id;
            const uint64_t status_reservation_token = gpu_status.reservation_token;
            const bool matching_status_identity =
                status_dispatch_id == dispatch_id &&
                status_reservation_token == completed_tick.reservation_token;
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
                    "GPU Tick Status Dispatch ID / Reservation Token did not match its immutable Submission Record.";
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
                result_world.state = tick_state::committed;
                result_world.last_tick_status = status_code::ok;
                result_world.budget_block_origin = work_budget_block_origin::none;
                result_world.budget_resolution = work_budget_resolution::none;
                result_world.work_budget_block_flags = 0u;
                result_world.executed_integration_segments =
                    gpu_status.executed_integration_segments;
                result_world.minimum_required_integration_segments =
                    gpu_status.minimum_required_integration_segments;
                result_world.diagnostic.clear();
            }
            else
            {
                runtime.release_render_candidate_slot(
                    result_world,
                    completed_tick.render_candidate_slot_id,
                    completed_tick.render_candidate_slot_generation);
                completed_tick.reservation.blocked = true;
                completed_tick.reservation.rejected =
                    gpu_status.terminal_state == 4u ||
                    gpu_status.terminal_state == 5u;
                result_world.reservations.push_front(
                    std::move(completed_tick.reservation));

                result_world.work_budget_block_flags =
                    static_cast<uint8_t>(
                        gpu_status.work_budget_block_flags & 0xffu);
                result_world.budget_block_origin =
                    static_cast<work_budget_block_origin>(
                        std::min(gpu_status.work_budget_block_origin, 2u));
                result_world.executed_integration_segments =
                    gpu_status.executed_integration_segments;
                result_world.minimum_required_integration_segments =
                    gpu_status.minimum_required_integration_segments;

                if (gpu_status.terminal_state == 3u)
                {
                    result_world.state = tick_state::blocked;
                    result_world.last_tick_status = status_code::blocked_by_predecessor;
                    result_world.budget_resolution = work_budget_resolution::retryable;
                }
                else if (gpu_status.terminal_state == 4u)
                {
                    result_world.state = tick_state::rejected;
                    result_world.last_tick_status = status_code::step_rejected;
                    result_world.budget_resolution = work_budget_resolution::none;
                }
                else
                {
                    result_world.state = tick_state::failed;
                    result_world.last_tick_status = status_code::gpu_execution_failed;
                    result_world.budget_resolution = work_budget_resolution::none;
                }
                result_world.diagnostic =
                    "GPU Tick ended without a COMMITTED terminal status; the active State Bank and reservation time were retained.";
            }
        }

        for (implementation::world_slot& result_world : runtime.worlds)
        {
            if (result_world.occupied &&
                result_world.in_flight_tick_count == 0u)
            {
                runtime.synchronize_scheduled_state_with_committed(result_world);
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
