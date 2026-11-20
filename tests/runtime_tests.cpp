#include "skin_physics/runtime/physics_runtime.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"
#include "skin_physics/runtime/detail/force_distribution.hpp"
#include "json_test_support.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <array>
#include <cstring>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <windows.h>
#include <d3d11.h>
#include <wrl/client.h>
#include <d3dcompiler.h>

#pragma comment(lib, "d3dcompiler.lib")

namespace
{
    using Microsoft::WRL::ComPtr;
    using skin_physics::runtime::physics_runtime;
    using skin_physics::runtime::status_code;

    void require(bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    std::string runtime_shader_root()
    {
        std::vector<wchar_t> executable_path(32768u, L'\0');
        const DWORD length = GetModuleFileNameW(
            nullptr,
            executable_path.data(),
            static_cast<DWORD>(executable_path.size()));
        if (length == 0u || length >= executable_path.size())
        {
            return {};
        }
        executable_path.resize(length);

        const std::filesystem::path executable(
            executable_path.begin(), executable_path.end());
        const std::filesystem::path repository_root =
            executable.parent_path().parent_path().parent_path().parent_path();
        return (repository_root / "shaders" / "runtime").string();
    }

    struct alignas(16) test_gpu_float4
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 0.0f;
    };

    static_assert(sizeof(test_gpu_float4) == 16,
        "Runtime Render SRV test element must remain 16 bytes.");

    template <typename T>
    std::vector<T> readback_render_srv(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ShaderResourceView* source_view,
        std::size_t element_count)
    {
        require(device != nullptr && context != nullptr && source_view != nullptr,
            "Render SRV readback requires a Device, Immediate Context and SRV.");
        require(element_count > 0u &&
            element_count <= static_cast<std::size_t>(
                std::numeric_limits<UINT>::max() / sizeof(T)),
            "Render SRV readback element count exceeds the D3D11 Buffer range.");

        ComPtr<ID3D11Resource> source_resource;
        source_view->GetResource(source_resource.GetAddressOf());
        require(source_resource != nullptr,
            "Render SRV must reference an allocated D3D11 Resource.");

        ComPtr<ID3D11Buffer> source_buffer;
        require(SUCCEEDED(source_resource.As(&source_buffer)) &&
            source_buffer != nullptr,
            "Render SRV resource must be a structured D3D11 Buffer.");

        D3D11_BUFFER_DESC source_description = {};
        source_buffer->GetDesc(&source_description);
        const UINT expected_byte_width = static_cast<UINT>(
            element_count * sizeof(T));
        require(source_description.ByteWidth == expected_byte_width,
            "Render SRV Buffer ByteWidth must match the requested typed stream.");

        D3D11_BUFFER_DESC staging_description = {};
        staging_description.ByteWidth = source_description.ByteWidth;
        staging_description.Usage = D3D11_USAGE_STAGING;
        staging_description.CPUAccessFlags = D3D11_CPU_ACCESS_READ;

        ComPtr<ID3D11Buffer> staging_buffer;
        require(SUCCEEDED(device->CreateBuffer(
                &staging_description, nullptr, staging_buffer.GetAddressOf())),
            "Render SRV staging Buffer creation must succeed.");
        context->CopyResource(staging_buffer.Get(), source_buffer.Get());

        D3D11_MAPPED_SUBRESOURCE mapped = {};
        require(SUCCEEDED(context->Map(
                staging_buffer.Get(), 0u, D3D11_MAP_READ, 0u, &mapped)),
            "Render SRV staging Buffer must be mappable for CPU readback.");

        std::vector<T> result(element_count);
        std::memcpy(result.data(), mapped.pData, expected_byte_width);
        context->Unmap(staging_buffer.Get(), 0u);
        return result;
    }

    bool create_warp_device(
        ComPtr<ID3D11Device>& device,
        ComPtr<ID3D11DeviceContext>& context)
    {
        const D3D_FEATURE_LEVEL requested_levels[] = {
            D3D_FEATURE_LEVEL_11_0
        };
        D3D_FEATURE_LEVEL selected_level = D3D_FEATURE_LEVEL_9_1;
        const HRESULT result = D3D11CreateDevice(
            nullptr,
            D3D_DRIVER_TYPE_WARP,
            nullptr,
            0,
            requested_levels,
            1,
            D3D11_SDK_VERSION,
            device.GetAddressOf(),
            &selected_level,
            context.GetAddressOf());
        return SUCCEEDED(result) &&
            selected_level >= D3D_FEATURE_LEVEL_11_0;
    }


    struct alignas(16) test_gpu_force_distribution_event_parameters
    {
        uint32_t vertex_count = 0u;
        uint32_t include_previous = 0u;
        uint32_t reserved0[2] = {};
        float action_position[3] = {};
        float radius = 0.0f;
        float direction[3] = {};
        float magnitude = 0.0f;
    };

    struct alignas(16) test_gpu_distribution_reduce_parameters
    {
        uint32_t vertex_count = 0u;
        uint32_t group_count = 0u;
        uint32_t reserved[2] = {};
    };

    struct alignas(16) test_gpu_tick_commit_control
    {
        uint32_t values[20] = {};
    };

    static_assert(sizeof(test_gpu_force_distribution_event_parameters) == 48,
        "Force distribution Event Parameters must match the production HLSL ABI.");
    static_assert(sizeof(test_gpu_distribution_reduce_parameters) == 16,
        "Force distribution Reduction Parameters must match the production HLSL ABI.");
    static_assert(sizeof(test_gpu_tick_commit_control) == 80,
        "Force distribution Tick Commit Control must match the production HLSL ABI.");

    template <typename T>
    ComPtr<ID3D11Buffer> create_distribution_test_structured_buffer(
        ID3D11Device* device,
        const std::vector<T>& values,
        UINT bind_flags)
    {
        require(device != nullptr && !values.empty() &&
            values.size() <= static_cast<std::size_t>(
                std::numeric_limits<UINT>::max() / sizeof(T)),
            "Force distribution test Buffer input must fit a non-empty D3D11 allocation.");

        D3D11_BUFFER_DESC description = {};
        description.ByteWidth = static_cast<UINT>(values.size() * sizeof(T));
        description.Usage = D3D11_USAGE_DEFAULT;
        description.BindFlags = bind_flags;
        description.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
        description.StructureByteStride = static_cast<UINT>(sizeof(T));

        D3D11_SUBRESOURCE_DATA initial_data = {};
        initial_data.pSysMem = values.data();

        ComPtr<ID3D11Buffer> buffer;
        require(SUCCEEDED(device->CreateBuffer(
                &description, &initial_data, buffer.GetAddressOf())),
            "Force distribution test Structured Buffer creation must succeed.");
        return buffer;
    }

    template <typename T>
    ComPtr<ID3D11Buffer> create_distribution_test_constant_buffer(
        ID3D11Device* device,
        const T& value)
    {
        static_assert((sizeof(T) & 15u) == 0u,
            "Force distribution constant buffers must be 16-byte aligned.");
        require(device != nullptr &&
            sizeof(T) <= D3D11_REQ_CONSTANT_BUFFER_ELEMENT_COUNT * 16u,
            "Force distribution test Constant Buffer must fit the D3D11 limit.");

        D3D11_BUFFER_DESC description = {};
        description.ByteWidth = static_cast<UINT>(sizeof(T));
        description.Usage = D3D11_USAGE_DEFAULT;
        description.BindFlags = D3D11_BIND_CONSTANT_BUFFER;

        D3D11_SUBRESOURCE_DATA initial_data = {};
        initial_data.pSysMem = &value;

        ComPtr<ID3D11Buffer> buffer;
        require(SUCCEEDED(device->CreateBuffer(
                &description, &initial_data, buffer.GetAddressOf())),
            "Force distribution test Constant Buffer creation must succeed.");
        return buffer;
    }

    ComPtr<ID3D11ShaderResourceView> create_distribution_test_srv(
        ID3D11Device* device,
        ID3D11Buffer* buffer,
        uint32_t element_count)
    {
        require(device != nullptr && buffer != nullptr && element_count > 0u,
            "Force distribution test SRV requires a valid Buffer and element count.");

        D3D11_SHADER_RESOURCE_VIEW_DESC description = {};
        description.Format = DXGI_FORMAT_UNKNOWN;
        description.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
        description.Buffer.FirstElement = 0u;
        description.Buffer.NumElements = element_count;

        ComPtr<ID3D11ShaderResourceView> view;
        require(SUCCEEDED(device->CreateShaderResourceView(
                buffer, &description, view.GetAddressOf())),
            "Force distribution test SRV creation must succeed.");
        return view;
    }

    ComPtr<ID3D11UnorderedAccessView> create_distribution_test_uav(
        ID3D11Device* device,
        ID3D11Buffer* buffer,
        uint32_t element_count)
    {
        require(device != nullptr && buffer != nullptr && element_count > 0u,
            "Force distribution test UAV requires a valid Buffer and element count.");

        D3D11_UNORDERED_ACCESS_VIEW_DESC description = {};
        description.Format = DXGI_FORMAT_UNKNOWN;
        description.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
        description.Buffer.FirstElement = 0u;
        description.Buffer.NumElements = element_count;

        ComPtr<ID3D11UnorderedAccessView> view;
        require(SUCCEEDED(device->CreateUnorderedAccessView(
                buffer, &description, view.GetAddressOf())),
            "Force distribution test UAV creation must succeed.");
        return view;
    }

    void test_runtime_force_distribution_gpu_cpu_parity()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Force distribution parity regression requires a D3D11 WARP Feature Level 11_0 device.");

        const std::filesystem::path shader_path =
            std::filesystem::path(runtime_shader_root()) /
            "soft_body_force_distribution.cs.hlsl";
        require(std::filesystem::is_regular_file(shader_path),
            "Force distribution parity regression must compile the production Runtime shader.");

        auto compile_shader = [&](const char* entry_point,
                                  ComPtr<ID3D11ComputeShader>& out_shader)
        {
            ComPtr<ID3DBlob> bytecode;
            ComPtr<ID3DBlob> errors;
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

            std::string diagnostic;
            if (errors != nullptr && errors->GetBufferSize() > 0u)
            {
                diagnostic.assign(
                    static_cast<const char*>(errors->GetBufferPointer()),
                    errors->GetBufferSize());
            }
            require(SUCCEEDED(compile_result),
                std::string("Production Force distribution shader entry point must compile: ") +
                    entry_point + " " + diagnostic);
            require(SUCCEEDED(device->CreateComputeShader(
                    bytecode->GetBufferPointer(),
                    bytecode->GetBufferSize(),
                    nullptr,
                    out_shader.GetAddressOf())),
                std::string("Production Force distribution compute shader creation must succeed: ") +
                    entry_point);
        };

        ComPtr<ID3D11ComputeShader> weight_shader;
        ComPtr<ID3D11ComputeShader> reduce_shader;
        ComPtr<ID3D11ComputeShader> finalize_shader;
        ComPtr<ID3D11ComputeShader> apply_shader;
        compile_shader("soft_body_force_distribution_weight", weight_shader);
        compile_shader("soft_body_force_distribution_reduce", reduce_shader);
        compile_shader("soft_body_force_distribution_finalize", finalize_shader);
        compile_shader("soft_body_force_distribution_apply", apply_shader);

        constexpr uint32_t vertex_count = 130u;
        constexpr uint32_t thread_group_size = 64u;
        constexpr uint32_t group_count =
            (vertex_count + thread_group_size - 1u) / thread_group_size;
        constexpr UINT data_bind_flags =
            D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;

        const test_json::value input = test_json::load(
            test_support::fixture_path(
                "phase13_force_distribution/warp_130.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path(
                "phase13_force_distribution/warp_130.expected.json"));
        const test_json::value::array& position_inputs =
            input.at("positions").as_array();
        const test_json::value::array& mass_inputs =
            input.at("rest_lumped_masses").as_array();
        require(position_inputs.size() == vertex_count &&
            mass_inputs.size() == vertex_count,
            "WARP fixture must contain exactly 130 Positions and Rest Lumped Mass values.");

        std::vector<skin_physics::asset::vec3> positions;
        std::vector<float> rest_masses;
        std::vector<test_gpu_float4> gpu_positions;
        positions.reserve(vertex_count);
        rest_masses.reserve(vertex_count);
        gpu_positions.reserve(vertex_count);

        for (uint32_t index = 0u; index < vertex_count; ++index)
        {
            const test_json::value::array& position_values =
                position_inputs[index].as_array();
            require(position_values.size() == 3u,
                "Each WARP fixture Position must contain three coordinates.");

            const skin_physics::asset::vec3 position = {
                static_cast<float>(position_values[0u].as_number()),
                static_cast<float>(position_values[1u].as_number()),
                static_cast<float>(position_values[2u].as_number())
            };
            const float rest_mass =
                static_cast<float>(mass_inputs[index].as_number());

            positions.push_back(position);
            rest_masses.push_back(rest_mass);
            gpu_positions.push_back({
                position.x, position.y, position.z, 1.0f
            });
        }

        const test_json::value::array& action_values =
            input.at("action_position").as_array();
        require(action_values.size() == 3u,
            "WARP fixture Action Position must contain three coordinates.");
        const skin_physics::asset::vec3 action_position = {
            static_cast<float>(action_values[0u].as_number()),
            static_cast<float>(action_values[1u].as_number()),
            static_cast<float>(action_values[2u].as_number())
        };
        const float radius = static_cast<float>(
            input.at("radius").as_number());

        // Keep the CPU inputs as distinct Mesh views while packing the same
        // deterministic Mesh order into the contiguous GPU buffers below. This
        // verifies one shared target-scope denominator across Mesh boundaries.
        const test_json::value::array& partition_inputs =
            input.at("mesh_partitions").as_array();
        require(partition_inputs.size() == 3u,
            "WARP fixture must describe three independent target Meshes.");

        std::vector<std::vector<skin_physics::asset::vec3>> mesh_positions(
            partition_inputs.size());
        std::vector<std::vector<float>> mesh_rest_masses(
            partition_inputs.size());
        std::vector<skin_physics::runtime::detail::force_distribution_mesh_view>
            mesh_views;
        std::vector<uint32_t> vertex_mesh_indices(vertex_count, 0u);
        std::vector<uint32_t> vertex_mesh_local_indices(vertex_count, 0u);
        mesh_views.reserve(partition_inputs.size());

        uint32_t partition_offset = 0u;
        for (uint32_t mesh_index = 0u;
            mesh_index < static_cast<uint32_t>(partition_inputs.size());
            ++mesh_index)
        {
            const double partition_size_value =
                partition_inputs[mesh_index].as_number();
            require(std::isfinite(partition_size_value) &&
                partition_size_value > 0.0 &&
                std::floor(partition_size_value) == partition_size_value &&
                partition_size_value <=
                    static_cast<double>(vertex_count - partition_offset),
                "Every WARP Mesh partition must have a positive integral vertex count within the input range.");

            const uint32_t partition_size =
                static_cast<uint32_t>(partition_size_value);
            mesh_positions[mesh_index].assign(
                positions.begin() + partition_offset,
                positions.begin() + partition_offset + partition_size);
            mesh_rest_masses[mesh_index].assign(
                rest_masses.begin() + partition_offset,
                rest_masses.begin() + partition_offset + partition_size);

            skin_physics::runtime::detail::force_distribution_mesh_view mesh_view;
            mesh_view.segment_start_positions = &mesh_positions[mesh_index];
            mesh_view.rest_lumped_masses = &mesh_rest_masses[mesh_index];
            mesh_views.push_back(mesh_view);

            for (uint32_t local_index = 0u;
                local_index < partition_size; ++local_index)
            {
                const uint32_t global_index = partition_offset + local_index;
                vertex_mesh_indices[global_index] = mesh_index;
                vertex_mesh_local_indices[global_index] = local_index;
            }
            partition_offset += partition_size;
        }
        require(partition_offset == vertex_count,
            "WARP Mesh partitions must cover every flattened GPU input exactly once.");

        skin_physics::runtime::detail::force_distribution_weights cpu_reference;
        std::string reference_diagnostic;
        require(
            skin_physics::runtime::detail::build_mass_weighted_wendland_c2_weights(
                action_position,
                radius,
                mesh_views,
                cpu_reference,
                reference_diagnostic),
            "CPU double-precision Wendland C2 reference must build: " +
                reference_diagnostic);
        require(cpu_reference.mesh_weights.size() == partition_inputs.size(),
            "CPU reference must preserve the three target Mesh weight arrays.");
        for (uint32_t mesh_index = 0u;
            mesh_index < static_cast<uint32_t>(partition_inputs.size());
            ++mesh_index)
        {
            require(cpu_reference.mesh_weights[mesh_index].size() ==
                mesh_positions[mesh_index].size(),
                "CPU reference must preserve each target Mesh vertex count.");
        }

        const double expected_weight_sum =
            expected.at("unnormalized_weight_sum").as_number();
        const test_json::value::array& expected_weights =
            expected.at("weights").as_array();
        require(expected_weights.size() == vertex_count &&
            std::abs(cpu_reference.unnormalized_weight_sum -
                expected_weight_sum) <= 1.0e-10,
            "CPU double-precision Force distribution must match the persistent 130-vertex / three-Mesh golden.");
        std::vector<double> per_mesh_weight_sums(mesh_views.size(), 0.0);
        for (uint32_t index = 0u; index < vertex_count; ++index)
        {
            const uint32_t mesh_index = vertex_mesh_indices[index];
            const uint32_t local_index = vertex_mesh_local_indices[index];
            const double weight =
                cpu_reference.mesh_weights[mesh_index][local_index];
            require(std::abs(weight - expected_weights[index].as_number()) <= 1.0e-12,
                "CPU double-precision per-vertex Force weights must match the persistent golden across Mesh boundaries.");
            per_mesh_weight_sums[mesh_index] += weight;
        }

        double cpu_normalized_weight_sum = 0.0;
        for (double mesh_sum : per_mesh_weight_sums)
        {
            require(mesh_sum > 0.0 && mesh_sum < 1.0,
                "Every fixture Mesh must receive a partial share of the single global normalization denominator.");
            cpu_normalized_weight_sum += mesh_sum;
        }
        require(std::abs(cpu_normalized_weight_sum - 1.0) <= 1.0e-12,
            "CPU Force distribution must conserve one descriptor magnitude across all target Meshes.");

        // A valid multi-Mesh result must remain intact when a later request fails.
        skin_physics::runtime::detail::force_distribution_weights unchanged_output =
            cpu_reference;
        std::vector<skin_physics::asset::vec3> no_support_positions;
        std::vector<float> no_support_masses;
        skin_physics::runtime::detail::force_distribution_mesh_view no_support_mesh;
        no_support_mesh.segment_start_positions = &no_support_positions;
        no_support_mesh.rest_lumped_masses = &no_support_masses;
        const std::vector<skin_physics::runtime::detail::force_distribution_mesh_view>
            no_support_mesh_views = { no_support_mesh };
        std::string no_support_diagnostic;
        require(
            !skin_physics::runtime::detail::build_mass_weighted_wendland_c2_weights(
                action_position,
                radius,
                no_support_mesh_views,
                unchanged_output,
                no_support_diagnostic),
            "A target scope with no vertices must fail with NO_FORCE_TARGET.");
        require(no_support_diagnostic.find("NO_FORCE_TARGET") != std::string::npos &&
            unchanged_output.unnormalized_weight_sum ==
                cpu_reference.unnormalized_weight_sum &&
            unchanged_output.mesh_weights == cpu_reference.mesh_weights,
            "A no-support failure must preserve the previous multi-Mesh output without partial mutation.");

        test_gpu_force_distribution_event_parameters event_parameters;
        event_parameters.vertex_count = vertex_count;
        event_parameters.action_position[0] = action_position.x;
        event_parameters.action_position[1] = action_position.y;
        event_parameters.action_position[2] = action_position.z;
        event_parameters.radius = radius;
        event_parameters.direction[0] = 1.0f;
        event_parameters.magnitude = 1.0f;

        test_gpu_distribution_reduce_parameters reduce_parameters;
        reduce_parameters.vertex_count = vertex_count;
        reduce_parameters.group_count = group_count;

        test_gpu_tick_commit_control control_values;
        control_values.values[2] = 1u;
        control_values.values[5] = 1u;
        control_values.values[13] = 1u;
        control_values.values[15] = 0xffffffffu;

        const std::vector<float> zero_weights(vertex_count, 0.0f);
        const std::vector<float> zero_sum(1u, 0.0f);
        const std::vector<test_gpu_float4> zero_accumulator(
            vertex_count, test_gpu_float4{});
        const std::vector<test_gpu_tick_commit_control> initial_control = {
            control_values
        };

        ComPtr<ID3D11Buffer> position_buffer =
            create_distribution_test_structured_buffer(
                device.Get(), gpu_positions, D3D11_BIND_SHADER_RESOURCE);
        ComPtr<ID3D11Buffer> mass_buffer =
            create_distribution_test_structured_buffer(
                device.Get(), rest_masses, D3D11_BIND_SHADER_RESOURCE);
        ComPtr<ID3D11Buffer> raw_weight_buffer =
            create_distribution_test_structured_buffer(
                device.Get(), zero_weights, data_bind_flags);
        ComPtr<ID3D11Buffer> partial_sum_buffer =
            create_distribution_test_structured_buffer(
                device.Get(),
                std::vector<float>(group_count, 0.0f),
                data_bind_flags);
        ComPtr<ID3D11Buffer> normalized_sum_buffer =
            create_distribution_test_structured_buffer(
                device.Get(), zero_sum, data_bind_flags);
        ComPtr<ID3D11Buffer> accumulator_buffer =
            create_distribution_test_structured_buffer(
                device.Get(), zero_accumulator, data_bind_flags);
        ComPtr<ID3D11Buffer> control_buffer =
            create_distribution_test_structured_buffer(
                device.Get(), initial_control, data_bind_flags);
        ComPtr<ID3D11Buffer> event_buffer =
            create_distribution_test_constant_buffer(
                device.Get(), event_parameters);
        ComPtr<ID3D11Buffer> reduce_buffer =
            create_distribution_test_constant_buffer(
                device.Get(), reduce_parameters);

        ComPtr<ID3D11ShaderResourceView> position_srv =
            create_distribution_test_srv(device.Get(), position_buffer.Get(), vertex_count);
        ComPtr<ID3D11ShaderResourceView> mass_srv =
            create_distribution_test_srv(device.Get(), mass_buffer.Get(), vertex_count);
        ComPtr<ID3D11ShaderResourceView> raw_weight_srv =
            create_distribution_test_srv(device.Get(), raw_weight_buffer.Get(), vertex_count);
        ComPtr<ID3D11ShaderResourceView> partial_sum_srv =
            create_distribution_test_srv(device.Get(), partial_sum_buffer.Get(), group_count);
        ComPtr<ID3D11ShaderResourceView> normalized_sum_srv =
            create_distribution_test_srv(device.Get(), normalized_sum_buffer.Get(), 1u);
        ComPtr<ID3D11ShaderResourceView> accumulator_srv =
            create_distribution_test_srv(device.Get(), accumulator_buffer.Get(), vertex_count);
        ComPtr<ID3D11ShaderResourceView> control_srv =
            create_distribution_test_srv(device.Get(), control_buffer.Get(), 1u);

        ComPtr<ID3D11UnorderedAccessView> raw_weight_uav =
            create_distribution_test_uav(device.Get(), raw_weight_buffer.Get(), vertex_count);
        ComPtr<ID3D11UnorderedAccessView> partial_sum_uav =
            create_distribution_test_uav(device.Get(), partial_sum_buffer.Get(), group_count);
        ComPtr<ID3D11UnorderedAccessView> normalized_sum_uav =
            create_distribution_test_uav(device.Get(), normalized_sum_buffer.Get(), 1u);
        ComPtr<ID3D11UnorderedAccessView> accumulator_uav =
            create_distribution_test_uav(device.Get(), accumulator_buffer.Get(), vertex_count);
        ComPtr<ID3D11UnorderedAccessView> control_uav =
            create_distribution_test_uav(device.Get(), control_buffer.Get(), 1u);

        ID3D11Buffer* constant_buffers[] = {
            event_buffer.Get(), reduce_buffer.Get()
        };
        auto unbind_compute_resources = [&]()
        {
            ID3D11ShaderResourceView* null_srvs[6] = {};
            ID3D11UnorderedAccessView* null_uavs[5] = {};
            ID3D11Buffer* null_constant_buffers[2] = {};
            context->CSSetShaderResources(0u, 6u, null_srvs);
            context->CSSetUnorderedAccessViews(0u, 5u, null_uavs, nullptr);
            context->CSSetConstantBuffers(0u, 2u, null_constant_buffers);
            context->CSSetShader(nullptr, nullptr, 0u);
        };

        auto dispatch_distribution_pass = [&](
            ID3D11ComputeShader* shader,
            const std::array<ID3D11ShaderResourceView*, 6>& srvs,
            const std::array<ID3D11UnorderedAccessView*, 5>& uavs,
            uint32_t dispatch_count)
        {
            context->CSSetShader(shader, nullptr, 0u);
            context->CSSetShaderResources(
                0u, static_cast<UINT>(srvs.size()), srvs.data());
            context->CSSetUnorderedAccessViews(
                0u, static_cast<UINT>(uavs.size()), uavs.data(), nullptr);
            context->CSSetConstantBuffers(0u, 2u, constant_buffers);
            context->Dispatch(dispatch_count, 1u, 1u);
            unbind_compute_resources();
        };

        std::array<ID3D11ShaderResourceView*, 6> srvs = {};
        std::array<ID3D11UnorderedAccessView*, 5> uavs = {};

        srvs[0] = position_srv.Get();
        srvs[1] = mass_srv.Get();
        uavs[0] = raw_weight_uav.Get();
        uavs[1] = control_uav.Get();
        dispatch_distribution_pass(weight_shader.Get(), srvs, uavs, group_count);

        srvs = {};
        uavs = {};
        srvs[2] = raw_weight_srv.Get();
        uavs[1] = control_uav.Get();
        uavs[2] = partial_sum_uav.Get();
        dispatch_distribution_pass(reduce_shader.Get(), srvs, uavs, group_count);

        srvs = {};
        uavs = {};
        srvs[2] = partial_sum_srv.Get();
        uavs[1] = control_uav.Get();
        uavs[3] = normalized_sum_uav.Get();
        dispatch_distribution_pass(finalize_shader.Get(), srvs, uavs, 1u);

        srvs = {};
        uavs = {};
        srvs[3] = raw_weight_srv.Get();
        srvs[4] = normalized_sum_srv.Get();
        uavs[1] = control_uav.Get();
        uavs[4] = accumulator_uav.Get();
        dispatch_distribution_pass(apply_shader.Get(), srvs, uavs, group_count);

        context->Flush();

        const std::vector<float> gpu_raw_weights =
            readback_render_srv<float>(
                device.Get(), context.Get(), raw_weight_srv.Get(), vertex_count);
        const std::vector<float> gpu_normalized_sum =
            readback_render_srv<float>(
                device.Get(), context.Get(), normalized_sum_srv.Get(), 1u);
        const std::vector<test_gpu_float4> gpu_distributed_weights =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), accumulator_srv.Get(), vertex_count);
        const std::vector<test_gpu_tick_commit_control> gpu_control =
            readback_render_srv<test_gpu_tick_commit_control>(
                device.Get(), context.Get(), control_srv.Get(), 1u);

        require(gpu_raw_weights.size() == vertex_count &&
            gpu_distributed_weights.size() == vertex_count &&
            gpu_normalized_sum.size() == 1u &&
            gpu_control.size() == 1u,
            "GPU Force distribution readback must preserve all per-vertex outputs and the scalar normalization sum.");
        require(gpu_control[0u].values[5] == 1u &&
            gpu_control[0u].values[6] == 0u &&
            gpu_control[0u].values[14] == 0u,
            "GPU Force distribution Weight / Reduction / Finalize / Apply stages must preserve the In-Flight Commit Control without terminal failure.");

        const double gpu_sum_error = std::abs(
            static_cast<double>(gpu_normalized_sum[0u]) -
            cpu_reference.unnormalized_weight_sum);
        const double gpu_sum_tolerance = std::max(
            2.0e-5,
            cpu_reference.unnormalized_weight_sum * 2.0e-6);
        require(gpu_sum_error <= gpu_sum_tolerance,
            "GPU grouped reduction normalization sum must match the CPU double-precision reference.");

        double normalized_weight_sum = 0.0;
        for (uint32_t index = 0u; index < vertex_count; ++index)
        {
            const uint32_t mesh_index = vertex_mesh_indices[index];
            const uint32_t local_index = vertex_mesh_local_indices[index];
            const double expected_q =
                cpu_reference.mesh_weights[mesh_index][local_index] *
                cpu_reference.unnormalized_weight_sum;
            const double q_error = std::abs(
                static_cast<double>(gpu_raw_weights[index]) - expected_q);
            const double q_tolerance = std::max(
                1.0e-6, std::abs(expected_q) * 5.0e-5);
            require(q_error <= q_tolerance,
                "GPU per-vertex unnormalized Wendland C2 q must match the CPU reference.");

            const double expected_weight =
                cpu_reference.mesh_weights[mesh_index][local_index];
            const double normalized_error = std::abs(
                static_cast<double>(gpu_distributed_weights[index].x) -
                expected_weight);
            require(normalized_error <= 2.0e-6,
                "GPU per-vertex normalized Force weight must match the CPU double-precision reference.");
            require(std::abs(gpu_distributed_weights[index].y) <= 1.0e-7f &&
                std::abs(gpu_distributed_weights[index].z) <= 1.0e-7f &&
                std::abs(gpu_distributed_weights[index].w) <= 1.0e-7f,
                "The GPU distribution Apply stage must preserve the authored direction in the output accumulator and keep the unused component zero.");

            normalized_weight_sum += gpu_distributed_weights[index].x;
        }

        require(std::abs(normalized_weight_sum - 1.0) <= 2.0e-5,
            "GPU normalized per-vertex Force weights must conserve unit descriptor magnitude.");
    }

    void test_runtime_initialization_and_world_handles()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Runtime regression requires a D3D11 WARP Feature Level 11_0 device.");

        physics_runtime runtime;
        skin_physics::runtime::runtime_desc runtime_desc;
        runtime_desc.shader_package_root = runtime_shader_root();
        require(!runtime_desc.shader_package_root.empty() &&
            std::filesystem::is_regular_file(
                std::filesystem::path(runtime_desc.shader_package_root) /
                    "tick_transaction_control.cs.hlsl"),
            "Runtime shader package root must resolve to the repository Tick Control shader.");
        runtime_desc.query_ring_capacity = 2u;
        const skin_physics::runtime::status initialize_status =
            runtime.initialize(device.Get(), context.Get(), runtime_desc);
        require(initialize_status.success(),
            std::string(
                "Runtime must accept a valid Host-owned WARP Device and Immediate Context. diagnostic=") +
                initialize_status.diagnostic +
                " native_code=" +
                std::to_string(initialize_status.native_code));
        require(runtime.initialized(),
            "Runtime initialized state must be observable.");

        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).code ==
            status_code::invalid_state,
            "Runtime must reject a second initialize call.");

        skin_physics::runtime::world_desc first_desc;
        first_desc.fixed_delta_time = 0.01;
        first_desc.maximum_in_flight_ticks = 2u;
        skin_physics::runtime::world_handle first_world;
        require(runtime.create_world(first_desc, &first_world).success(),
            "Runtime must create a World with valid time and admission limits.");

        skin_physics::runtime::world_desc second_desc;
        second_desc.maximum_in_flight_ticks = 4u;
        skin_physics::runtime::world_handle second_world;
        require(runtime.create_world(second_desc, &second_world).success(),
            "Runtime must grow the Status Query Ring when the aggregate World in-flight capacity grows.");

        skin_physics::runtime::world_desc invalid_desc = first_desc;
        invalid_desc.maximum_substeps =
            runtime_desc.maximum_world_substeps_hard_limit + 1u;
        skin_physics::runtime::world_handle invalid_world;
        require(runtime.create_world(invalid_desc, &invalid_world).code ==
            status_code::invalid_argument,
            "World creation must reject an Admission Limit above the Runtime Hard Limit.");

        require(runtime.destroy_world(first_world).success(),
            "An idle World must be destroyable.");
        require(runtime.poll(first_world, nullptr).code == status_code::stale_handle,
            "Destroyed World Handles must be rejected as stale.");

        require(runtime.destroy_world(second_world).success(),
            "The second idle World must be destroyable.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime shutdown must release the Status Ring and Host COM references after all Worlds are destroyed.");
    }

    void test_runtime_time_budget_and_pause_contract()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Runtime regression requires a D3D11 WARP Feature Level 11_0 device.");

        physics_runtime runtime;
        skin_physics::runtime::runtime_desc runtime_desc;
        runtime_desc.shader_package_root = runtime_shader_root();
        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).success(),
            "Runtime initialization must succeed for the time / budget regression.");

        skin_physics::runtime::world_desc desc;
        desc.fixed_delta_time = 0.01;
        desc.maximum_steps_per_advance = 2u;
        skin_physics::runtime::world_handle world;
        require(runtime.create_world(desc, &world).success(),
            "Time regression World creation must succeed.");

        skin_physics::runtime::world_execution_budget budget;
        budget.maximum_substeps = 64u;
        budget.maximum_integration_segments_per_tick = 256u;
        require(runtime.set_world_execution_budget(world, budget).success(),
            "World Execution Budget must accept values below the Runtime Hard Limits.");

        skin_physics::runtime::world_status world_status;
        require(runtime.poll(world, &world_status).success() &&
            world_status.execution_budget_revision == 2u,
            "Each accepted Execution Budget update must advance its revision.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "World must enter Playing.");
        skin_physics::runtime::advance_report report;
        require(runtime.advance(world, 0.025, &report).success(),
            "Advance must accept valid wall-clock time.");
        require(report.pending_tick_count == 0u &&
            report.submitted_tick_count == 2u &&
            report.in_flight_tick_count == 2u &&
            std::abs(report.accumulator_seconds - 0.005) <= 1.0e-12,
            "Advance must submit two ordered GPU Ticks up to the World in-flight limit and preserve the fractional remainder.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::paused).success(),
            "World must enter Paused.");
        require(runtime.advance(world, 0.5, &report).success(),
            "Advance while Paused must still allow polling without consuming wall-clock time.");
        require(report.pending_tick_count == 0u &&
            report.in_flight_tick_count == 2u &&
            std::abs(report.accumulator_seconds - 0.005) <= 1.0e-12,
            "Paused Advance must preserve the unallocated remainder and keep both in-flight GPU Ticks unchanged.");

        require(runtime.set_fixed_delta_time(world, 0.02).code == status_code::busy,
            "Fixed Delta Time must not change while complete Tick requests remain in the Accumulator.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::stopped).success(),
            "World must enter Stopped without discarding the Accumulator.");
        bool status_observed = false;
        for (uint32_t attempt = 0u; attempt < 1024u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &world_status).success(),
                "Non-blocking Tick Status Poll must succeed.");
            if (world_status.in_flight_tick_count == 0u)
            {
                status_observed = true;
                break;
            }
            std::this_thread::yield();
        }
        require(status_observed &&
            world_status.state == skin_physics::runtime::tick_state::committed &&
            world_status.gpu_committed_sequence == 2u &&
            world_status.cpu_observed_sequence == 2u &&
            std::abs(world_status.gpu_committed_simulation_time - 0.02) <= 1.0e-12 &&
            std::abs(world_status.cpu_observed_completed_time - 0.02) <= 1.0e-12 &&
            std::abs(world_status.requested_simulation_time - 0.025) <= 1.0e-12 &&
            std::abs(world_status.effective_requested_time - 0.025) <= 1.0e-12 &&
            std::abs(world_status.reserved_tick_time) <= 1.0e-12,
            "GPU Commit and CPU-observed completion must advance only after the Event Query and COMMITTED Status are observed.");

        require(runtime.reset_world(world).success(),
            "Idle World Reset must explicitly discard pending requested time.");
        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::paused).success(),
            "World must be Paused before changing Fixed Delta Time.");
        require(runtime.set_fixed_delta_time(world, 0.02).success(),
            "Fixed Delta Time can change while Paused after whole pending Ticks are cleared.");

        require(runtime.single_step(world, &report).success() &&
            report.pending_tick_count == 1u,
            "Single Step must enqueue exactly one fixed-step request while Paused.");
        skin_physics::runtime::world_execution_budget pending_step_budget = budget;
        pending_step_budget.maximum_substeps = 32u;
        require(runtime.set_world_execution_budget(
                world, pending_step_budget).code == status_code::busy,
            "An unreserved Single Step request must prevent an otherwise idle Execution Budget update.");
        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::stopped).success(),
            "Stop must cancel an unreserved single-step request.");
        require(runtime.poll(world, &world_status).success() &&
            world_status.pending_tick_count == 0u &&
            std::abs(world_status.cancelled_requested_time - 0.02) <= 1.0e-12 &&
            std::abs(world_status.effective_requested_time) <= 1.0e-12,
            "Cancelled Step time must be tracked separately and excluded from effective requested time.");

        require(runtime.reset_world(world).success(),
            "World must reset after the Step cancellation regression.");
        require(runtime.destroy_world(world).success(),
            "World with no pending time or lease must be destroyable.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the time / budget regression.");
    }

    void test_runtime_cpu_preflight_reservation_retry()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Runtime regression requires a D3D11 WARP Feature Level 11_0 device.");

        physics_runtime runtime;
        skin_physics::runtime::runtime_desc runtime_desc;
        runtime_desc.shader_package_root = runtime_shader_root();
        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).success(),
            "Runtime initialization must succeed for the CPU Preflight regression.");

        skin_physics::runtime::world_desc desc;
        desc.fixed_delta_time = 0.02;
        desc.minimum_substeps = 1u;
        desc.maximum_substeps = 2u;
        desc.contact_max_substep_delta_time = 0.005;
        skin_physics::runtime::world_handle world;
        require(runtime.create_world(desc, &world).success(),
            "CPU Preflight World creation must succeed.");
        skin_physics::runtime::world_handle other_world;
        require(runtime.create_world(desc, &other_world).success(),
            "A second World is required to verify that shared Query Ring polling does not observe another World's CPU-only Status.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "CPU Preflight World must enter Playing.");

        skin_physics::runtime::advance_report report;
        require(runtime.advance(world, 0.02, &report).success(),
            "Advance must accept time even when the derived CPU Preflight later blocks.");
        require(report.cpu_preflight_blocked_tick_count == 1u &&
            report.first_cpu_preflight_blocked_tick_sequence == 1u &&
            report.pending_tick_count == 1u &&
            std::abs(report.accumulator_seconds) <= 1.0e-12,
            "CPU Preflight must create one blocked head Reservation and move its time out of the Accumulator.");

        skin_physics::runtime::world_status other_status;
        require(runtime.poll(other_world, &other_status).success(),
            "Polling an unrelated World must succeed while the first World has a CPU-only Preflight Block.");

        skin_physics::runtime::world_execution_budget unobserved_budget;
        unobserved_budget.maximum_substeps = 4u;
        unobserved_budget.maximum_integration_segments_per_tick = 512u;
        require(runtime.set_world_execution_budget(
                world, unobserved_budget).code == status_code::busy,
            "Polling another World must not authorize CPU Preflight Budget Retry for this World.");

        skin_physics::runtime::world_status before_retry;
        require(runtime.poll(world, &before_retry).success() &&
            before_retry.execution_budget_revision == 1u &&
            before_retry.state == skin_physics::runtime::tick_state::blocked &&
            before_retry.last_tick_status == status_code::work_budget_exceeded &&
            before_retry.budget_block_origin ==
                skin_physics::runtime::work_budget_block_origin::cpu_preflight &&
            before_retry.planned_substeps == 4u &&
            before_retry.head_reserved_tick_sequence == 1u &&
            before_retry.head_input_snapshot_version != 0u &&
            std::abs(before_retry.reserved_tick_time - 0.02) <= 1.0e-12,
            "CPU Preflight must report required work while preserving the exact head Reservation.");

        require(runtime.set_gravity(
                world, { 0.0f, -1.0f, 0.0f }).code == status_code::busy,
            "World input mutation must be rejected while a Tick Reservation holds its immutable input snapshot.");

        skin_physics::runtime::world_execution_budget insufficient_budget;
        insufficient_budget.maximum_substeps = 3u;
        insufficient_budget.maximum_integration_segments_per_tick = 512u;
        require(runtime.set_world_execution_budget(
                world, insufficient_budget).code == status_code::invalid_argument,
            "A blocked Reservation must reject an Admission Limit below its derived Substep lower bound.");
        skin_physics::runtime::world_execution_budget decreasing_budget;
        decreasing_budget.maximum_substeps = 4u;
        decreasing_budget.maximum_integration_segments_per_tick = 256u;
        require(runtime.set_world_execution_budget(
                world, decreasing_budget).code == status_code::invalid_argument,
            "A blocked Reservation must reject lowering an unrelated Admission Limit even when the derived work fits.");
        skin_physics::runtime::world_status after_rejected_budget;
        require(runtime.poll(world, &after_rejected_budget).success() &&
            after_rejected_budget.execution_budget_revision == 1u &&
            after_rejected_budget.state == skin_physics::runtime::tick_state::blocked &&
            after_rejected_budget.head_reserved_tick_sequence == 1u &&
            std::abs(after_rejected_budget.reserved_tick_time - 0.02) <= 1.0e-12,
            "Rejected Budget changes must leave Revision, blocked Reservation, Sequence and reserved time unchanged.");

        skin_physics::runtime::world_execution_budget increased_budget;
        increased_budget.maximum_substeps = 4u;
        increased_budget.maximum_integration_segments_per_tick = 512u;
        require(runtime.set_world_execution_budget(world, increased_budget).success(),
            "World budget recovery must accept the required Substep count below the Runtime Hard Limit.");

        skin_physics::runtime::world_status after_retry;
        require(runtime.poll(world, &after_retry).success() &&
            after_retry.execution_budget_revision == 2u &&
            after_retry.state == skin_physics::runtime::tick_state::pending &&
            after_retry.last_tick_status == status_code::ok &&
            after_retry.work_budget_block_flags == 0u &&
            after_retry.head_reserved_tick_sequence == 1u &&
            after_retry.head_input_snapshot_version ==
                before_retry.head_input_snapshot_version &&
            std::abs(after_retry.reserved_tick_time - 0.02) <= 1.0e-12 &&
            after_retry.pending_tick_count == 1u,
            "Budget recovery must re-evaluate the same Reservation without allocating a duplicate Tick or changing its reserved time.");

        require(runtime.reset_world(world).success(),
            "Reset must explicitly release a CPU-only pending Reservation.");
        require(runtime.destroy_world(world).success(),
            "World must be destroyable after Reservation Reset.");
        require(runtime.destroy_world(other_world).success(),
            "The unrelated World must remain idle and destroyable after its Status Poll.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the CPU Preflight recovery regression.");
    }

    void test_runtime_soft_body_gpu_tick_and_state_bank_sequence()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Soft Body Runtime regression requires a D3D11 WARP Feature Level 11_0 device.");

        physics_runtime runtime;
        skin_physics::runtime::runtime_desc runtime_desc;
        runtime_desc.shader_package_root = runtime_shader_root();
        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).success(),
            "Runtime must compile the production Tick and Soft Body shader package.");

        skin_physics::physics::raw_tet_mesh raw_mesh;
        raw_mesh.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f }
        };
        raw_mesh.tetrahedra = { { 0u, 1u, 2u, 3u } };
        const skin_physics::physics::physics_mesh_build_result built_mesh =
            skin_physics::physics::build_physics_mesh(raw_mesh);
        require(built_mesh.success,
            "Soft Body Runtime fixture must build a valid one-Tet Physics Mesh.");

        skin_physics::asset::mesh render_mesh;
        render_mesh.name = "runtime_soft_body_tet";
        render_mesh.mode = skin_physics::asset::physics_mode::soft_body;
        for (const skin_physics::asset::vec3& position : raw_mesh.positions)
        {
            skin_physics::asset::vertex vertex;
            vertex.position = position;
            vertex.normal = { 0.0f, 0.0f, 1.0f };
            vertex.tangent = { 1.0f, 0.0f, 0.0f, 1.0f };
            vertex.attributes =
                skin_physics::asset::attribute_normal |
                skin_physics::asset::attribute_tangent;
            render_mesh.vertices.push_back(vertex);
        }
        for (const skin_physics::physics::boundary_triangle& boundary :
            built_mesh.value.boundary_triangles)
        {
            render_mesh.indices.insert(
                render_mesh.indices.end(),
                boundary.vertices.begin(),
                boundary.vertices.end());
        }

        skin_physics::runtime::model_desc model_desc;
        model_desc.source_model.meshes.push_back(render_mesh);
        model_desc.source_model.meshes.push_back(render_mesh);
        model_desc.world_transform.translation = { 2.0f, 3.0f, 4.0f };
        const float root_rotation_scale = 1.0002f;
        model_desc.world_transform.rotation = {
            0.0f,
            0.3826834324f * root_rotation_scale,
            0.0f,
            0.9238795325f * root_rotation_scale
        };
        skin_physics::runtime::model_mesh_physics_asset mesh_asset;
        mesh_asset.mesh_index = 0u;
        mesh_asset.physics_mesh = built_mesh.value;
        model_desc.mesh_physics_assets.push_back(mesh_asset);
        mesh_asset.mesh_index = 1u;
        model_desc.mesh_physics_assets.push_back(std::move(mesh_asset));

        skin_physics::runtime::world_desc world_desc;
        world_desc.fixed_delta_time = 0.01;
        world_desc.contact_max_substep_delta_time = 0.004;
        world_desc.maximum_integration_segments_per_tick = 4u;
        world_desc.maximum_in_flight_ticks = 2u;
        world_desc.maximum_steps_per_advance = 2u;
        skin_physics::runtime::world_handle world;
        require(runtime.create_world(world_desc, &world).success(),
            "Soft Body Runtime World creation must succeed.");

        skin_physics::runtime::model_desc invalid_rest_inverse_desc = model_desc;
        invalid_rest_inverse_desc.mesh_physics_assets[0].physics_mesh
            .tetrahedra[0].rest_inverse[0] =
                std::numeric_limits<float>::quiet_NaN();
        skin_physics::runtime::model_handle invalid_model;
        require(runtime.add_model(
                world, invalid_rest_inverse_desc, &invalid_model).code ==
                    status_code::asset_invalid,
            "Runtime must reject non-finite rest inverse data before allocating Soft Body GPU resources.");

        skin_physics::runtime::model_desc invalid_csr_desc = model_desc;
        invalid_csr_desc.mesh_physics_assets[0].physics_mesh
            .vertex_tetrahedron_ids[0] =
                std::numeric_limits<uint32_t>::max();
        require(runtime.add_model(
                world, invalid_csr_desc, &invalid_model).code ==
                    status_code::asset_invalid,
            "Runtime must reject Vertex-to-Tet CSR entries that do not exactly match Tet topology.");

        skin_physics::runtime::model_desc duplicate_vertex_desc = model_desc;
        duplicate_vertex_desc.mesh_physics_assets[0].physics_mesh
            .tetrahedra[0].vertices[3] = 0u;
        require(runtime.add_model(
                world, duplicate_vertex_desc, &invalid_model).code ==
                    status_code::asset_invalid,
            "Runtime must reject tetrahedra with repeated vertex references.");

        skin_physics::runtime::model_handle model;
        require(runtime.add_model(world, model_desc, &model).success(),
            "Runtime must upload the Soft Body asset and allocate two persistent State Banks plus Tick Scratch.");

        skin_physics::runtime::mesh_physics_parameters schedule_material;
        schedule_material.density = 1000.0f;
        schedule_material.young_modulus = 1.0f;
        schedule_material.poisson_ratio = 0.3f;
        schedule_material.damping = 0.0f;
        require(runtime.set_mesh_physics_parameters(
                world, model, 0u, schedule_material).success() &&
            runtime.set_mesh_physics_parameters(
                world, model, 1u, schedule_material).success(),
            "Low-stiffness Soft Body materials must leave the World Substep schedule at its contact-time minimum.");

        skin_physics::runtime::attribute_edit_desc paint_edit;
        paint_edit.channel =
            skin_physics::asset::physics_attribute_channel::shape_recovery;
        paint_edit.vertex_indices = { 0u };
        paint_edit.target = 0.25f;
        paint_edit.strength = 1.0f;
        require(runtime.apply_attribute_edit(world, model, 0u, paint_edit).success(),
            "An idle Soft Body Paint edit must update Render Mesh values and refresh the Physics Mesh harmonic transfer.");

        skin_physics::runtime::force_desc force;
        force.event_id = 101u;
        force.target.model = model;
        force.target.mesh_index = 0u;
        force.position = { 2.0f, 3.0f, 4.0f };
        force.direction = { 0.0f, 0.0f, 0.0f };
        force.magnitude = 10.0f;
        force.radius = 1.25f;
        force.duration = 0.1;
        force.start_time = 0.0;
        require(runtime.enqueue_force(world, force).code == status_code::invalid_argument,
            "Runtime must reject a zero-length Force direction before reserving its Event ID.");
        force.direction = { 0.0f, 5.0f, 0.0f };
        force.start_time = std::numeric_limits<double>::max() * 0.5;
        force.duration = std::numeric_limits<double>::max();
        require(runtime.enqueue_force(world, force).code == status_code::invalid_argument,
            "Runtime must reject a Force interval whose finite endpoints overflow in World time.");
        force.start_time = 1.0e300;
        force.duration = std::numeric_limits<double>::epsilon();
        require(runtime.enqueue_force(world, force).code == status_code::invalid_argument,
            "Runtime must reject a Force duration that collapses to a zero-width interval at its start time.");
        force.start_time = 0.0025;
        force.duration = 0.0025;
        require(runtime.enqueue_force(world, force).success(),
            "Runtime must normalize and accept a finite non-unit Soft Body Force direction.");
        skin_physics::runtime::force_desc same_time_force = force;
        same_time_force.event_id = 104u;
        same_time_force.start_time = force.start_time;
        same_time_force.duration = force.duration;
        require(runtime.enqueue_force(world, same_time_force).success(),
            "A second Force Event may share the same World-time boundaries without duplicating Segment boundaries.");

        skin_physics::runtime::force_desc end_boundary_force = force;
        end_boundary_force.event_id = 105u;
        end_boundary_force.start_time = 0.01;
        end_boundary_force.duration = 0.01;
        require(runtime.enqueue_force(world, end_boundary_force).success(),
            "A Force starting exactly at the Tick end must be retained for the next Tick without extending this Tick.");

        skin_physics::runtime::impulse_desc impulse;
        impulse.event_id = 101u;
        impulse.target.model = model;
        impulse.target.mesh_index = 0u;
        impulse.position = { 2.0f, 3.0f, 4.0f };
        impulse.direction = { 0.0f, -3.0f, 0.0f };
        impulse.impulse_magnitude = 2.0f;
        impulse.radius = 1.0f;
        impulse.event_time = 0.0075;
        require(runtime.enqueue_impulse(world, impulse).code == status_code::invalid_argument,
            "Force and Impulse descriptors must share one World-wide unique Event ID namespace.");
        impulse.event_id = 102u;
        impulse.direction = { 0.0f, 0.0f, 0.0f };
        require(runtime.enqueue_impulse(world, impulse).code == status_code::invalid_argument,
            "Runtime must reject a zero-length Impulse direction before reserving its Event ID.");
        impulse.direction = { 0.0f, -3.0f, 0.0f };
        require(runtime.enqueue_impulse(world, impulse).success(),
            "Runtime must normalize and accept a finite non-unit Soft Body Impulse direction.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "Force / Impulse regression World must enter Playing.");
        skin_physics::runtime::advance_report force_report;
        require(runtime.advance(world, 0.01, &force_report).success() &&
            force_report.submitted_tick_count == 0u &&
            force_report.in_flight_tick_count == 0u &&
            force_report.pending_tick_count == 1u &&
            force_report.cpu_preflight_blocked_tick_count == 1u,
            "Force / Impulse time boundaries must contribute to CPU Preflight Segment admission before GPU submission.");
        skin_physics::runtime::force_desc late_force = force;
        late_force.event_id = 103u;
        require(runtime.enqueue_force(world, late_force).code == status_code::busy,
            "A Tick Reservation must freeze World input mutation until Commit or Reset.");
        skin_physics::runtime::world_status force_status;
        require(runtime.poll(world, &force_status).success() &&
            force_status.state == skin_physics::runtime::tick_state::blocked &&
            force_status.last_tick_status == status_code::work_budget_exceeded &&
            force_status.budget_block_origin ==
                skin_physics::runtime::work_budget_block_origin::cpu_preflight &&
            force_status.work_budget_block_flags == 0x02u &&
            force_status.head_reserved_tick_sequence == 1u &&
            force_status.head_input_snapshot_version != 0u &&
            force_status.planned_substeps == 3u &&
            force_status.planned_integration_segments == 6u,
            "Force start / end and Impulse times must produce six planned Segments from three base Substeps and block at the four-Segment Admission Limit.");
        const uint64_t force_snapshot_version =
            force_status.head_input_snapshot_version;
        skin_physics::runtime::world_execution_budget recovered_event_budget;
        recovered_event_budget.maximum_substeps = 128u;
        recovered_event_budget.maximum_integration_segments_per_tick = 6u;
        require(runtime.set_world_execution_budget(
                world, recovered_event_budget).success(),
            "Budget recovery must admit the exact event-derived Segment count.");
        skin_physics::runtime::advance_report force_retry_report;
        skin_physics::runtime::world_status force_retry_pre_submit_status;
        require(runtime.poll(world, &force_retry_pre_submit_status).success() &&
            force_retry_pre_submit_status.state == skin_physics::runtime::tick_state::pending &&
            force_retry_pre_submit_status.head_reserved_tick_sequence == 1u &&
            force_retry_pre_submit_status.head_input_snapshot_version ==
                force_snapshot_version &&
            force_retry_pre_submit_status.planned_substeps == 3u &&
            force_retry_pre_submit_status.planned_integration_segments == 6u,
            "Budget Retry must preserve the exact Input Snapshot Version and event-derived Segment plan before GPU submission.");

        require(runtime.advance(world, 0.0, &force_retry_report).success() &&
            force_retry_report.submitted_tick_count == 1u &&
            force_retry_report.in_flight_tick_count == 1u &&
            force_retry_report.pending_tick_count == 0u,
            "The admitted exact-Mesh Force / Impulse Reservation must submit without adding time or allocating another Tick.");
        skin_physics::runtime::world_status force_retry_status;
        bool force_tick_observed = false;
        for (uint32_t attempt = 0u; attempt < 2048u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &force_retry_status).success(),
                "Force / Impulse GPU Tick observation must remain non-blocking.");
            if (force_retry_status.in_flight_tick_count == 0u)
            {
                force_tick_observed = true;
                break;
            }
            std::this_thread::yield();
        }
        require(force_tick_observed &&
            force_retry_status.state == skin_physics::runtime::tick_state::committed &&
            force_retry_status.last_tick_status == status_code::ok &&
            force_retry_status.head_input_snapshot_version == 0u &&
            force_retry_status.submitted_sequence == 1u &&
            force_retry_status.gpu_committed_sequence == 1u &&
            force_retry_status.cpu_observed_sequence == 1u &&
            force_retry_status.planned_substeps == 3u,
            std::string(
                "Force / Impulse GPU execution must commit the same six-Segment Reservation after Budget Retry. observed=") +
                (force_tick_observed ? "true" : "false") +
                " state=" +
                std::to_string(static_cast<int>(force_retry_status.state)) +
                " status=" +
                std::to_string(static_cast<int>(force_retry_status.last_tick_status)) +
                " submitted=" + std::to_string(force_retry_status.submitted_sequence) +
                " gpu_committed=" +
                std::to_string(force_retry_status.gpu_committed_sequence) +
                " cpu_observed=" +
                std::to_string(force_retry_status.cpu_observed_sequence) +
                " reserved=" +
                std::to_string(force_retry_status.head_reserved_tick_sequence) +
                " snapshot=" +
                std::to_string(force_retry_status.head_input_snapshot_version) +
                " planned_segments=" +
                std::to_string(force_retry_status.planned_integration_segments) +
                " executed_segments=" +
                std::to_string(force_retry_status.executed_integration_segments) +
                " diagnostic=" + force_retry_status.diagnostic);

        skin_physics::runtime::render_binding force_mesh_binding;
        skin_physics::runtime::render_binding untouched_mesh_binding;
        require(runtime.get_render_binding(
                world, model, 0u, &force_mesh_binding).success() &&
            runtime.get_render_binding(
                world, model, 1u, &untouched_mesh_binding).success() &&
            force_mesh_binding.committed_tick_sequence == 1u &&
            untouched_mesh_binding.committed_tick_sequence == 1u,
            "A completed Force / Impulse Tick must publish both Soft Body Mesh Render Candidates under one Host Lease.");
        const std::vector<test_gpu_float4> force_mesh_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), force_mesh_binding.position_srv,
                force_mesh_binding.vertex_count);
        const std::vector<test_gpu_float4> untouched_mesh_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), untouched_mesh_binding.position_srv,
                untouched_mesh_binding.vertex_count);
        require(force_mesh_positions.size() == untouched_mesh_positions.size() &&
            !force_mesh_positions.empty(),
            "Force / Impulse Render Feedback must preserve selected and sibling Mesh vertex counts.");
        require(force_mesh_positions[0u].y <
                untouched_mesh_positions[0u].y - 2.0e-5f,
            "The selected Mesh must receive its mass-weighted Force / Impulse while the sibling Mesh remains gravity-only.");
        require(runtime.release_render_bindings(
                world, force_mesh_binding.lease_generation).success(),
            "Force / Impulse Render Candidate SRVs must release their shared Host Lease after readback.");

        const float exact_mesh_event_delta =
            force_mesh_positions[0u].y - untouched_mesh_positions[0u].y;
        require(exact_mesh_event_delta < -2.0e-5f,
            "The exact-Mesh reference must produce a measurable event response before target-scope comparisons.");

        require(runtime.reset_world(world).success(),
            "Reset must restore both Meshes before the Model-scope denominator regression.");
        skin_physics::runtime::force_desc model_scope_force = force;
        model_scope_force.event_id = 301u;
        model_scope_force.target.mesh_index = skin_physics::asset::invalid_index;
        skin_physics::runtime::force_desc model_scope_same_time_force =
            same_time_force;
        model_scope_same_time_force.event_id = 302u;
        model_scope_same_time_force.target.mesh_index =
            skin_physics::asset::invalid_index;
        skin_physics::runtime::impulse_desc model_scope_impulse = impulse;
        model_scope_impulse.event_id = 303u;
        model_scope_impulse.target.mesh_index =
            skin_physics::asset::invalid_index;
        require(runtime.enqueue_force(world, model_scope_force).success() &&
            runtime.enqueue_force(world, model_scope_same_time_force).success() &&
            runtime.enqueue_impulse(world, model_scope_impulse).success(),
            "Model-scope Force / Impulse Events must accept a Model Handle with an invalid Mesh Index.");
        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "The Model-scope target World must enter Playing.");
        skin_physics::runtime::advance_report model_scope_report;
        require(runtime.advance(world, 0.01, &model_scope_report).success() &&
            model_scope_report.submitted_tick_count == 1u &&
            model_scope_report.in_flight_tick_count == 1u,
            "A Model-scope Force / Impulse Tick must submit all Meshes through one target-scope denominator.");
        skin_physics::runtime::world_status model_scope_status;
        bool model_scope_observed = false;
        for (uint32_t attempt = 0u; attempt < 2048u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &model_scope_status).success(),
                "Model-scope Force / Impulse Status polling must remain non-blocking.");
            if (model_scope_status.in_flight_tick_count == 0u)
            {
                model_scope_observed = true;
                break;
            }
            std::this_thread::yield();
        }
        require(model_scope_observed &&
            model_scope_status.state == skin_physics::runtime::tick_state::committed &&
            model_scope_status.gpu_committed_sequence == 1u,
            "Model-scope Force / Impulse work must Commit after its shared Mesh reduction and Render validation.");

        skin_physics::runtime::render_binding model_scope_mesh0_binding;
        skin_physics::runtime::render_binding model_scope_mesh1_binding;
        require(runtime.get_render_binding(
                world, model, 0u, &model_scope_mesh0_binding).success() &&
            runtime.get_render_binding(
                world, model, 1u, &model_scope_mesh1_binding).success(),
            "Model-scope Render Candidates must publish both target Meshes under one Host Lease.");
        const std::vector<test_gpu_float4> model_scope_mesh0_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), model_scope_mesh0_binding.position_srv,
                model_scope_mesh0_binding.vertex_count);
        const std::vector<test_gpu_float4> model_scope_mesh1_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), model_scope_mesh1_binding.position_srv,
                model_scope_mesh1_binding.vertex_count);
        const float model_scope_mesh0_delta =
            model_scope_mesh0_positions[0u].y - untouched_mesh_positions[0u].y;
        const float model_scope_mesh1_delta =
            model_scope_mesh1_positions[0u].y - untouched_mesh_positions[0u].y;
        const float model_scope_mesh0_ratio =
            model_scope_mesh0_delta / exact_mesh_event_delta;
        const float model_scope_mesh1_ratio =
            model_scope_mesh1_delta / exact_mesh_event_delta;
        require(std::abs(model_scope_mesh0_ratio - 0.5f) <= 0.15f &&
            std::abs(model_scope_mesh1_ratio - 0.5f) <= 0.15f,
            std::string(
                "A Model-scope event must divide one descriptor magnitude across both Meshes; each identical Mesh receives half the exact-Mesh response. ratios=") +
                std::to_string(model_scope_mesh0_ratio) + "," +
                std::to_string(model_scope_mesh1_ratio));
        require(runtime.release_render_bindings(
                world, model_scope_mesh0_binding.lease_generation).success(),
            "Model-scope Render Candidate SRVs must release their shared Host Lease.");

        require(runtime.reset_world(world).success(),
            "Reset must restore the original Model before the World-scope denominator regression.");
        skin_physics::runtime::model_handle second_model;
        require(runtime.add_model(world, model_desc, &second_model).success(),
            "A second two-Mesh Soft Body Model must be registerable before a World-scope event.");
        skin_physics::runtime::force_desc world_scope_force = force;
        world_scope_force.event_id = 401u;
        world_scope_force.target = {};
        skin_physics::runtime::force_desc world_scope_same_time_force =
            same_time_force;
        world_scope_same_time_force.event_id = 402u;
        world_scope_same_time_force.target = {};
        skin_physics::runtime::impulse_desc world_scope_impulse = impulse;
        world_scope_impulse.event_id = 403u;
        world_scope_impulse.target = {};
        require(runtime.enqueue_force(world, world_scope_force).success() &&
            runtime.enqueue_force(world, world_scope_same_time_force).success() &&
            runtime.enqueue_impulse(world, world_scope_impulse).success(),
            "World-scope Force / Impulse Events must accept the default invalid Model / Mesh target.");
        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "The World-scope target World must enter Playing.");
        skin_physics::runtime::advance_report world_scope_report;
        require(runtime.advance(world, 0.01, &world_scope_report).success() &&
            world_scope_report.submitted_tick_count == 1u &&
            world_scope_report.in_flight_tick_count == 1u,
            "A World-scope Force / Impulse Tick must submit all registered Models and Meshes through one denominator.");
        skin_physics::runtime::world_status world_scope_status;
        bool world_scope_observed = false;
        for (uint32_t attempt = 0u; attempt < 2048u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &world_scope_status).success(),
                "World-scope Force / Impulse Status polling must remain non-blocking.");
            if (world_scope_status.in_flight_tick_count == 0u)
            {
                world_scope_observed = true;
                break;
            }
            std::this_thread::yield();
        }
        require(world_scope_observed &&
            world_scope_status.state == skin_physics::runtime::tick_state::committed &&
            world_scope_status.gpu_committed_sequence == 1u,
            "World-scope Force / Impulse work must Commit only after the shared four-Mesh denominator is applied.");

        skin_physics::runtime::render_binding world_scope_mesh0_binding;
        skin_physics::runtime::render_binding world_scope_mesh1_binding;
        skin_physics::runtime::render_binding second_model_mesh0_binding;
        skin_physics::runtime::render_binding second_model_mesh1_binding;
        require(runtime.get_render_binding(
                world, model, 0u, &world_scope_mesh0_binding).success() &&
            runtime.get_render_binding(
                world, model, 1u, &world_scope_mesh1_binding).success() &&
            runtime.get_render_binding(
                world, second_model, 0u, &second_model_mesh0_binding).success() &&
            runtime.get_render_binding(
                world, second_model, 1u, &second_model_mesh1_binding).success(),
            "World-scope Render Candidate publication must expose all four target Meshes.");
        const std::vector<test_gpu_float4> world_scope_mesh0_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), world_scope_mesh0_binding.position_srv,
                world_scope_mesh0_binding.vertex_count);
        const std::vector<test_gpu_float4> world_scope_mesh1_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), world_scope_mesh1_binding.position_srv,
                world_scope_mesh1_binding.vertex_count);
        const std::vector<test_gpu_float4> second_model_mesh0_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), second_model_mesh0_binding.position_srv,
                second_model_mesh0_binding.vertex_count);
        const std::vector<test_gpu_float4> second_model_mesh1_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), second_model_mesh1_binding.position_srv,
                second_model_mesh1_binding.vertex_count);
        const std::vector<float> world_scope_deltas = {
            world_scope_mesh0_positions[0u].y - untouched_mesh_positions[0u].y,
            world_scope_mesh1_positions[0u].y - untouched_mesh_positions[0u].y,
            second_model_mesh0_positions[0u].y - untouched_mesh_positions[0u].y,
            second_model_mesh1_positions[0u].y - untouched_mesh_positions[0u].y
        };
        std::vector<float> world_scope_response_ratios;
        world_scope_response_ratios.reserve(world_scope_deltas.size());
        for (float value : world_scope_deltas)
        {
            world_scope_response_ratios.push_back(
                value / exact_mesh_event_delta);
        }
        require(std::all_of(
                world_scope_response_ratios.begin(),
                world_scope_response_ratios.end(),
                [](float value)
                {
                    return std::abs(value - 0.25f) <= 0.08f;
                }),
            std::string(
                "A World-scope event must divide one descriptor magnitude across all four Meshes; each identical Mesh receives one quarter the exact-Mesh response. ratios=") +
                std::to_string(world_scope_response_ratios[0u]) + "," +
                std::to_string(world_scope_response_ratios[1u]) + "," +
                std::to_string(world_scope_response_ratios[2u]) + "," +
                std::to_string(world_scope_response_ratios[3u]));
        require(runtime.release_render_bindings(
                world, world_scope_mesh0_binding.lease_generation).success(),
            "World-scope Render Candidate SRVs must release their shared Host Lease.");

        require(runtime.reset_world(world).success(),
            "Reset must clear committed scoped Force / Impulse Events before the NO_FORCE_TARGET regression.");
        skin_physics::runtime::force_desc no_support_force = force;
        no_support_force.event_id = 201u;
        no_support_force.position = { 100.0f, 100.0f, 100.0f };
        no_support_force.radius = 0.25f;
        no_support_force.start_time = 0.0;
        no_support_force.duration = 0.01;
        require(runtime.enqueue_force(world, no_support_force).success(),
            "A valid exact-Mesh Force descriptor with empty spatial support must be accepted for GPU classification.");
        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "The no-support Force regression World must enter Playing.");
        skin_physics::runtime::advance_report no_support_report;
        require(runtime.advance(world, 0.01, &no_support_report).success() &&
            no_support_report.submitted_tick_count == 1u &&
            no_support_report.in_flight_tick_count == 1u,
            "A no-support Force must reach the GPU reduction before diagnosis.");
        skin_physics::runtime::world_status no_support_status;
        bool no_support_observed = false;
        for (uint32_t attempt = 0u; attempt < 2048u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &no_support_status).success(),
                "NO_FORCE_TARGET status observation must remain non-blocking.");
            if (no_support_status.in_flight_tick_count == 0u)
            {
                no_support_observed = true;
                break;
            }
            std::this_thread::yield();
        }
        require(no_support_observed &&
            no_support_status.state == skin_physics::runtime::tick_state::rejected &&
            no_support_status.last_tick_status == status_code::no_force_target &&
            no_support_status.gpu_committed_sequence == 0u &&
            std::abs(no_support_status.reserved_tick_time - 0.01) <= 1.0e-12 &&
            no_support_status.diagnostic.find("NO_FORCE_TARGET") != std::string::npos,
            "An empty Wendland C2 support must reject the Tick without committing time or losing its diagnostic Reservation.");
        require(runtime.reset_world(world).success(),
            "Reset must release the rejected no-support Reservation and Event.");

        require(runtime.enqueue_force(world, force).success(),
            "Reset must release the Event ID namespace so a new Tick can reuse the Event ID.");
        require(runtime.reset_world(world).success(),
            "A second Reset must clear the event before the Root Transform regression.");
        skin_physics::runtime::world_status reset_status;
        require(runtime.poll(world, &reset_status).success() &&
            reset_status.head_input_snapshot_version == 0u,
            "Reset must release the previous Tick Input Snapshot when its Reservation is cancelled.");
        require(runtime.reset_world(world).success(),
            "Reset must explicitly clear the unsupported Force / Impulse Events and retained Reservation.");
        require(runtime.enqueue_force(world, force).success(),
            "Reset must release the Event ID namespace so a new Tick can register the same Event ID.");
        require(runtime.reset_world(world).success(),
            "A second Reset must clear the re-registered event before the Root Transform regression.");
        skin_physics::runtime::world_status reset_status_after_event_reuse;
        require(runtime.poll(world, &reset_status_after_event_reuse).success() &&
            reset_status_after_event_reuse.head_input_snapshot_version == 0u,
            "Reset must release the previous Tick Input Snapshot after Event ID reuse.");

        skin_physics::runtime::transform sampled_root =
            model_desc.world_transform;
        sampled_root.translation.x += 0.5f;
        require(runtime.set_model_transform_sample(
                world, model, 0.0, sampled_root, false).success(),
            "Runtime must accept a validated Root Transform Sample for the deferred-animation regression.");
        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "Root Sample regression World must enter Playing.");

        skin_physics::runtime::advance_report animation_report;
        require(runtime.advance(world, 0.01, &animation_report).success() &&
            animation_report.submitted_tick_count == 0u &&
            animation_report.in_flight_tick_count == 0u &&
            animation_report.pending_tick_count == 1u,
            "A World with unintegrated Root Transform trajectory input must retain its Tick without GPU submission.");
        skin_physics::runtime::world_status animation_status;
        require(runtime.poll(world, &animation_status).success() &&
            animation_status.state == skin_physics::runtime::tick_state::blocked &&
            animation_status.last_tick_status == status_code::blocked_by_input &&
            animation_status.in_flight_tick_count == 0u &&
            animation_status.head_reserved_tick_sequence == 1u &&
            animation_status.head_input_snapshot_version > force_snapshot_version &&
            animation_status.diagnostic.find("Animation / Root Transform") !=
                std::string::npos,
            "Runtime must explicitly report BLOCKED_BY_INPUT for deferred Root Transform trajectory work instead of committing a Tick with the input silently ignored.");
        require(runtime.reset_world(world).success(),
            "Reset must explicitly clear the CPU-only Reservation and unintegrated Root Transform Samples.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "Soft Body Runtime World must re-enter Playing after clearing the unsupported Root Transform Sample.");

        skin_physics::runtime::advance_report advance;
        require(runtime.advance(world, 0.02, &advance).success(),
            "Runtime must accept two fixed-step Soft Body Tick requests.");
        require(advance.submitted_tick_count == 2u &&
            advance.in_flight_tick_count == 2u &&
            advance.pending_tick_count == 0u,
            "Runtime must submit two consecutive gated Soft Body Ticks within the World in-flight limit.");

        skin_physics::runtime::render_binding unobserved_binding;
        require(runtime.get_render_binding(
                world, model, 0u, &unobserved_binding).code ==
                    status_code::not_ready,
            "A GPU-submitted Render Candidate must remain private until its Event Query and COMMITTED Status are CPU-observed.");

        skin_physics::runtime::world_status result;
        bool all_queries_observed = false;
        for (uint32_t attempt = 0u; attempt < 2048u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &result).success(),
                "Soft Body GPU Tick Status polling must remain non-blocking and successful.");
            if (result.in_flight_tick_count == 0u)
            {
                all_queries_observed = true;
                break;
            }
            std::this_thread::yield();
        }

        require(all_queries_observed &&
            result.state == skin_physics::runtime::tick_state::committed &&
            result.last_tick_status == status_code::ok &&
            result.submitted_sequence == 2u &&
            result.gpu_committed_sequence == 2u &&
            result.cpu_observed_sequence == 2u &&
            std::abs(result.gpu_committed_simulation_time - 0.02) <= 1.0e-12 &&
            std::abs(result.cpu_observed_completed_time - 0.02) <= 1.0e-12,
            "WARP must execute odd-substep Scratch publication, alternate the committed State Bank across Ticks, and observe the contiguous Commit prefix.");

        skin_physics::runtime::advance_report third_tick_report;
        require(runtime.advance(world, 0.01, &third_tick_report).success() &&
            third_tick_report.submitted_tick_count == 1u &&
            third_tick_report.in_flight_tick_count == 1u &&
            third_tick_report.pending_tick_count == 0u,
            "A third Tick must be submitted before the Host acquires the previous committed Render Candidate.");

        skin_physics::runtime::render_binding render_binding;
        require(runtime.get_render_binding(
                world, model, 0u, &render_binding).success(),
            "A CPU-observed committed Soft Body Render Candidate must be exposed through a Host Render Lease.");
        require(render_binding.position_srv != nullptr &&
            render_binding.normal_srv != nullptr &&
            render_binding.tangent_srv != nullptr &&
            render_binding.format == DXGI_FORMAT_R32G32B32A32_FLOAT &&
            render_binding.stride == 16u &&
            render_binding.vertex_count == render_mesh.vertices.size() &&
            render_binding.index_count == render_mesh.indices.size() &&
            render_binding.committed_tick_sequence == 2u &&
            render_binding.render_slot_id != skin_physics::asset::invalid_index &&
            render_binding.resource_generation != 0u &&
            render_binding.lease_generation != 0u,
            "Render Binding must identify the committed Ring Slot and provide the expected GPU stream layout.");

        const std::vector<test_gpu_float4> rendered_positions =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), render_binding.position_srv,
                render_binding.vertex_count);
        const std::vector<test_gpu_float4> rendered_normals =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), render_binding.normal_srv,
                render_binding.vertex_count);
        const std::vector<test_gpu_float4> rendered_tangents =
            readback_render_srv<test_gpu_float4>(
                device.Get(), context.Get(), render_binding.tangent_srv,
                render_binding.vertex_count);

        require(rendered_positions.size() == render_mesh.vertices.size() &&
            rendered_normals.size() == render_mesh.vertices.size() &&
            rendered_tangents.size() == render_mesh.vertices.size(),
            "GPU Render Candidate streams must preserve the Render Mesh Vertex count.");

        for (std::size_t vertex_index = 0u;
            vertex_index < rendered_positions.size(); ++vertex_index)
        {
            const test_gpu_float4& position = rendered_positions[vertex_index];
            const test_gpu_float4& normal = rendered_normals[vertex_index];
            const test_gpu_float4& tangent = rendered_tangents[vertex_index];
            const skin_physics::asset::vec3& rest_position =
                render_mesh.vertices[vertex_index].position;

            require(std::isfinite(position.x) && std::isfinite(position.y) &&
                std::isfinite(position.z) && position.w == 1.0f,
                "Committed Render Position SRV must contain finite homogeneous model-local positions.");
            require(std::abs(position.x - rest_position.x) <= 1.0e-3f &&
                std::abs(position.z - rest_position.z) <= 1.0e-3f &&
                position.y < rest_position.y &&
                position.y > rest_position.y - 0.01f,
                "Render Feedback must invert the translated, rotated and slightly non-unit input Root Transform and preserve the expected small gravity displacement in model-local space.");

            const double normal_length_squared =
                static_cast<double>(normal.x) * normal.x +
                static_cast<double>(normal.y) * normal.y +
                static_cast<double>(normal.z) * normal.z;
            const double tangent_length_squared =
                static_cast<double>(tangent.x) * tangent.x +
                static_cast<double>(tangent.y) * tangent.y +
                static_cast<double>(tangent.z) * tangent.z;
            const double normal_tangent_dot =
                static_cast<double>(normal.x) * tangent.x +
                static_cast<double>(normal.y) * tangent.y +
                static_cast<double>(normal.z) * tangent.z;

            require(std::isfinite(normal_length_squared) &&
                std::abs(normal_length_squared - 1.0) <= 1.0e-3,
                "Committed Render Normal SRV must contain normalized finite normals.");
            require(std::isfinite(tangent_length_squared) &&
                std::abs(tangent_length_squared - 1.0) <= 1.0e-3 &&
                std::abs(normal_tangent_dot) <= 1.0e-3 &&
                std::abs(std::abs(tangent.w) - 1.0f) <= 1.0e-4f,
                "Committed Render Tangent SRV must contain normalized, orthogonal tangents with a valid handedness sign.");
        }

        // Tick 3 was already submitted when the Host acquired the Tick 2 Candidate.
        // Its Query can commit a newer slot while the older slot remains leased.
        bool third_query_observed = false;
        for (uint32_t attempt = 0u; attempt < 2048u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &result).success(),
                "Render Lease must coexist with non-blocking Query observation.");
            if (result.in_flight_tick_count == 0u)
            {
                third_query_observed = true;
                break;
            }
            std::this_thread::yield();
        }
        require(third_query_observed &&
            result.cpu_observed_sequence == 3u &&
            result.gpu_committed_sequence == 3u,
            "An already-submitted later Tick must be allowed to commit while an older Render Candidate is leased.");

        skin_physics::runtime::render_binding same_lease_binding;
        require(runtime.get_render_binding(
                world, model, 1u, &same_lease_binding).success() &&
            same_lease_binding.lease_generation == render_binding.lease_generation &&
            same_lease_binding.render_slot_id == render_binding.render_slot_id &&
            same_lease_binding.render_slot_generation ==
                render_binding.render_slot_generation &&
            same_lease_binding.resource_generation ==
                render_binding.resource_generation &&
            same_lease_binding.committed_tick_sequence == 2u,
            "A second Mesh request must stay pinned to the original leased Slot even after a newer Candidate becomes latest.");

        require(runtime.release_render_bindings(
                world, render_binding.lease_generation + 1u).code ==
                    status_code::stale_handle,
            "A mismatched future Lease Generation must be rejected without releasing the active Host Lease.");

        skin_physics::runtime::advance_report while_leased_report;
        require(runtime.advance(world, 0.01, &while_leased_report).success() &&
            while_leased_report.submitted_tick_count == 0u &&
            while_leased_report.in_flight_tick_count == 0u &&
            while_leased_report.pending_tick_count == 1u &&
            while_leased_report.render_binding_backpressure &&
            !while_leased_report.capacity_blocked,
            "A Host Render Lease must report Render Backpressure, block reuse of its pinned Ring Slot and keep new Tick time pending.");

        require(runtime.release_render_bindings(
                world, render_binding.lease_generation).success(),
            "Host must release the Render Lease after unbinding the candidate SRVs.");
        require(runtime.release_render_bindings(
                world, render_binding.lease_generation).code == status_code::stale_handle,
            "A Render Lease generation must be released exactly once.");

        skin_physics::runtime::render_binding latest_binding;
        require(runtime.get_render_binding(
                world, model, 1u, &latest_binding).success() &&
            latest_binding.committed_tick_sequence == 3u &&
            latest_binding.render_slot_id != render_binding.render_slot_id &&
            latest_binding.lease_generation > render_binding.lease_generation,
            "After the older Lease is released, the latest CPU-observed Render Candidate must become available.");
        require(runtime.release_render_bindings(
                world, latest_binding.lease_generation).success(),
            "The newest Render Candidate Lease must also be releasable.");

        skin_physics::runtime::advance_report resumed_report;
        require(runtime.advance(world, 0.0, &resumed_report).success() &&
            resumed_report.submitted_tick_count == 1u &&
            resumed_report.in_flight_tick_count == 1u &&
            resumed_report.pending_tick_count == 0u,
            "Releasing the Render Lease must allow the exact pending Reservation to resume without adding new frame time.");

        bool resumed_query_observed = false;
        for (uint32_t attempt = 0u; attempt < 2048u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &result).success(),
                "Resumed Render Lease-blocked Tick must be observable through the non-blocking Query Ring.");
            if (result.in_flight_tick_count == 0u)
            {
                resumed_query_observed = true;
                break;
            }
            std::this_thread::yield();
        }
        require(resumed_query_observed &&
            result.cpu_observed_sequence == 4u &&
            result.gpu_committed_sequence == 4u &&
            std::abs(result.cpu_observed_completed_time - 0.04) <= 1.0e-12,
            "The pending Reservation must commit as Tick 4 with its original requested simulation time after Lease release.");

        require(runtime.reset_world(world).success(),
            "Reset must restore both persistent Soft Body State Banks to the model's initial world-space pose and discard the CPU-only Tick held by the Render Lease.");

        require(runtime.destroy_model(world, model).success(),
            "An idle reset Soft Body Model must release its GPU State Bank resources.");
        require(runtime.destroy_world(world).success(),
            "The reset and resource-released Soft Body World must be destroyable.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the Soft Body GPU state-bank regression.");
    }

    void test_runtime_multiple_in_flight_non_commit_preserves_head_sequence()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Predecessor failure regression requires a D3D11 WARP Feature Level 11_0 device.");

        physics_runtime runtime;
        skin_physics::runtime::runtime_desc runtime_desc;
        runtime_desc.shader_package_root = runtime_shader_root();
        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).success(),
            "Runtime must compile Tick, FEM and Render Feedback shaders for the predecessor failure regression.");

        skin_physics::physics::raw_tet_mesh raw_mesh;
        raw_mesh.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f }
        };
        raw_mesh.tetrahedra = { { 0u, 1u, 2u, 3u } };
        const skin_physics::physics::physics_mesh_build_result built_mesh =
            skin_physics::physics::build_physics_mesh(raw_mesh);
        require(built_mesh.success,
            "Predecessor failure fixture must build a valid one-Tet Physics Mesh.");

        skin_physics::asset::mesh render_mesh;
        render_mesh.name = "runtime_soft_body_degenerate_render_face";
        render_mesh.mode = skin_physics::asset::physics_mode::soft_body;
        render_mesh.vertices.resize(4u);
        for (uint32_t vertex_index = 0u; vertex_index < 4u; ++vertex_index)
        {
            render_mesh.vertices[vertex_index].position =
                raw_mesh.positions[vertex_index];
        }
        for (const skin_physics::physics::boundary_triangle& boundary :
            built_mesh.value.boundary_triangles)
        {
            render_mesh.indices.insert(
                render_mesh.indices.end(),
                boundary.vertices.begin(),
                boundary.vertices.end());
        }

        // Add an unused degenerate triangle whose three distinct Render
        // Vertex IDs bind to the exact same point on a Tet boundary edge. Their
        // GPU candidate positions are bit-identical, so finite-area validation
        // deterministically fails without invalidating the Physics Mesh surface.
        const std::array<skin_physics::asset::vec3, 3> degenerate_positions = {
            skin_physics::asset::vec3{ 0.25f, 0.0f, 0.0f },
            skin_physics::asset::vec3{ 0.25f, 0.0f, 0.0f },
            skin_physics::asset::vec3{ 0.25f, 0.0f, 0.0f }
        };
        for (const skin_physics::asset::vec3& position : degenerate_positions)
        {
            skin_physics::asset::vertex vertex;
            vertex.position = position;
            render_mesh.vertices.push_back(vertex);
        }
        render_mesh.indices.insert(
            render_mesh.indices.end(), { 4u, 5u, 6u });

        skin_physics::runtime::model_desc model_desc;
        model_desc.source_model.meshes.push_back(render_mesh);
        skin_physics::runtime::model_mesh_physics_asset mesh_asset;
        mesh_asset.mesh_index = 0u;
        mesh_asset.physics_mesh = built_mesh.value;
        model_desc.mesh_physics_assets.push_back(std::move(mesh_asset));

        skin_physics::runtime::world_desc world_desc;
        world_desc.fixed_delta_time = 0.01;
        world_desc.contact_max_substep_delta_time = 0.004;
        world_desc.maximum_in_flight_ticks = 2u;
        world_desc.maximum_steps_per_advance = 2u;
        skin_physics::runtime::world_handle world;
        require(runtime.create_world(world_desc, &world).success(),
            "Predecessor failure World creation must succeed.");

        skin_physics::runtime::model_handle model;
        require(runtime.add_model(world, model_desc, &model).success(),
            "Collinear Render face is valid Render topology and must bind to the Tet Physics Mesh.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "Predecessor failure World must enter Playing.");

        skin_physics::runtime::advance_report advance;
        require(runtime.advance(world, 0.02, &advance).success() &&
            advance.submitted_tick_count == 2u &&
            advance.in_flight_tick_count == 2u &&
            advance.pending_tick_count == 0u,
            "The failing head Tick and its dependent successor must both be submitted before asynchronous Status observation.");

        skin_physics::runtime::world_status result;
        bool all_queries_observed = false;
        for (uint32_t attempt = 0u; attempt < 2048u; ++attempt)
        {
            context->Flush();
            require(runtime.poll(world, &result).success(),
                "Predecessor failure Status polling must remain non-blocking and successful.");
            if (result.in_flight_tick_count == 0u)
            {
                all_queries_observed = true;
                break;
            }
            std::this_thread::yield();
        }

        require(all_queries_observed,
            "Both the head Tick failure and the successor predecessor gate must be observed.");
        require(result.state == skin_physics::runtime::tick_state::failed &&
            result.last_tick_status == status_code::gpu_execution_failed,
            "A Render Candidate numerical failure in the head Tick must remain the World terminal result after the successor reports BLOCKED_BY_PREDECESSOR.");
        require(result.head_input_snapshot_version != 0u,
            "A non-COMMITTED In-Flight Tick must retain its original Input Snapshot after Reservation restoration.");
        require(result.gpu_committed_sequence == 0u &&
            result.cpu_observed_sequence == 0u &&
            result.in_flight_tick_count == 0u &&
            result.head_reserved_tick_sequence == 1u &&
            result.pending_tick_count == 2u &&
            result.head_input_snapshot_version != 0u &&
            std::abs(result.reserved_tick_time - 0.02) <= 1.0e-12,
            "Both non-committed Reservations must be retained in ascending Sequence order with full reserved time and no published State Bank.");

        skin_physics::runtime::world_status repeated;
        require(runtime.poll(world, &repeated).success() &&
            repeated.state == skin_physics::runtime::tick_state::failed &&
            repeated.last_tick_status == status_code::gpu_execution_failed &&
            repeated.head_reserved_tick_sequence == 1u &&
            repeated.pending_tick_count == 2u &&
            repeated.head_input_snapshot_version ==
                result.head_input_snapshot_version,
            "A subsequent Poll must preserve the original FAILED result and the ordered diagnostic Reservations.");

        skin_physics::runtime::advance_report rejected_advance;
        require(runtime.advance(world, 0.0, &rejected_advance).code ==
                status_code::invalid_state,
            "A terminal FAILED World must not re-submit either retained Reservation.");

        require(runtime.reset_world(world).success(),
            "Reset must explicitly discard both retained non-committed Reservations.");
        require(runtime.destroy_model(world, model).success(),
            "Reset predecessor failure Model must release State and Candidate Ring resources.");
        require(runtime.destroy_world(world).success(),
            "Reset predecessor failure World must be destroyable.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the predecessor failure regression.");
    }

    void test_runtime_soft_body_cfl_preflight_and_budget_recovery()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Soft Body CFL regression requires a D3D11 WARP Feature Level 11_0 device.");

        physics_runtime runtime;
        skin_physics::runtime::runtime_desc runtime_desc;
        runtime_desc.shader_package_root = runtime_shader_root();
        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).success(),
            "Runtime initialization must compile the production Soft Body shader package.");

        skin_physics::physics::raw_tet_mesh raw_mesh;
        raw_mesh.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f }
        };
        raw_mesh.tetrahedra = { { 0u, 1u, 2u, 3u } };
        const skin_physics::physics::physics_mesh_build_result built_mesh =
            skin_physics::physics::build_physics_mesh(raw_mesh);
        require(built_mesh.success,
            "Soft Body CFL fixture must build a valid one-Tet Physics Mesh.");

        skin_physics::asset::mesh render_mesh;
        render_mesh.name = "runtime_soft_body_cfl_tet";
        render_mesh.mode = skin_physics::asset::physics_mode::soft_body;
        for (const skin_physics::asset::vec3& position : raw_mesh.positions)
        {
            skin_physics::asset::vertex vertex;
            vertex.position = position;
            render_mesh.vertices.push_back(vertex);
        }
        for (const skin_physics::physics::boundary_triangle& boundary :
            built_mesh.value.boundary_triangles)
        {
            render_mesh.indices.insert(
                render_mesh.indices.end(),
                boundary.vertices.begin(),
                boundary.vertices.end());
        }

        skin_physics::runtime::model_desc model_desc;
        model_desc.source_model.meshes.push_back(render_mesh);
        skin_physics::runtime::model_mesh_physics_asset mesh_asset;
        mesh_asset.mesh_index = 0u;
        mesh_asset.physics_mesh = built_mesh.value;
        model_desc.mesh_physics_assets.push_back(std::move(mesh_asset));

        skin_physics::runtime::world_desc world_desc;
        world_desc.fixed_delta_time = 0.01;
        world_desc.world_max_substep_delta_time = 0.0;
        world_desc.contact_max_substep_delta_time = 0.0;
        world_desc.minimum_substeps = 1u;
        world_desc.maximum_substeps = 4u;
        world_desc.maximum_integration_segments_per_tick = 512u;
        world_desc.maximum_steps_per_advance = 1u;
        world_desc.maximum_in_flight_ticks = 1u;
        skin_physics::runtime::world_handle world;
        require(runtime.create_world(world_desc, &world).success(),
            "Soft Body CFL World creation must succeed.");

        skin_physics::runtime::model_handle model;
        require(runtime.add_model(world, model_desc, &model).success(),
            "CFL fixture Soft Body Model must upload its Tet and attribute data.");

        skin_physics::runtime::mesh_physics_parameters parameters;
        parameters.density = 1000.0f;
        parameters.young_modulus = 1.0e9f;
        parameters.poisson_ratio = 0.3f;
        parameters.damping = 0.0f;
        skin_physics::runtime::mesh_physics_parameters invalid_tracking_parameters =
            parameters;
        invalid_tracking_parameters.tracking_stiffness =
            std::numeric_limits<float>::quiet_NaN();
        require(runtime.set_mesh_physics_parameters(
                world, model, 0u, invalid_tracking_parameters).code ==
                    status_code::invalid_argument,
            "Animation Tracking stiffness must reject NaN before it can enter CPU Preflight.");
        invalid_tracking_parameters = parameters;
        invalid_tracking_parameters.tracking_damping = -1.0f;
        require(runtime.set_mesh_physics_parameters(
                world, model, 0u, invalid_tracking_parameters).code ==
                    status_code::invalid_argument,
            "Animation Tracking damping must reject negative values.");

        require(runtime.set_mesh_physics_parameters(
                world, model, 0u, parameters).success(),
            "High-stiffness FEM CFL material must be accepted before Tick Reservation.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "CFL World must enter Playing.");

        skin_physics::runtime::advance_report report;
        require(runtime.advance(world, 0.01, &report).success(),
            "CFL-limited Advance must retain the accepted requested time.");
        require(report.submitted_tick_count == 0u &&
            report.in_flight_tick_count == 0u &&
            report.cpu_preflight_blocked_tick_count == 1u &&
            report.pending_tick_count == 1u,
            "High Soft Body stiffness must block the Tick in CPU Preflight without GPU submission.");

        skin_physics::runtime::world_status blocked;
        require(runtime.poll(world, &blocked).success() &&
            blocked.state == skin_physics::runtime::tick_state::blocked &&
            blocked.last_tick_status == status_code::work_budget_exceeded &&
            blocked.budget_block_origin ==
                skin_physics::runtime::work_budget_block_origin::cpu_preflight &&
            blocked.planned_substeps > world_desc.maximum_substeps &&
            blocked.head_reserved_tick_sequence == 1u &&
            std::abs(blocked.reserved_tick_time - 0.01) <= 1.0e-12,
            "FEM CFL must increase the common World Substep requirement and preserve the exact blocked Reservation.");

        skin_physics::runtime::world_execution_budget recovered_budget;
        recovered_budget.maximum_substeps = 512u;
        recovered_budget.maximum_integration_segments_per_tick = 512u;
        require(runtime.set_world_execution_budget(
                world, recovered_budget).success(),
            "World budget must be increased to satisfy the CFL-derived Substep count.");

        skin_physics::runtime::world_status recovered;
        require(runtime.poll(world, &recovered).success() &&
            recovered.state == skin_physics::runtime::tick_state::pending &&
            recovered.last_tick_status == status_code::ok &&
            recovered.execution_budget_revision == 2u &&
            recovered.planned_substeps == blocked.planned_substeps &&
            recovered.head_reserved_tick_sequence == 1u &&
            std::abs(recovered.reserved_tick_time - 0.01) <= 1.0e-12,
            "CFL budget recovery must preserve the same Substep plan, Reservation sequence and reserved time.");

        require(runtime.reset_world(world).success(),
            "Reset must release the pending FEM CFL-blocked Reservation.");
        require(runtime.destroy_model(world, model).success(),
            "Reset FEM CFL Model must release its GPU resources.");
        require(runtime.destroy_world(world).success(),
            "Reset FEM CFL World must be destroyable.");

        skin_physics::runtime::world_desc tracking_world_desc = world_desc;
        tracking_world_desc.maximum_substeps = 4u;
        skin_physics::runtime::world_handle tracking_world;
        require(runtime.create_world(tracking_world_desc, &tracking_world).success(),
            "Animation Tracking stability World creation must succeed.");
        skin_physics::runtime::model_handle tracking_model;
        require(runtime.add_model(
                tracking_world, model_desc, &tracking_model).success(),
            "Animation Tracking stability Model must upload its rest masses and Paint weights.");

        skin_physics::runtime::mesh_physics_parameters tracking_parameters;
        tracking_parameters.density = 1000.0f;
        tracking_parameters.young_modulus = 1.0f;
        tracking_parameters.poisson_ratio = 0.3f;
        tracking_parameters.damping = 0.0f;
        tracking_parameters.tracking_stiffness = 1.0e6f;
        tracking_parameters.tracking_damping = 100.0f;
        require(runtime.set_mesh_physics_parameters(
                tracking_world, tracking_model, 0u, tracking_parameters).success(),
            "Positive Animation Tracking stiffness / damping must be accepted.");

        skin_physics::runtime::transform tracking_sample;
        tracking_sample.translation = { 0.1f, 0.0f, 0.0f };
        require(runtime.set_model_transform_sample(
                tracking_world, tracking_model, 0.0,
                tracking_sample, false).success(),
            "An immutable Root Transform Sample must activate Animation Tracking stability preflight.");

        require(runtime.set_playback_state(
                tracking_world, skin_physics::runtime::playback_state::playing).success(),
            "Animation Tracking stability World must enter Playing.");
        skin_physics::runtime::advance_report tracking_report;
        require(runtime.advance(tracking_world, 0.01, &tracking_report).success() &&
            tracking_report.submitted_tick_count == 0u &&
            tracking_report.in_flight_tick_count == 0u &&
            tracking_report.cpu_preflight_blocked_tick_count == 1u &&
            tracking_report.pending_tick_count == 1u,
            "Animation Tracking stiffness must contribute to CPU Preflight before any GPU Submission.");
        skin_physics::runtime::world_status tracking_blocked;
        require(runtime.poll(tracking_world, &tracking_blocked).success() &&
            tracking_blocked.state == skin_physics::runtime::tick_state::blocked &&
            tracking_blocked.last_tick_status == status_code::work_budget_exceeded &&
            tracking_blocked.budget_block_origin ==
                skin_physics::runtime::work_budget_block_origin::cpu_preflight &&
            tracking_blocked.planned_substeps > tracking_world_desc.maximum_substeps &&
            tracking_blocked.head_reserved_tick_sequence == 1u &&
            std::abs(tracking_blocked.reserved_tick_time - 0.01) <= 1.0e-12,
            "High Animation Tracking stiffness must reduce the common World dt limit and preserve the blocked Reservation.");

        require(runtime.reset_world(tracking_world).success(),
            "Reset must release the Animation Tracking CFL Reservation and its Root Transform Sample.");
        skin_physics::runtime::attribute_edit_desc disable_tracking_paint;
        disable_tracking_paint.channel =
            skin_physics::asset::physics_attribute_channel::animation_tracking;
        disable_tracking_paint.vertex_indices = { 0u, 1u, 2u, 3u };
        disable_tracking_paint.target = 0.0f;
        disable_tracking_paint.strength = 1.0f;
        require(runtime.apply_attribute_edit(
                tracking_world, tracking_model, 0u,
                disable_tracking_paint).success(),
            "Animation Tracking Paint must be able to disable the stiffness contribution on every Physics Vertex.");
        require(runtime.set_mesh_physics_parameters(
                tracking_world, tracking_model, 0u, tracking_parameters).success(),
            "Animation Tracking material coefficients must survive Reset and remain available after Paint updates.");
        require(runtime.set_model_transform_sample(
                tracking_world, tracking_model, 0.0,
                tracking_sample, false).success(),
            "Animation Tracking Sample must be re-established after the Paint edit.");
        require(runtime.set_playback_state(
                tracking_world, skin_physics::runtime::playback_state::playing).success(),
            "Animation Tracking World must resume Playing after Reset.");

        skin_physics::runtime::advance_report painted_tracking_report;
        require(runtime.advance(
                tracking_world, 0.01, &painted_tracking_report).success() &&
            painted_tracking_report.submitted_tick_count == 0u &&
            painted_tracking_report.pending_tick_count == 1u,
            "The trajectory remains deferred while its painted-out Tracking stiffness is excluded from Preflight.");
        skin_physics::runtime::world_status painted_tracking_status;
        require(runtime.poll(tracking_world, &painted_tracking_status).success() &&
            painted_tracking_status.state == skin_physics::runtime::tick_state::blocked &&
            painted_tracking_status.last_tick_status == status_code::blocked_by_input &&
            painted_tracking_status.planned_substeps == 1u &&
            painted_tracking_status.head_reserved_tick_sequence == 1u,
            "Zero Animation Tracking Paint must remove the Tracking frequency limit while preserving the explicit BLOCKED_BY_INPUT trajectory gate.");

        require(runtime.reset_world(tracking_world).success(),
            "Reset must release the deferred Animation Trajectory Reservation.");
        require(runtime.destroy_model(tracking_world, tracking_model).success(),
            "Reset Animation Tracking Model must release its GPU resources.");
        require(runtime.destroy_world(tracking_world).success(),
            "Reset Animation Tracking World must be destroyable.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the FEM / Animation Tracking stability preflight regressions.");
    }

    void test_runtime_cpu_preflight_count_overflow_fails_world()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Runtime regression requires a D3D11 WARP Feature Level 11_0 device.");

        physics_runtime runtime;
        skin_physics::runtime::runtime_desc runtime_desc;
        runtime_desc.shader_package_root = runtime_shader_root();
        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).success(),
            "Runtime initialization must succeed for the CPU Preflight overflow regression.");

        skin_physics::runtime::world_desc desc;
        desc.fixed_delta_time = 0.01;
        desc.contact_max_substep_delta_time = 1.0e-12;
        desc.maximum_in_flight_ticks = 1u;
        skin_physics::runtime::world_handle world;
        require(runtime.create_world(desc, &world).success(),
            "Overflow regression World creation must accept finite positive temporal limits.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "Overflow regression World must enter Playing.");

        skin_physics::runtime::advance_report report;
        require(runtime.advance(world, 0.01, &report).success(),
            "Advance must preserve the requested Tick while CPU Preflight classifies its unrepresentable work count.");
        require(report.submitted_tick_count == 0u &&
            report.in_flight_tick_count == 0u &&
            report.pending_tick_count == 1u &&
            report.cpu_preflight_blocked_tick_count == 0u,
            "An unrepresentable work count must not submit GPU work or masquerade as a retryable CPU budget block.");

        skin_physics::runtime::world_status result;
        require(runtime.poll(world, &result).success() &&
            result.state == skin_physics::runtime::tick_state::failed &&
            result.last_tick_status == status_code::capacity_exceeded &&
            result.budget_block_origin ==
                skin_physics::runtime::work_budget_block_origin::none &&
            result.budget_resolution ==
                skin_physics::runtime::work_budget_resolution::none &&
            result.work_budget_block_flags == 0u &&
            result.head_reserved_tick_sequence == 1u &&
            std::abs(result.reserved_tick_time - 0.01) <= 1.0e-12,
            "A uint32 work-count overflow must enter terminal FAILED while retaining the diagnostic Reservation and its exact time.");

        require(runtime.reset_world(world).success(),
            "Reset must explicitly clear a terminal CPU Preflight count-overflow failure.");
        require(runtime.destroy_world(world).success(),
            "World must be destroyable after resetting the terminal overflow failure.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the CPU Preflight overflow regression.");
    }

    void test_runtime_submission_thread_guard()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(create_warp_device(device, context),
            "Runtime regression requires a D3D11 WARP Feature Level 11_0 device.");

        physics_runtime runtime;
        skin_physics::runtime::runtime_desc runtime_desc;
        runtime_desc.shader_package_root = runtime_shader_root();
        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).success(),
            "Runtime initialization must succeed for the thread guard regression.");

        skin_physics::runtime::world_handle world;
        require(runtime.create_world({}, &world).success(),
            "Thread guard World creation must succeed.");

        status_code worker_status = status_code::ok;
        std::thread worker([&runtime, world, &worker_status]()
        {
            const skin_physics::runtime::status result =
                runtime.set_time_scale(world, 0.5);
            worker_status = result.code;
        });
        worker.join();

        require(worker_status == status_code::submission_thread_violation,
            "Runtime APIs that own Immediate Context scheduling must reject calls from a non-submission thread.");

        require(runtime.destroy_world(world).success(),
            "World must remain intact after a rejected non-submission-thread API call.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the thread guard regression.");
    }
}

int run_runtime_tests()
{
    test_runtime_force_distribution_gpu_cpu_parity();
    std::cout << "[PASS] R1_WARP_FORCE_DISTRIBUTION_CPU_GPU_WEIGHT_PARITY\n";
    test_runtime_initialization_and_world_handles();
    std::cout << "[PASS] R0_RUNTIME_INITIALIZATION_AND_HANDLES\n";
    test_runtime_time_budget_and_pause_contract();
    std::cout << "[PASS] R0_RUNTIME_TIME_BUDGET_AND_PAUSE\n";
    test_runtime_submission_thread_guard();
    std::cout << "[PASS] R0_RUNTIME_SUBMISSION_THREAD_GUARD\n";
    test_runtime_cpu_preflight_reservation_retry();
    std::cout << "[PASS] R0_RUNTIME_CPU_PREFLIGHT_RESERVATION_RETRY\n";
    test_runtime_cpu_preflight_count_overflow_fails_world();
    std::cout << "[PASS] R0_RUNTIME_CPU_PREFLIGHT_COUNT_OVERFLOW\n";
    test_runtime_soft_body_gpu_tick_and_state_bank_sequence();
    std::cout << "[PASS] R1_WARP_SOFT_BODY_RUNTIME_STATE_BANKS\n";
    std::cout << "[PASS] R1_WARP_SOFT_BODY_SEGMENT_DT_PARAMETERS\n";
    std::cout << "[PASS] R1_RUNTIME_ANIMATION_INPUT_DEFERRED\n";
    std::cout << "[PASS] R1_WARP_SOFT_BODY_FORCE_IMPULSE_AND_NO_SUPPORT\n";
    std::cout << "[PASS] R1_WARP_SOFT_BODY_CROSS_MESH_FORCE_TARGET_SCOPES\n";
    std::cout << "[PASS] R1_WARP_SOFT_BODY_RENDER_CANDIDATE_LEASE\n";
    test_runtime_multiple_in_flight_non_commit_preserves_head_sequence();
    std::cout << "[PASS] R0_WARP_PREDECESSOR_FAILURE_ORDER\n";
    test_runtime_soft_body_cfl_preflight_and_budget_recovery();
    std::cout << "[PASS] R1_WARP_SOFT_BODY_CFL_PREFLIGHT\n";
    std::cout << "[PASS] R1_WARP_SOFT_BODY_ANIMATION_TRACKING_STABILITY_PREFLIGHT\n";
    std::cout << "RuntimeTests passed.\n";
    return 0;
}
