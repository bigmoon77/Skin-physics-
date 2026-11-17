#include "skin_physics/physics/cloth_solver.hpp"
#include "test_support.hpp"
#include "json_test_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>

namespace
{
    using Microsoft::WRL::ComPtr;
    using skin_physics::asset::vec3;

    struct alignas(16) gpu_float4
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 0.0f;
    };

    struct gpu_cloth_constraint
    {
        uint32_t indices[4] = {};
        uint32_t type = 0;
        float rest_value = 0.0f;
        float normalization = 0.0f;
        float compliance = 0.0f;
    };

    struct gpu_cloth_contribution
    {
        gpu_float4 correction0;
        gpu_float4 correction1;
        gpu_float4 correction2;
        gpu_float4 correction3;
        gpu_float4 lambda;
    };

    struct alignas(16) gpu_step_parameters
    {
        float delta_time = 0.02f;
        float relaxation = 1.0f;
        uint32_t constraint_count = 0;
        uint32_t reserved = 0;
    };

    void require(bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    HRESULT create_warp_device(
        ID3D11Device** device,
        ID3D11DeviceContext** context)
    {
        D3D_FEATURE_LEVEL requested[] = { D3D_FEATURE_LEVEL_11_0 };
        D3D_FEATURE_LEVEL selected = D3D_FEATURE_LEVEL_9_1;
        return D3D11CreateDevice(
            nullptr,
            D3D_DRIVER_TYPE_WARP,
            nullptr,
            0,
            requested,
            1,
            D3D11_SDK_VERSION,
            device,
            &selected,
            context);
    }

    ComPtr<ID3D11ComputeShader> compile_compute_shader(
        ID3D11Device* device,
        const std::filesystem::path& path,
        const char* entry_point)
    {
        ComPtr<ID3DBlob> bytecode;
        ComPtr<ID3DBlob> errors;
        const HRESULT compile_result = D3DCompileFromFile(
            path.c_str(),
            nullptr,
            D3D_COMPILE_STANDARD_FILE_INCLUDE,
            entry_point,
            "cs_5_0",
            D3DCOMPILE_ENABLE_STRICTNESS,
            0,
            bytecode.GetAddressOf(),
            errors.GetAddressOf());
        if (FAILED(compile_result))
        {
            std::string diagnostic;
            if (errors != nullptr)
            {
                const char* begin = static_cast<const char*>(errors->GetBufferPointer());
                diagnostic.assign(begin, begin + errors->GetBufferSize());
            }
            throw std::runtime_error("Compute shader compilation failed: " + diagnostic);
        }

        ComPtr<ID3D11ComputeShader> shader;
        const HRESULT create_result = device->CreateComputeShader(
            bytecode->GetBufferPointer(),
            bytecode->GetBufferSize(),
            nullptr,
            shader.GetAddressOf());
        require(SUCCEEDED(create_result), "Compute shader creation failed.");
        return shader;
    }

    template <typename T>
    ComPtr<ID3D11Buffer> create_structured_buffer(
        ID3D11Device* device,
        const std::vector<T>& values,
        UINT bind_flags)
    {
        require(!values.empty(), "Structured test buffer must contain at least one element.");
        D3D11_BUFFER_DESC description = {};
        description.ByteWidth = static_cast<UINT>(values.size() * sizeof(T));
        description.Usage = D3D11_USAGE_DEFAULT;
        description.BindFlags = bind_flags;
        description.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
        description.StructureByteStride = sizeof(T);
        D3D11_SUBRESOURCE_DATA initial_data = {};
        initial_data.pSysMem = values.data();

        ComPtr<ID3D11Buffer> buffer;
        const HRESULT result = device->CreateBuffer(
            &description, &initial_data, buffer.GetAddressOf());
        require(SUCCEEDED(result), "Structured GPU buffer creation failed.");
        return buffer;
    }

    template <typename T>
    ComPtr<ID3D11ShaderResourceView> create_structured_srv(
        ID3D11Device* device,
        ID3D11Buffer* buffer,
        UINT element_count)
    {
        D3D11_SHADER_RESOURCE_VIEW_DESC description = {};
        description.Format = DXGI_FORMAT_UNKNOWN;
        description.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
        description.Buffer.FirstElement = 0;
        description.Buffer.NumElements = element_count;

        ComPtr<ID3D11ShaderResourceView> view;
        const HRESULT result = device->CreateShaderResourceView(
            buffer, &description, view.GetAddressOf());
        require(SUCCEEDED(result), "Structured SRV creation failed.");
        return view;
    }

    template <typename T>
    ComPtr<ID3D11UnorderedAccessView> create_structured_uav(
        ID3D11Device* device,
        ID3D11Buffer* buffer,
        UINT element_count)
    {
        D3D11_UNORDERED_ACCESS_VIEW_DESC description = {};
        description.Format = DXGI_FORMAT_UNKNOWN;
        description.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
        description.Buffer.FirstElement = 0;
        description.Buffer.NumElements = element_count;

        ComPtr<ID3D11UnorderedAccessView> view;
        const HRESULT result = device->CreateUnorderedAccessView(
            buffer, &description, view.GetAddressOf());
        require(SUCCEEDED(result), "Structured UAV creation failed.");
        return view;
    }

    ComPtr<ID3D11Buffer> create_constant_buffer(
        ID3D11Device* device,
        const void* data,
        UINT byte_width)
    {
        require((byte_width & 15u) == 0u, "Constant buffer size must be 16-byte aligned.");
        D3D11_BUFFER_DESC description = {};
        description.ByteWidth = byte_width;
        description.Usage = D3D11_USAGE_DEFAULT;
        description.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        D3D11_SUBRESOURCE_DATA initial_data = {};
        initial_data.pSysMem = data;

        ComPtr<ID3D11Buffer> buffer;
        const HRESULT result = device->CreateBuffer(
            &description, &initial_data, buffer.GetAddressOf());
        require(SUCCEEDED(result), "Constant buffer creation failed.");
        return buffer;
    }

    template <typename T>
    std::vector<T> readback_structured_buffer(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11Buffer* source,
        std::size_t count)
    {
        const UINT byte_width = static_cast<UINT>(count * sizeof(T));
        D3D11_BUFFER_DESC description = {};
        description.ByteWidth = byte_width;
        description.Usage = D3D11_USAGE_STAGING;
        description.CPUAccessFlags = D3D11_CPU_ACCESS_READ;

        ComPtr<ID3D11Buffer> staging;
        HRESULT result = device->CreateBuffer(
            &description, nullptr, staging.GetAddressOf());
        require(SUCCEEDED(result), "GPU staging buffer creation failed.");
        context->CopyResource(staging.Get(), source);

        D3D11_MAPPED_SUBRESOURCE mapped = {};
        result = context->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &mapped);
        require(SUCCEEDED(result), "GPU staging Readback Map failed.");
        std::vector<T> output(count);
        std::memcpy(output.data(), mapped.pData, byte_width);
        context->Unmap(staging.Get(), 0);
        return output;
    }

    void dispatch_compute(
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* shader,
        const std::vector<ID3D11ShaderResourceView*>& resources,
        const std::vector<ID3D11UnorderedAccessView*>& unordered_views,
        ID3D11Buffer* constants,
        UINT group_count)
    {
        context->CSSetShader(shader, nullptr, 0);
        context->CSSetShaderResources(
            0, static_cast<UINT>(resources.size()), resources.data());
        context->CSSetUnorderedAccessViews(
            0, static_cast<UINT>(unordered_views.size()),
            unordered_views.data(), nullptr);
        if (constants != nullptr)
        {
            context->CSSetConstantBuffers(0, 1, &constants);
        }
        context->Dispatch(group_count, 1, 1);

        std::vector<ID3D11ShaderResourceView*> null_resources(resources.size(), nullptr);
        std::vector<ID3D11UnorderedAccessView*> null_unordered(unordered_views.size(), nullptr);
        context->CSSetShaderResources(
            0, static_cast<UINT>(null_resources.size()), null_resources.data());
        context->CSSetUnorderedAccessViews(
            0, static_cast<UINT>(null_unordered.size()),
            null_unordered.data(), nullptr);
        if (constants != nullptr)
        {
            ID3D11Buffer* null_constants[] = { nullptr };
            context->CSSetConstantBuffers(0, 1, null_constants);
        }
        context->CSSetShader(nullptr, nullptr, 0);
    }

    vec3 read_vec3(const test_json::value& value)
    {
        return {
            static_cast<float>(value.at(0).as_number()),
            static_cast<float>(value.at(1).as_number()),
            static_cast<float>(value.at(2).as_number())
        };
    }

    skin_physics::physics::cloth_mesh load_cloth_fixture(
        const test_json::value& input)
    {
        skin_physics::physics::raw_cloth_mesh raw;
        for (const test_json::value& point : input.at("positions").as_array())
        {
            raw.rest_positions.push_back(read_vec3(point));
        }
        for (const test_json::value& triangle : input.at("triangles").as_array())
        {
            raw.triangles.push_back({
                static_cast<uint32_t>(triangle.at(0).as_number()),
                static_cast<uint32_t>(triangle.at(1).as_number()),
                static_cast<uint32_t>(triangle.at(2).as_number())
            });
        }

        const auto built = skin_physics::physics::build_cloth_mesh(raw);
        require(built.success, "Cloth GPU fixture must build using the CPU topology reference.");
        return built.value;
    }

    std::vector<gpu_cloth_constraint> make_gpu_constraints(
        const skin_physics::physics::cloth_mesh& mesh,
        uint32_t type,
        float compliance)
    {
        std::vector<gpu_cloth_constraint> output;
        if (type == 0)
        {
            for (const auto& source : mesh.stretch_constraints)
            {
                gpu_cloth_constraint item;
                item.indices[0] = source.vertices[0];
                item.indices[1] = source.vertices[1];
                item.type = type;
                item.rest_value = source.rest_length;
                item.compliance = compliance;
                output.push_back(item);
            }
        }
        else if (type == 1)
        {
            for (const auto& source : mesh.shear_constraints)
            {
                gpu_cloth_constraint item;
                item.indices[0] = source.vertices[0];
                item.indices[1] = source.vertices[1];
                item.indices[2] = source.vertices[2];
                item.indices[3] = source.vertices[2];
                item.type = type;
                item.rest_value = source.rest_dot;
                item.normalization = source.normalization_length;
                item.compliance = compliance;
                output.push_back(item);
            }
        }
        else
        {
            for (const auto& source : mesh.bend_constraints)
            {
                gpu_cloth_constraint item;
                for (std::size_t index = 0; index < 4; ++index)
                {
                    item.indices[index] = source.vertices[index];
                }
                item.type = type;
                item.rest_value = source.rest_angle;
                item.compliance = compliance;
                output.push_back(item);
            }
        }
        return output;
    }

    void test_phase05_warp_xpbd_constraints(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* shader,
        const test_json::value& input,
        const skin_physics::physics::cloth_mesh& mesh,
        uint32_t type,
        const char* label)
    {
        std::vector<vec3> source_positions = mesh.rest_positions;
        if (type == 0)
        {
            source_positions[1].x += 0.2f;
            source_positions[3].z += 0.1f;
        }
        else if (type == 1)
        {
            source_positions[1].y += 0.3f;
            source_positions[2].x += 0.1f;
        }
        else
        {
            source_positions[3].z += 0.23f;
        }

        const char* compliance_name =
            type == 0 ? "stretch" : (type == 1 ? "shear" : "bend");
        const float compliance = static_cast<float>(
            input.at("compliance").at(compliance_name).as_number());
        const float delta_time = static_cast<float>(
            input.at("fixed_delta_time").as_number());
        const std::vector<gpu_cloth_constraint> constraints =
            make_gpu_constraints(mesh, type, compliance);
        require(!constraints.empty(), "Every XPBD GPU pass needs at least one constraint.");

        std::vector<gpu_float4> gpu_positions;
        gpu_positions.reserve(source_positions.size());
        for (const vec3& value : source_positions)
        {
            gpu_positions.push_back({ value.x, value.y, value.z, 0.0f });
        }
        std::vector<float> inverse_masses(source_positions.size(), 1.0f);
        std::vector<gpu_cloth_contribution> initial_contributions(
            constraints.size(), gpu_cloth_contribution{});

        const ComPtr<ID3D11Buffer> positions_buffer =
            create_structured_buffer(device, gpu_positions, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> masses_buffer =
            create_structured_buffer(device, inverse_masses, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> constraints_buffer =
            create_structured_buffer(device, constraints, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> contribution_buffer =
            create_structured_buffer(device, initial_contributions, D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11ShaderResourceView> positions_view =
            create_structured_srv<gpu_float4>(device, positions_buffer.Get(),
                static_cast<UINT>(gpu_positions.size()));
        const ComPtr<ID3D11ShaderResourceView> masses_view =
            create_structured_srv<float>(device, masses_buffer.Get(),
                static_cast<UINT>(inverse_masses.size()));
        const ComPtr<ID3D11ShaderResourceView> constraints_view =
            create_structured_srv<gpu_cloth_constraint>(device, constraints_buffer.Get(),
                static_cast<UINT>(constraints.size()));
        const ComPtr<ID3D11UnorderedAccessView> contribution_view =
            create_structured_uav<gpu_cloth_contribution>(device, contribution_buffer.Get(),
                static_cast<UINT>(constraints.size()));

        gpu_step_parameters parameters;
        parameters.delta_time = delta_time;
        parameters.relaxation = 1.0f;
        parameters.constraint_count = static_cast<uint32_t>(constraints.size());
        const ComPtr<ID3D11Buffer> constant_buffer =
            create_constant_buffer(device, &parameters, sizeof(parameters));

        dispatch_compute(
            context, shader,
            { positions_view.Get(), masses_view.Get(), constraints_view.Get() },
            { contribution_view.Get() },
            constant_buffer.Get(),
            static_cast<UINT>((constraints.size() + 63u) / 64u));

        const std::vector<gpu_cloth_contribution> gpu_contributions =
            readback_structured_buffer<gpu_cloth_contribution>(
                device, context, contribution_buffer.Get(), constraints.size());

        skin_physics::physics::cloth_solver_settings cpu_settings;
        cpu_settings.fixed_delta_time = delta_time;
        cpu_settings.iterations = 1;
        cpu_settings.relaxation = 1.0f;
        cpu_settings.solve_stretch = type == 0;
        cpu_settings.solve_shear = type == 1;
        cpu_settings.solve_bend = type == 2;
        cpu_settings.stretch_compliance = type == 0 ? compliance : 0.0f;
        cpu_settings.shear_compliance = type == 1 ? compliance : 0.0f;
        cpu_settings.bend_compliance = type == 2 ? compliance : 0.0f;

        const auto cpu_result = skin_physics::physics::solve_cloth(
            mesh, source_positions, inverse_masses, cpu_settings);
        require(cpu_result.success, std::string(label) + " CPU reference solve failed.");

        std::vector<vec3> gathered = source_positions;
        for (std::size_t constraint_index = 0;
            constraint_index < constraints.size(); ++constraint_index)
        {
            const gpu_cloth_contribution& contribution =
                gpu_contributions[constraint_index];
            const gpu_float4 corrections[4] = {
                contribution.correction0,
                contribution.correction1,
                contribution.correction2,
                contribution.correction3
            };
            for (std::size_t local_index = 0; local_index < 4; ++local_index)
            {
                const uint32_t vertex_id = constraints[constraint_index].indices[local_index];
                gathered[vertex_id].x += corrections[local_index].x;
                gathered[vertex_id].y += corrections[local_index].y;
                gathered[vertex_id].z += corrections[local_index].z;
            }
        }

        for (std::size_t vertex_index = 0; vertex_index < gathered.size(); ++vertex_index)
        {
            const vec3& expected = cpu_result.positions[vertex_index];
            require(std::abs(gathered[vertex_index].x - expected.x) <= 2.0e-4f &&
                std::abs(gathered[vertex_index].y - expected.y) <= 2.0e-4f &&
                std::abs(gathered[vertex_index].z - expected.z) <= 2.0e-4f,
                std::string(label) + " WARP XPBD contributions must match the CPU reference.");
        }
    }

    void test_phase05_warp_cloth_all_constraint_types()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result), "D3D11 WARP device creation must succeed.");

        const test_json::value input = test_json::load(
            test_support::fixture_path("phase05_cloth/cloth_patch.input.json"));
        const skin_physics::physics::cloth_mesh mesh = load_cloth_fixture(input);
        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_path("phase05_07_gpu/cloth_constraints.cs.hlsl"),
            "main");

        test_phase05_warp_xpbd_constraints(
            device.Get(), context.Get(), shader.Get(), input, mesh, 0, "Stretch");
        std::cout << "[PASS] P05_WARP_STRETCH\n";
        test_phase05_warp_xpbd_constraints(
            device.Get(), context.Get(), shader.Get(), input, mesh, 1, "Shear");
        std::cout << "[PASS] P05_WARP_SHEAR\n";
        test_phase05_warp_xpbd_constraints(
            device.Get(), context.Get(), shader.Get(), input, mesh, 2, "Bend");
        std::cout << "[PASS] P05_WARP_BEND\n";
    }
}

int run_gpu_solver_tests()
{
    test_phase05_warp_cloth_all_constraint_types();
    std::cout << "GpuSolverTests passed.\n";
    return 0;
}
