#include "skin_physics/physics/cloth_solver.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"
#include "skin_physics/physics/soft_body_fem.hpp"
#include "skin_physics/physics/collision.hpp"
#include "skin_physics/physics/animation_track.hpp"
#include "skin_physics/physics/physics_mesh_skinning.hpp"
#include "skin_physics/physics/animation_gpu_resources.hpp"
#include "skin_physics/physics/attribute_paint.hpp"
#include "skin_physics/physics/ccd.hpp"
#include "skin_physics/physics/correction_ccd.hpp"
#include "skin_physics/simulation/d3d11_timestamp_profiler.hpp"
#include "skin_physics/simulation/d3d11_compute_graph.hpp"
#include "test_support.hpp"
#include "json_test_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
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

    struct gpu_cloth_jacobi_contribution
    {
        gpu_float4 correction0;
        gpu_float4 correction1;
        gpu_float4 correction2;
        gpu_float4 correction3;
    };

    struct gpu_cloth_stretch_constraint
    {
        uint32_t vertices[2] = {};
        float rest_length = 0.0f;
        float reserved = 0.0f;
    };

    struct gpu_cloth_vertex_constraint_ref
    {
        uint32_t type = 0u;
        uint32_t constraint_index = 0u;
        uint32_t local_vertex_index = 0u;
        uint32_t reserved = 0u;
    };

    struct alignas(16) gpu_cloth_jacobi_parameters
    {
        uint32_t constraint_count = 0u;
        uint32_t vertex_count = 0u;
        float delta_time = 0.0f;
        float relaxation = 1.0f;
    };

    struct alignas(16) gpu_cloth_jacobi_gather_parameters
    {
        uint32_t vertex_count = 0u;
        uint32_t stretch_constraint_count = 0u;
        uint32_t shear_constraint_count = 0u;
        uint32_t bend_constraint_count = 0u;
        uint32_t reference_count = 0u;
        uint32_t reserved[3] = {};
    };

    static_assert(sizeof(gpu_cloth_jacobi_contribution) == 64u,
        "Production Cloth Jacobi Contribution ABI must remain 64 bytes.");
    static_assert(sizeof(gpu_cloth_stretch_constraint) == 16u,
        "Production Cloth Stretch Constraint ABI must remain 16 bytes.");
    static_assert(sizeof(gpu_cloth_vertex_constraint_ref) == 16u,
        "Production Cloth Vertex Constraint CSR ABI must remain 16 bytes.");
    static_assert(sizeof(gpu_cloth_jacobi_parameters) == 16u,
        "Production Cloth Jacobi Contribution parameters must remain 16 bytes.");
    static_assert(sizeof(gpu_cloth_jacobi_gather_parameters) == 32u,
        "Production Cloth Jacobi Gather parameters must remain 32 bytes.");

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

    void require_immutable_structured_buffer(
        ID3D11Buffer* buffer,
        uint32_t element_count,
        uint32_t element_stride,
        const std::string& label)
    {
        require(buffer != nullptr, label + " GPU buffer must exist.");
        D3D11_BUFFER_DESC description = {};
        buffer->GetDesc(&description);
        require(description.Usage == D3D11_USAGE_IMMUTABLE &&
            description.BindFlags == D3D11_BIND_SHADER_RESOURCE &&
            description.CPUAccessFlags == 0u &&
            description.MiscFlags == D3D11_RESOURCE_MISC_BUFFER_STRUCTURED &&
            description.StructureByteStride == element_stride &&
            description.ByteWidth == element_count * element_stride,
            label + " must use an immutable SRV-only StructuredBuffer with the exact record stride and count.");
    }

    void require_structured_srv_element_count(
        ID3D11ShaderResourceView* view,
        uint32_t element_count,
        const std::string& label)
    {
        require(view != nullptr, label + " SRV must exist.");
        D3D11_SHADER_RESOURCE_VIEW_DESC description = {};
        view->GetDesc(&description);
        require(description.Format == DXGI_FORMAT_UNKNOWN &&
            description.ViewDimension == D3D11_SRV_DIMENSION_BUFFER &&
            description.Buffer.FirstElement == 0u &&
            description.Buffer.NumElements == element_count,
            label + " SRV must expose the complete immutable StructuredBuffer range.");
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

    void test_phase05_warp_colored_cloth_stretch()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(SUCCEEDED(create_warp_device(
                device.GetAddressOf(), context.GetAddressOf())),
            "Colored Cloth Stretch WARP test requires a Feature Level 11_0 device.");

        const skin_physics::physics::cloth_mesh mesh =
            load_cloth_fixture(test_json::load(
                test_support::fixture_path("phase05_cloth/cloth_patch.input.json")));
        std::string coloring_diagnostic;
        require(skin_physics::physics::validate_cloth_stretch_coloring(
                mesh, coloring_diagnostic),
            "Colored Cloth Stretch test requires valid CPU graph coloring: " +
                coloring_diagnostic);

        const std::filesystem::path shader_path =
            test_support::fixture_directory().parent_path().parent_path() /
            "shaders" / "runtime" / "cloth_stretch_solve.cs.hlsl";
        const ComPtr<ID3D11ComputeShader> shader =
            compile_compute_shader(device.Get(), shader_path, "main");

        struct gpu_cloth_stretch_constraint
        {
            uint32_t vertices[2] = {};
            float rest_length = 0.0f;
            float reserved = 0.0f;
        };
        static_assert(sizeof(gpu_cloth_stretch_constraint) == 16u,
            "Production Stretch Constraint ABI must remain 16 bytes.");
        struct alignas(16) gpu_cloth_stretch_parameters
        {
            uint32_t first_color_index = 0u;
            uint32_t color_constraint_count = 0u;
            uint32_t vertex_count = 0u;
            uint32_t constraint_count = 0u;
            float delta_time = 0.0f;
            float compliance = 0.0f;
            float relaxation = 0.0f;
            uint32_t reserved = 0u;
        };
        static_assert(sizeof(gpu_cloth_stretch_parameters) == 32u,
            "Production Stretch parameter ABI must remain 32 bytes.");

        std::vector<vec3> cpu_positions = mesh.rest_positions;
        cpu_positions[3].x += 0.2f;
        cpu_positions[4].z += 0.1f;
        std::vector<gpu_float4> gpu_positions;
        gpu_positions.reserve(cpu_positions.size());
        for (const vec3& position : cpu_positions)
        {
            gpu_positions.push_back(
                { position.x, position.y, position.z, 0.0f });
        }
        const std::vector<float> inverse_masses(
            cpu_positions.size(), 1.0f);
        std::vector<float> cpu_lambdas(
            mesh.stretch_constraints.size(), 0.0f);
        const std::vector<float> initial_lambdas = cpu_lambdas;
        const std::vector<uint32_t> initial_status = { 0u };

        std::vector<gpu_cloth_stretch_constraint> constraints;
        constraints.reserve(mesh.stretch_constraints.size());
        for (const auto& source : mesh.stretch_constraints)
        {
            gpu_cloth_stretch_constraint constraint;
            constraint.vertices[0] = source.vertices[0];
            constraint.vertices[1] = source.vertices[1];
            constraint.rest_length = source.rest_length;
            constraints.push_back(constraint);
        }

        const ComPtr<ID3D11Buffer> positions_buffer =
            create_structured_buffer(
                device.Get(), gpu_positions, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> inverse_mass_buffer =
            create_structured_buffer(
                device.Get(), inverse_masses, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> constraint_buffer =
            create_structured_buffer(
                device.Get(), constraints, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> color_index_buffer =
            create_structured_buffer(
                device.Get(),
                mesh.stretch_color_constraint_indices,
                D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> lambda_buffer =
            create_structured_buffer(
                device.Get(), initial_lambdas, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> status_buffer =
            create_structured_buffer(
                device.Get(), initial_status, D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11ShaderResourceView> inverse_mass_srv =
            create_structured_srv<float>(
                device.Get(), inverse_mass_buffer.Get(),
                static_cast<UINT>(inverse_masses.size()));
        const ComPtr<ID3D11ShaderResourceView> constraint_srv =
            create_structured_srv<gpu_cloth_stretch_constraint>(
                device.Get(), constraint_buffer.Get(),
                static_cast<UINT>(constraints.size()));
        const ComPtr<ID3D11ShaderResourceView> color_index_srv =
            create_structured_srv<uint32_t>(
                device.Get(), color_index_buffer.Get(),
                static_cast<UINT>(
                    mesh.stretch_color_constraint_indices.size()));
        const ComPtr<ID3D11UnorderedAccessView> positions_uav =
            create_structured_uav<gpu_float4>(
                device.Get(), positions_buffer.Get(),
                static_cast<UINT>(gpu_positions.size()));
        const ComPtr<ID3D11UnorderedAccessView> lambda_uav =
            create_structured_uav<float>(
                device.Get(), lambda_buffer.Get(),
                static_cast<UINT>(initial_lambdas.size()));
        const ComPtr<ID3D11UnorderedAccessView> status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), status_buffer.Get(), 1u);

        constexpr float delta_time = 1.0f / 120.0f;
        constexpr float compliance = 1.0e-5f;
        constexpr float relaxation = 0.8f;
        gpu_cloth_stretch_parameters parameters;
        parameters.vertex_count = static_cast<uint32_t>(gpu_positions.size());
        parameters.constraint_count =
            static_cast<uint32_t>(constraints.size());
        parameters.delta_time = delta_time;
        parameters.compliance = compliance;
        parameters.relaxation = relaxation;
        const ComPtr<ID3D11Buffer> parameter_buffer =
            create_constant_buffer(
                device.Get(), &parameters, sizeof(parameters));

        ID3D11ShaderResourceView* shader_resources[] = {
            inverse_mass_srv.Get(), constraint_srv.Get(),
            color_index_srv.Get()
        };
        ID3D11UnorderedAccessView* unordered_access_views[] = {
            positions_uav.Get(), lambda_uav.Get(), status_uav.Get()
        };
        ID3D11Buffer* constant_buffers[] = { parameter_buffer.Get() };

        for (uint32_t iteration = 0u; iteration < 4u; ++iteration)
        {
            for (uint32_t color_index = 0u;
                color_index < mesh.stretch_color_ranges.size();
                ++color_index)
            {
                const auto& color_range =
                    mesh.stretch_color_ranges[color_index];
                parameters.first_color_index =
                    color_range.first_constraint_index;
                parameters.color_constraint_count =
                    color_range.constraint_count;
                context->UpdateSubresource(
                    parameter_buffer.Get(), 0u, nullptr,
                    &parameters, 0u, 0u);

                ID3D11ShaderResourceView* current_resources[] = {
                    inverse_mass_srv.Get(), constraint_srv.Get(),
                    color_index_srv.Get()
                };
                context->CSSetShader(shader.Get(), nullptr, 0u);
                context->CSSetShaderResources(
                    0u, 3u, current_resources);
                context->CSSetUnorderedAccessViews(
                    0u, 3u, unordered_access_views, nullptr);
                context->CSSetConstantBuffers(
                    0u, 1u, constant_buffers);
                context->Dispatch(
                    (color_range.constraint_count + 63u) / 64u,
                    1u, 1u);

                const auto cpu_step =
                    skin_physics::physics::solve_cloth_stretch_color(
                        mesh,
                        color_index,
                        cpu_positions,
                        inverse_masses,
                        cpu_lambdas,
                        delta_time,
                        compliance,
                        relaxation);
                require(cpu_step.success,
                    "Colored Stretch CPU reference must accept each scheduled color.");
                cpu_positions = cpu_step.positions;
                cpu_lambdas = cpu_step.lambda_values;
            }
        }

        ID3D11ShaderResourceView* null_shader_resources[] = {
            nullptr, nullptr, nullptr
        };
        ID3D11UnorderedAccessView* null_unordered_access_views[] = {
            nullptr, nullptr, nullptr
        };
        ID3D11Buffer* null_constant_buffers[] = { nullptr };
        context->CSSetShaderResources(0u, 3u, null_shader_resources);
        context->CSSetUnorderedAccessViews(
            0u, 3u, null_unordered_access_views, nullptr);
        context->CSSetConstantBuffers(
            0u, 1u, null_constant_buffers);
        context->CSSetShader(nullptr, nullptr, 0u);
        context->Flush();

        const std::vector<gpu_float4> actual_positions =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(),
                positions_buffer.Get(), gpu_positions.size());
        const std::vector<float> actual_lambdas =
            readback_structured_buffer<float>(
                device.Get(), context.Get(),
                lambda_buffer.Get(), initial_lambdas.size());
        const std::vector<uint32_t> actual_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(actual_status[0u] == 0u,
            "Production colored Stretch shader must complete without diagnostics.");

        for (std::size_t vertex_index = 0u;
            vertex_index < actual_positions.size(); ++vertex_index)
        {
            require(std::abs(actual_positions[vertex_index].x -
                    cpu_positions[vertex_index].x) <= 2.0e-4f &&
                std::abs(actual_positions[vertex_index].y -
                    cpu_positions[vertex_index].y) <= 2.0e-4f &&
                std::abs(actual_positions[vertex_index].z -
                    cpu_positions[vertex_index].z) <= 2.0e-4f,
                "Production colored Stretch GPU positions must match the CPU per-color XPBD reference.");
        }
        for (std::size_t constraint_index = 0u;
            constraint_index < actual_lambdas.size(); ++constraint_index)
        {
            require(std::abs(actual_lambdas[constraint_index] -
                    cpu_lambdas[constraint_index]) <= 2.0e-4f,
                "Production colored Stretch GPU Lambda must match the Segment-local CPU reference.");
        }

        std::cout << "[PASS] P05_WARP_COLORED_STRETCH_XPBD\n";
    }

    void test_phase05_warp_cloth_segment_state_passes()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(SUCCEEDED(create_warp_device(
                device.GetAddressOf(), context.GetAddressOf())),
            "Cloth Segment State WARP test requires Feature Level 11_0.");

        const std::filesystem::path shader_root =
            test_support::fixture_directory().parent_path().parent_path() /
            "shaders" / "runtime";
        const ComPtr<ID3D11ComputeShader> predict_shader =
            compile_compute_shader(
                device.Get(), shader_root / "cloth_predict.cs.hlsl", "main");
        const ComPtr<ID3D11ComputeShader> reconstruct_shader =
            compile_compute_shader(
                device.Get(), shader_root / "cloth_reconstruct_velocity.cs.hlsl", "main");
        const ComPtr<ID3D11ComputeShader> reset_shader =
            compile_compute_shader(
                device.Get(), shader_root / "cloth_reset_lambdas.cs.hlsl", "main");

        const std::vector<gpu_float4> start_positions = {
            { 0.0f, 0.0f, 0.0f, 1.0f },
            { 2.0f, 3.0f, 4.0f, 1.0f },
            { 5.0f, 6.0f, 7.0f, 1.0f }
        };
        const std::vector<gpu_float4> start_velocities = {
            { 1.0f, 0.0f, 0.0f, 0.0f },
            { -2.0f, 0.0f, 1.0f, 0.0f },
            { 9.0f, 8.0f, 7.0f, 0.0f }
        };
        const std::vector<float> inverse_masses = { 1.0f, 2.0f, 0.0f };
        const std::vector<uint32_t> zero_status = { 0u };

        const ComPtr<ID3D11Buffer> start_position_buffer =
            create_structured_buffer(device.Get(), start_positions,
                D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> start_velocity_buffer =
            create_structured_buffer(device.Get(), start_velocities,
                D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> inverse_mass_buffer =
            create_structured_buffer(device.Get(), inverse_masses,
                D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> predicted_position_buffer =
            create_structured_buffer(device.Get(), start_positions,
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> predicted_velocity_buffer =
            create_structured_buffer(device.Get(), start_velocities,
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> status_buffer =
            create_structured_buffer(device.Get(), zero_status,
                D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11ShaderResourceView> start_position_srv =
            create_structured_srv<gpu_float4>(
                device.Get(), start_position_buffer.Get(), 3u);
        const ComPtr<ID3D11ShaderResourceView> start_velocity_srv =
            create_structured_srv<gpu_float4>(
                device.Get(), start_velocity_buffer.Get(), 3u);
        const ComPtr<ID3D11ShaderResourceView> inverse_mass_srv =
            create_structured_srv<float>(
                device.Get(), inverse_mass_buffer.Get(), 3u);
        const ComPtr<ID3D11UnorderedAccessView> predicted_position_uav =
            create_structured_uav<gpu_float4>(
                device.Get(), predicted_position_buffer.Get(), 3u);
        const ComPtr<ID3D11UnorderedAccessView> predicted_velocity_uav =
            create_structured_uav<gpu_float4>(
                device.Get(), predicted_velocity_buffer.Get(), 3u);
        const ComPtr<ID3D11UnorderedAccessView> status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), status_buffer.Get(), 1u);

        struct alignas(16) predict_parameters
        {
            uint32_t vertex_count = 0u;
            float delta_time = 0.0f;
            uint32_t reserved[2] = {};
            float gravity[3] = {};
            uint32_t reserved2 = 0u;
        };
        static_assert(sizeof(predict_parameters) == 32u);
        predict_parameters predict_constants;
        predict_constants.vertex_count = 3u;
        predict_constants.delta_time = 0.25f;
        predict_constants.gravity[0] = 0.0f;
        predict_constants.gravity[1] = -4.0f;
        predict_constants.gravity[2] = 0.0f;
        const ComPtr<ID3D11Buffer> predict_constant_buffer =
            create_constant_buffer(
                device.Get(), &predict_constants,
                static_cast<UINT>(sizeof(predict_constants)));

        dispatch_compute(
            context.Get(), predict_shader.Get(),
            { start_position_srv.Get(), start_velocity_srv.Get(),
                inverse_mass_srv.Get() },
            { predicted_position_uav.Get(), predicted_velocity_uav.Get(),
                status_uav.Get() },
            predict_constant_buffer.Get(), 1u);

        const std::vector<gpu_float4> predicted_positions =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(), predicted_position_buffer.Get(), 3u);
        const std::vector<gpu_float4> predicted_velocities =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(), predicted_velocity_buffer.Get(), 3u);
        require(std::abs(predicted_positions[0].x - 0.25f) <= 1.0e-6f &&
            std::abs(predicted_positions[0].y + 0.25f) <= 1.0e-6f &&
            std::abs(predicted_velocities[0].x - 1.0f) <= 1.0e-6f &&
            std::abs(predicted_velocities[0].y + 1.0f) <= 1.0e-6f,
            "Cloth Segment prediction must apply semi-implicit gravity and preserve the exact Segment dt.");
        require(std::abs(predicted_positions[1].x - 1.5f) <= 1.0e-6f &&
            std::abs(predicted_positions[1].y - 2.75f) <= 1.0e-6f &&
            std::abs(predicted_positions[1].z - 4.25f) <= 1.0e-6f &&
            std::abs(predicted_velocities[1].x + 2.0f) <= 1.0e-6f &&
            std::abs(predicted_velocities[1].y + 1.0f) <= 1.0e-6f &&
            std::abs(predicted_velocities[1].z - 1.0f) <= 1.0e-6f,
            "Cloth Segment prediction must integrate every dynamic Vertex independently.");
        require(predicted_positions[2].x == start_positions[2].x &&
            predicted_positions[2].y == start_positions[2].y &&
            predicted_positions[2].z == start_positions[2].z &&
            predicted_velocities[2].x == 0.0f &&
            predicted_velocities[2].y == 0.0f &&
            predicted_velocities[2].z == 0.0f,
            "Zero inverse-mass Cloth Vertices must preserve their Position and own zero Velocity.");
        const std::vector<uint32_t> prediction_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(prediction_status[0] == 0u,
            "Valid Cloth Segment prediction must not raise solver diagnostics.");

        std::vector<gpu_float4> solved_positions = predicted_positions;
        solved_positions[0].x += 0.1f;
        solved_positions[0].y -= 0.05f;
        solved_positions[1].z += 0.2f;
        solved_positions[2].x += 100.0f;
        context->UpdateSubresource(
            predicted_position_buffer.Get(), 0u, nullptr,
            solved_positions.data(), 0u, 0u);

        struct alignas(16) reconstruct_parameters
        {
            uint32_t vertex_count = 0u;
            float delta_time = 0.0f;
            uint32_t reserved[2] = {};
        };
        static_assert(sizeof(reconstruct_parameters) == 16u);
        reconstruct_parameters reconstruct_constants;
        reconstruct_constants.vertex_count = 3u;
        reconstruct_constants.delta_time = 0.25f;
        const ComPtr<ID3D11Buffer> reconstruct_constant_buffer =
            create_constant_buffer(
                device.Get(), &reconstruct_constants,
                static_cast<UINT>(sizeof(reconstruct_constants)));
        const ComPtr<ID3D11UnorderedAccessView> predicted_velocity_uav =
            create_structured_uav<gpu_float4>(
                device.Get(), predicted_velocity_buffer.Get(), 3u);

        dispatch_compute(
            context.Get(), reconstruct_shader.Get(),
            { start_position_srv.Get(),
                create_structured_srv<gpu_float4>(
                    device.Get(), predicted_position_buffer.Get(), 3u).Get(),
                inverse_mass_srv.Get() },
            { predicted_velocity_uav.Get(), status_uav.Get() },
            reconstruct_constant_buffer.Get(), 1u);

        const std::vector<gpu_float4> reconstructed_velocities =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(), predicted_velocity_buffer.Get(), 3u);
        require(std::abs(reconstructed_velocities[0].x - 1.4f) <= 1.0e-5f &&
            std::abs(reconstructed_velocities[0].y + 1.2f) <= 1.0e-5f &&
            std::abs(reconstructed_velocities[1].x + 2.0f) <= 1.0e-5f &&
            std::abs(reconstructed_velocities[1].y + 1.0f) <= 1.0e-5f &&
            std::abs(reconstructed_velocities[1].z - 1.8f) <= 1.0e-5f,
            "Cloth Velocity reconstruction must use the final XPBD Position delta over the same Segment duration.");
        require(reconstructed_velocities[2].x == 0.0f &&
            reconstructed_velocities[2].y == 0.0f &&
            reconstructed_velocities[2].z == 0.0f,
            "Cloth Velocity reconstruction must keep zero inverse-mass Vertices fixed.");
        const std::vector<uint32_t> reconstruction_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(reconstruction_status[0] == 0u,
            "Valid Cloth Velocity reconstruction must not raise solver diagnostics.");

        const std::vector<float> dirty_lambdas = { 3.5f, -2.0f, 0.25f, 9.0f };
        const ComPtr<ID3D11Buffer> lambda_buffer =
            create_structured_buffer(
                device.Get(), dirty_lambdas,
                D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> lambda_uav =
            create_structured_uav<float>(
                device.Get(), lambda_buffer.Get(),
                static_cast<UINT>(dirty_lambdas.size()));
        struct alignas(16) reset_parameters
        {
            uint32_t lambda_count = 0u;
            uint32_t reserved[3] = {};
        };
        static_assert(sizeof(reset_parameters) == 16u);
        reset_parameters reset_constants;
        reset_constants.lambda_count =
            static_cast<uint32_t>(dirty_lambdas.size());
        const ComPtr<ID3D11Buffer> reset_constant_buffer =
            create_constant_buffer(
                device.Get(), &reset_constants,
                static_cast<UINT>(sizeof(reset_constants)));
        dispatch_compute(
            context.Get(), reset_shader.Get(), {},
            { lambda_uav.Get(), status_uav.Get() },
            reset_constant_buffer.Get(), 1u);
        const std::vector<float> reset_lambdas =
            readback_structured_buffer<float>(
                device.Get(), context.Get(), lambda_buffer.Get(),
                dirty_lambdas.size());
        require(std::all_of(
                reset_lambdas.begin(), reset_lambdas.end(),
                [](float value) { return value == 0.0f; }),
            "Every Cloth XPBD Lambda must be reset at the start of a new Integration Segment.");
        const std::vector<uint32_t> reset_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(reset_status[0] == 0u,
            "Valid Cloth Lambda reset must not raise solver diagnostics.");

        std::cout << "[PASS] P05_WARP_CLOTH_SEGMENT_STATE_PASSES\n";
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

    void test_phase05_warp_cloth_jacobi_contribution_gather()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        require(SUCCEEDED(create_warp_device(
                device.GetAddressOf(), context.GetAddressOf())),
            "Cloth Jacobi contribution / gather WARP test requires Feature Level 11_0.");

        const test_json::value input = test_json::load(
            test_support::fixture_path("phase05_cloth/cloth_patch.input.json"));
        const skin_physics::physics::cloth_mesh mesh = load_cloth_fixture(input);
        require(!mesh.shear_constraints.empty() && !mesh.bend_constraints.empty() &&
            !mesh.vertex_constraint_refs.empty() &&
            mesh.vertex_constraint_offsets.size() == mesh.rest_positions.size() + 1u,
            "Cloth Jacobi gather fixture requires Shear / Bend constraints and a valid Vertex-to-Constraint CSR.");

        const std::filesystem::path shader_root =
            test_support::fixture_directory().parent_path().parent_path() /
            "shaders" / "runtime";
        const ComPtr<ID3D11ComputeShader> contribution_shader =
            compile_compute_shader(
                device.Get(),
                shader_root / "cloth_jacobi_contributions.cs.hlsl",
                "main");
        const ComPtr<ID3D11ComputeShader> gather_shader =
            compile_compute_shader(
                device.Get(),
                shader_root / "cloth_jacobi_gather.cs.hlsl",
                "main");

        std::vector<vec3> source_positions = mesh.rest_positions;
        source_positions[1u].y += 0.3f;
        source_positions[2u].x += 0.1f;
        source_positions[3u].z += 0.23f;
        std::vector<gpu_float4> gpu_positions;
        gpu_positions.reserve(source_positions.size());
        for (const vec3& position : source_positions)
        {
            gpu_positions.push_back(
                { position.x, position.y, position.z, 0.0f });
        }
        std::vector<float> inverse_masses(
            source_positions.size(), 1.0f);
        // Exercise pinned-vertex mass weighting in both Contribution and Gather.
        inverse_masses[0u] = 0.0f;

        const float delta_time = static_cast<float>(
            input.at("fixed_delta_time").as_number());
        const float shear_compliance = static_cast<float>(
            input.at("compliance").at("shear").as_number());
        const float bend_compliance = static_cast<float>(
            input.at("compliance").at("bend").as_number());
        constexpr float relaxation = 0.8f;

        const std::vector<gpu_cloth_constraint> shear_constraints =
            make_gpu_constraints(mesh, 1u, shear_compliance);
        const std::vector<gpu_cloth_constraint> bend_constraints =
            make_gpu_constraints(mesh, 2u, bend_compliance);
        std::vector<gpu_cloth_stretch_constraint> stretch_constraints;
        stretch_constraints.reserve(mesh.stretch_constraints.size());
        for (const auto& source : mesh.stretch_constraints)
        {
            gpu_cloth_stretch_constraint value;
            value.vertices[0] = source.vertices[0u];
            value.vertices[1] = source.vertices[1u];
            value.rest_length = source.rest_length;
            stretch_constraints.push_back(value);
        }

        std::vector<gpu_cloth_vertex_constraint_ref> references;
        references.reserve(mesh.vertex_constraint_refs.size());
        for (const auto& source : mesh.vertex_constraint_refs)
        {
            gpu_cloth_vertex_constraint_ref value;
            value.type = source.type;
            value.constraint_index = source.constraint_index;
            value.local_vertex_index = source.local_vertex_index;
            value.reserved = source.reserved;
            references.push_back(value);
        }

        const std::vector<float> initial_shear_lambdas(
            shear_constraints.size(), 0.0f);
        const std::vector<float> initial_bend_lambdas(
            bend_constraints.size(), 0.0f);
        const std::vector<gpu_cloth_jacobi_contribution> zero_shear_contributions(
            shear_constraints.size());
        const std::vector<gpu_cloth_jacobi_contribution> zero_bend_contributions(
            bend_constraints.size());
        const std::vector<uint32_t> initial_status = { 0u };

        const ComPtr<ID3D11Buffer> positions_buffer =
            create_structured_buffer(
                device.Get(), gpu_positions, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> masses_buffer =
            create_structured_buffer(
                device.Get(), inverse_masses, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> shear_constraints_buffer =
            create_structured_buffer(
                device.Get(), shear_constraints, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> bend_constraints_buffer =
            create_structured_buffer(
                device.Get(), bend_constraints, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> shear_contributions_buffer =
            create_structured_buffer(
                device.Get(), zero_shear_contributions,
                D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> bend_contributions_buffer =
            create_structured_buffer(
                device.Get(), zero_bend_contributions,
                D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> shear_lambdas_buffer =
            create_structured_buffer(
                device.Get(), initial_shear_lambdas,
                D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> bend_lambdas_buffer =
            create_structured_buffer(
                device.Get(), initial_bend_lambdas,
                D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> status_buffer =
            create_structured_buffer(
                device.Get(), initial_status, D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11ShaderResourceView> positions_srv =
            create_structured_srv<gpu_float4>(
                device.Get(), positions_buffer.Get(),
                static_cast<UINT>(gpu_positions.size()));
        const ComPtr<ID3D11ShaderResourceView> masses_srv =
            create_structured_srv<float>(
                device.Get(), masses_buffer.Get(),
                static_cast<UINT>(inverse_masses.size()));
        const ComPtr<ID3D11ShaderResourceView> shear_constraints_srv =
            create_structured_srv<gpu_cloth_constraint>(
                device.Get(), shear_constraints_buffer.Get(),
                static_cast<UINT>(shear_constraints.size()));
        const ComPtr<ID3D11ShaderResourceView> bend_constraints_srv =
            create_structured_srv<gpu_cloth_constraint>(
                device.Get(), bend_constraints_buffer.Get(),
                static_cast<UINT>(bend_constraints.size()));
        const ComPtr<ID3D11UnorderedAccessView> shear_contributions_uav =
            create_structured_uav<gpu_cloth_jacobi_contribution>(
                device.Get(), shear_contributions_buffer.Get(),
                static_cast<UINT>(shear_constraints.size()));
        const ComPtr<ID3D11UnorderedAccessView> bend_contributions_uav =
            create_structured_uav<gpu_cloth_jacobi_contribution>(
                device.Get(), bend_contributions_buffer.Get(),
                static_cast<UINT>(bend_constraints.size()));
        const ComPtr<ID3D11UnorderedAccessView> shear_lambdas_uav =
            create_structured_uav<float>(
                device.Get(), shear_lambdas_buffer.Get(),
                static_cast<UINT>(initial_shear_lambdas.size()));
        const ComPtr<ID3D11UnorderedAccessView> bend_lambdas_uav =
            create_structured_uav<float>(
                device.Get(), bend_lambdas_buffer.Get(),
                static_cast<UINT>(initial_bend_lambdas.size()));
        const ComPtr<ID3D11UnorderedAccessView> status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), status_buffer.Get(), 1u);

        gpu_cloth_jacobi_parameters contribution_parameters;
        contribution_parameters.vertex_count =
            static_cast<uint32_t>(gpu_positions.size());
        contribution_parameters.delta_time = delta_time;
        contribution_parameters.relaxation = relaxation;
        const ComPtr<ID3D11Buffer> contribution_parameter_buffer =
            create_constant_buffer(
                device.Get(), &contribution_parameters,
                sizeof(contribution_parameters));

        contribution_parameters.constraint_count =
            static_cast<uint32_t>(shear_constraints.size());
        contribution_parameters.relaxation = relaxation;
        context->UpdateSubresource(
            contribution_parameter_buffer.Get(), 0u, nullptr,
            &contribution_parameters, 0u, 0u);
        dispatch_compute(
            context.Get(), contribution_shader.Get(),
            { positions_srv.Get(), masses_srv.Get(), shear_constraints_srv.Get() },
            { shear_contributions_uav.Get(), shear_lambdas_uav.Get(),
                status_uav.Get() },
            contribution_parameter_buffer.Get(),
            static_cast<UINT>((shear_constraints.size() + 63u) / 64u));

        contribution_parameters.constraint_count =
            static_cast<uint32_t>(bend_constraints.size());
        context->UpdateSubresource(
            contribution_parameter_buffer.Get(), 0u, nullptr,
            &contribution_parameters, 0u, 0u);
        dispatch_compute(
            context.Get(), contribution_shader.Get(),
            { positions_srv.Get(), masses_srv.Get(), bend_constraints_srv.Get() },
            { bend_contributions_uav.Get(), bend_lambdas_uav.Get(),
                status_uav.Get() },
            contribution_parameter_buffer.Get(),
            static_cast<UINT>((bend_constraints.size() + 63u) / 64u));

        const ComPtr<ID3D11Buffer> offsets_buffer =
            create_structured_buffer(
                device.Get(), mesh.vertex_constraint_offsets,
                D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> references_buffer =
            create_structured_buffer(
                device.Get(), references, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> stretch_constraints_buffer =
            create_structured_buffer(
                device.Get(), stretch_constraints, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> output_positions_buffer =
            create_structured_buffer(
                device.Get(), gpu_positions, D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11ShaderResourceView> offsets_srv =
            create_structured_srv<uint32_t>(
                device.Get(), offsets_buffer.Get(),
                static_cast<UINT>(mesh.vertex_constraint_offsets.size()));
        const ComPtr<ID3D11ShaderResourceView> references_srv =
            create_structured_srv<gpu_cloth_vertex_constraint_ref>(
                device.Get(), references_buffer.Get(),
                static_cast<UINT>(references.size()));
        const ComPtr<ID3D11ShaderResourceView> stretch_constraints_srv =
            create_structured_srv<gpu_cloth_stretch_constraint>(
                device.Get(), stretch_constraints_buffer.Get(),
                static_cast<UINT>(stretch_constraints.size()));
        const ComPtr<ID3D11ShaderResourceView> shear_contributions_srv =
            create_structured_srv<gpu_cloth_jacobi_contribution>(
                device.Get(), shear_contributions_buffer.Get(),
                static_cast<UINT>(shear_constraints.size()));
        const ComPtr<ID3D11ShaderResourceView> bend_contributions_srv =
            create_structured_srv<gpu_cloth_jacobi_contribution>(
                device.Get(), bend_contributions_buffer.Get(),
                static_cast<UINT>(bend_constraints.size()));
        const ComPtr<ID3D11UnorderedAccessView> output_positions_uav =
            create_structured_uav<gpu_float4>(
                device.Get(), output_positions_buffer.Get(),
                static_cast<UINT>(gpu_positions.size()));

        gpu_cloth_jacobi_gather_parameters gather_parameters;
        gather_parameters.vertex_count =
            static_cast<uint32_t>(gpu_positions.size());
        gather_parameters.stretch_constraint_count =
            static_cast<uint32_t>(stretch_constraints.size());
        gather_parameters.shear_constraint_count =
            static_cast<uint32_t>(shear_constraints.size());
        gather_parameters.bend_constraint_count =
            static_cast<uint32_t>(bend_constraints.size());
        gather_parameters.reference_count =
            static_cast<uint32_t>(references.size());
        const ComPtr<ID3D11Buffer> gather_parameter_buffer =
            create_constant_buffer(
                device.Get(), &gather_parameters, sizeof(gather_parameters));

        dispatch_compute(
            context.Get(), gather_shader.Get(),
            { positions_srv.Get(), offsets_srv.Get(), references_srv.Get(),
                stretch_constraints_srv.Get(), shear_constraints_srv.Get(),
                bend_constraints_srv.Get(), shear_contributions_srv.Get(),
                bend_contributions_srv.Get() },
            { output_positions_uav.Get(), status_uav.Get() },
            gather_parameter_buffer.Get(),
            static_cast<UINT>((gpu_positions.size() + 63u) / 64u));

        const std::vector<uint32_t> actual_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(actual_status[0u] == 0u,
            "Production Cloth Jacobi contribution / gather passes must complete without diagnostics.");

        const std::vector<gpu_float4> actual_positions =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(),
                output_positions_buffer.Get(), gpu_positions.size());

        skin_physics::physics::cloth_solver_settings cpu_settings;
        cpu_settings.fixed_delta_time = delta_time;
        cpu_settings.iterations = 1u;
        cpu_settings.relaxation = relaxation;
        cpu_settings.solve_stretch = false;
        cpu_settings.solve_shear = true;
        cpu_settings.solve_bend = true;
        cpu_settings.shear_compliance = shear_compliance;
        cpu_settings.bend_compliance = bend_compliance;
        const auto cpu_result = skin_physics::physics::solve_cloth(
            mesh, source_positions, inverse_masses, cpu_settings);
        require(cpu_result.success,
            "Cloth Jacobi gather CPU reference must complete successfully.");

        for (std::size_t vertex_index = 0u;
            vertex_index < actual_positions.size(); ++vertex_index)
        {
            const vec3& expected = cpu_result.positions[vertex_index];
            require(std::abs(actual_positions[vertex_index].x - expected.x) <= 4.0e-4f &&
                std::abs(actual_positions[vertex_index].y - expected.y) <= 4.0e-4f &&
                std::abs(actual_positions[vertex_index].z - expected.z) <= 4.0e-4f,
                "Production Cloth Jacobi contribution / CSR gather positions must match the CPU simultaneous XPBD reference.");
        }

        std::vector<gpu_cloth_vertex_constraint_ref> invalid_references =
            references;
        uint32_t invalid_reference_vertex = 0xffffffffu;
        for (uint32_t vertex_index = 0u;
            vertex_index < mesh.rest_positions.size() &&
                invalid_reference_vertex == 0xffffffffu;
            ++vertex_index)
        {
            const uint32_t first =
                mesh.vertex_constraint_offsets[vertex_index];
            const uint32_t end =
                mesh.vertex_constraint_offsets[vertex_index + 1u];
            for (uint32_t reference_index = first;
                reference_index < end; ++reference_index)
            {
                if (invalid_references[reference_index].type == 1u ||
                    invalid_references[reference_index].type == 2u)
                {
                    invalid_references[reference_index].constraint_index =
                        0xffffffffu;
                    invalid_reference_vertex = vertex_index;
                    break;
                }
            }
        }
        require(invalid_reference_vertex != 0xffffffffu,
            "Cloth Jacobi invalid-CSR regression requires a Shear / Bend reference.");

        const ComPtr<ID3D11Buffer> invalid_references_buffer =
            create_structured_buffer(
                device.Get(), invalid_references, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> invalid_output_buffer =
            create_structured_buffer(
                device.Get(), gpu_positions, D3D11_BIND_UNORDERED_ACCESS);
        const std::vector<uint32_t> zero_status = { 0u };
        const ComPtr<ID3D11Buffer> invalid_status_buffer =
            create_structured_buffer(
                device.Get(), zero_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11ShaderResourceView> invalid_references_srv =
            create_structured_srv<gpu_cloth_vertex_constraint_ref>(
                device.Get(), invalid_references_buffer.Get(),
                static_cast<UINT>(invalid_references.size()));
        const ComPtr<ID3D11UnorderedAccessView> invalid_output_uav =
            create_structured_uav<gpu_float4>(
                device.Get(), invalid_output_buffer.Get(),
                static_cast<UINT>(gpu_positions.size()));
        const ComPtr<ID3D11UnorderedAccessView> invalid_status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), invalid_status_buffer.Get(), 1u);
        dispatch_compute(
            context.Get(), gather_shader.Get(),
            { positions_srv.Get(), offsets_srv.Get(),
                invalid_references_srv.Get(), stretch_constraints_srv.Get(),
                shear_constraints_srv.Get(), bend_constraints_srv.Get(),
                shear_contributions_srv.Get(), bend_contributions_srv.Get() },
            { invalid_output_uav.Get(), invalid_status_uav.Get() },
            gather_parameter_buffer.Get(),
            static_cast<UINT>((gpu_positions.size() + 63u) / 64u));

        const std::vector<uint32_t> invalid_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), invalid_status_buffer.Get(), 1u);
        const std::vector<gpu_float4> invalid_output =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(),
                invalid_output_buffer.Get(), gpu_positions.size());
        const gpu_float4& rejected_vertex =
            invalid_output[invalid_reference_vertex];
        require((invalid_status[0u] & (1u << 2u)) != 0u &&
            std::abs(rejected_vertex.x -
                gpu_positions[invalid_reference_vertex].x) <= 1.0e-7f &&
            std::abs(rejected_vertex.y -
                gpu_positions[invalid_reference_vertex].y) <= 1.0e-7f &&
            std::abs(rejected_vertex.z -
                gpu_positions[invalid_reference_vertex].z) <= 1.0e-7f,
            "Production Cloth Jacobi gather must flag an invalid CSR reference and preserve the affected source Position.");

        std::cout << "[PASS] P05_WARP_CLOTH_JACOBI_CONTRIBUTION_GATHER\n";
    }

    struct gpu_tet_input
    {
        uint32_t vertex_ids[4] = {};
        gpu_float4 rest_inverse[3] = {};
        gpu_float4 rest_gradients[4] = {};
        gpu_float4 material0;
        gpu_float4 material1;
    };

    struct gpu_tet_output
    {
        gpu_float4 deformation[3];
        gpu_float4 rotation[3];
        gpu_float4 stress[3];
        gpu_float4 forces[4];
        gpu_float4 metrics;
    };

    struct alignas(16) gpu_tet_dispatch_parameters
    {
        uint32_t tetrahedron_count = 1;
        uint32_t reserved[3] = {};
    };

    struct alignas(16) gpu_tick_commit_control
    {
        uint32_t committed_sequence_low = 0u;
        uint32_t committed_sequence_high = 0u;
        uint32_t state_generation_low = 1u;
        uint32_t state_generation_high = 0u;
        uint32_t active_state_set = 0u;
        uint32_t terminal_state = 1u;
        uint32_t terminal_failure = 0u;
        uint32_t validation_mask = 0u;
        uint32_t required_validation_mask = 1u;
        uint32_t dispatch_id_low = 1u;
        uint32_t dispatch_id_high = 0u;
        uint32_t reservation_token_low = 1u;
        uint32_t reservation_token_high = 0u;
        uint32_t last_gate_state = 1u;
        uint32_t diagnostic = 0u;
        uint32_t render_slot_id = skin_physics::asset::invalid_index;
        uint32_t render_slot_generation = 0u;
        uint32_t reserved[3] = {};
    };

    static_assert(sizeof(gpu_tet_input) == 160, "Tet input layout must match HLSL.");
    static_assert(sizeof(gpu_tet_output) == 224, "Tet output layout must match HLSL.");
    static_assert(sizeof(gpu_tick_commit_control) == 80,
        "Production Runtime FEM Tick Commit Control layout must match HLSL.");

    void test_phase06_warp_corotational_fem()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase06_soft_body/tet_deformation.input.json"));

        skin_physics::physics::raw_tet_mesh raw_mesh;
        for (const test_json::value& point : input.at("rest_positions").as_array())
        {
            raw_mesh.positions.push_back(read_vec3(point));
        }
        const auto& tet_indices = input.at("tetrahedron").as_array();
        raw_mesh.tetrahedra.push_back({
            static_cast<uint32_t>(tet_indices[0].as_number()),
            static_cast<uint32_t>(tet_indices[1].as_number()),
            static_cast<uint32_t>(tet_indices[2].as_number()),
            static_cast<uint32_t>(tet_indices[3].as_number())
        });
        skin_physics::physics::physics_mesh_build_settings build_settings;
        build_settings.density = static_cast<float>(input.at("density").as_number());
        const auto built = skin_physics::physics::build_physics_mesh(raw_mesh, build_settings);
        require(built.success && built.value.tetrahedra.size() == 1,
            "Phase 6 FEM GPU fixture must build one valid tetrahedron.");

        skin_physics::physics::soft_body_material material;
        material.shear_modulus = static_cast<float>(input.at("mu").as_number());
        material.bulk_modulus = static_cast<float>(input.at("K").as_number());
        material.density = static_cast<float>(input.at("density").as_number());
        material.inversion_stiffness = 10000.0f;
        material.minimum_jacobian = 0.1f;

        const skin_physics::physics::tetrahedron& source_tet =
            built.value.tetrahedra[0];
        gpu_tet_input gpu_parameters;
        for (std::size_t index = 0; index < 4; ++index)
        {
            gpu_parameters.vertex_ids[index] = source_tet.vertices[index];
            const vec3& gradient = source_tet.rest_shape_gradients[index];
            gpu_parameters.rest_gradients[index] = {
                gradient.x, gradient.y, gradient.z, 0.0f
            };
        }
        for (std::size_t row = 0; row < 3; ++row)
        {
            gpu_parameters.rest_inverse[row] = {
                source_tet.rest_inverse[row * 3],
                source_tet.rest_inverse[row * 3 + 1],
                source_tet.rest_inverse[row * 3 + 2],
                0.0f
            };
        }
        gpu_parameters.material0 = {
            source_tet.rest_volume,
            material.shear_modulus,
            material.bulk_modulus,
            material.shape_recovery_scale
        };
        gpu_parameters.material1 = {
            material.volume_retention_scale,
            material.inversion_stiffness,
            material.minimum_jacobian,
            0.0f
        };

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result), "D3D11 WARP device creation must succeed.");

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_path("phase06_soft_body/corotational_fem.cs.hlsl"),
            "main");

        const auto& cases = input.at("deformation_cases").as_array();
        for (const test_json::value& deformation_case : cases)
        {
            std::vector<vec3> current_positions;
            std::vector<gpu_float4> gpu_positions;
            for (const test_json::value& point :
                deformation_case.at("current_positions").as_array())
            {
                const vec3 value = read_vec3(point);
                current_positions.push_back(value);
                gpu_positions.push_back({ value.x, value.y, value.z, 0.0f });
            }

            const auto cpu = skin_physics::physics::evaluate_corotational_tetrahedron(
                built.value, 0, current_positions, material);
            require(cpu.success && cpu.state_committable,
                "Positive-Jacobian FEM fixture must be accepted by the CPU reference.");

            const std::vector<gpu_tet_input> tet_inputs = { gpu_parameters };
            const std::vector<gpu_tet_output> initial_output(1);
            const ComPtr<ID3D11Buffer> positions_buffer =
                create_structured_buffer(device.Get(), gpu_positions, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> tet_buffer =
                create_structured_buffer(device.Get(), tet_inputs, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> output_buffer =
                create_structured_buffer(device.Get(), initial_output, D3D11_BIND_UNORDERED_ACCESS);

            const ComPtr<ID3D11ShaderResourceView> positions_view =
                create_structured_srv<gpu_float4>(device.Get(), positions_buffer.Get(), 4);
            const ComPtr<ID3D11ShaderResourceView> tet_view =
                create_structured_srv<gpu_tet_input>(device.Get(), tet_buffer.Get(), 1);
            const ComPtr<ID3D11UnorderedAccessView> output_view =
                create_structured_uav<gpu_tet_output>(device.Get(), output_buffer.Get(), 1);

            gpu_tet_dispatch_parameters dispatch_parameters;
            const ComPtr<ID3D11Buffer> constants =
                create_constant_buffer(device.Get(), &dispatch_parameters, sizeof(dispatch_parameters));
            dispatch_compute(
                context.Get(), shader.Get(),
                { positions_view.Get(), tet_view.Get() },
                { output_view.Get() },
                constants.Get(), 1);

            const std::vector<gpu_tet_output> gpu_results =
                readback_structured_buffer<gpu_tet_output>(
                    device.Get(), context.Get(), output_buffer.Get(), 1);
            const gpu_tet_output& gpu = gpu_results[0];

            const float tolerance = 2.0e-3f;
            for (std::size_t row = 0; row < 3; ++row)
            {
                const float gpu_deformation[3] = {
                    gpu.deformation[row].x, gpu.deformation[row].y,
                    gpu.deformation[row].z
                };
                const float gpu_rotation[3] = {
                    gpu.rotation[row].x, gpu.rotation[row].y,
                    gpu.rotation[row].z
                };
                const float gpu_stress[3] = {
                    gpu.stress[row].x, gpu.stress[row].y,
                    gpu.stress[row].z
                };
                for (std::size_t column = 0; column < 3; ++column)
                {
                    const std::size_t matrix_index = row * 3 + column;
                    require(std::abs(gpu_deformation[column] -
                        cpu.deformation_gradient.value[matrix_index]) <= 1.0e-4f,
                        "WARP deformation gradient must match CPU FEM.");
                    require(std::abs(gpu_rotation[column] -
                        cpu.rotation.value[matrix_index]) <= 1.0e-4f,
                        "WARP polar rotation must match CPU FEM.");
                    require(std::abs(gpu_stress[column] -
                        cpu.first_piola_stress.value[matrix_index]) <= tolerance,
                        "WARP first Piola stress must match CPU FEM.");
                }
            }

            for (std::size_t vertex = 0; vertex < 4; ++vertex)
            {
                const float actual[3] = {
                    gpu.forces[vertex].x,
                    gpu.forces[vertex].y,
                    gpu.forces[vertex].z
                };
                const vec3& expected = cpu.internal_forces[vertex];
                require(std::abs(actual[0] - expected.x) <= tolerance &&
                    std::abs(actual[1] - expected.y) <= tolerance &&
                    std::abs(actual[2] - expected.z) <= tolerance,
                    "WARP tetrahedral force contributions must match CPU FEM.");
            }

            require(std::abs(gpu.metrics.x - cpu.jacobian) <= 1.0e-5f &&
                std::abs(gpu.metrics.y - cpu.current_volume) <= 1.0e-5f &&
                std::abs(gpu.metrics.z - cpu.elastic_energy) <= tolerance &&
                gpu.metrics.w == 1.0f,
                "WARP FEM Jacobian, volume, energy and commit flag must match the CPU reference.");
        }
        std::cout << "[PASS] P06_WARP_COROTATIONAL_FEM\n";
    }


    void test_r1_warp_runtime_soft_body_fem_svd_parity()
    {
        skin_physics::physics::raw_tet_mesh raw_mesh;
        raw_mesh.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f }
        };
        raw_mesh.tetrahedra = { { 0u, 1u, 2u, 3u } };
        const auto built = skin_physics::physics::build_physics_mesh(raw_mesh);
        require(built.success && built.value.tetrahedra.size() == 1u,
            "Runtime Soft Body SVD fixture must build one valid tetrahedron.");

        skin_physics::physics::soft_body_material material;
        material.shear_modulus = 1000.0f;
        material.bulk_modulus = 5000.0f;
        material.density = 1000.0f;
        material.inversion_stiffness = 10000.0f;
        material.minimum_jacobian = 0.1f;

        const skin_physics::physics::tetrahedron& source_tet =
            built.value.tetrahedra[0];
        gpu_tet_input tet_input;
        for (std::size_t index = 0u; index < 4u; ++index)
        {
            tet_input.vertex_ids[index] = source_tet.vertices[index];
            const vec3& gradient = source_tet.rest_shape_gradients[index];
            tet_input.rest_gradients[index] = {
                gradient.x, gradient.y, gradient.z, 0.0f
            };
        }
        for (std::size_t row = 0u; row < 3u; ++row)
        {
            tet_input.rest_inverse[row] = {
                source_tet.rest_inverse[row * 3u],
                source_tet.rest_inverse[row * 3u + 1u],
                source_tet.rest_inverse[row * 3u + 2u],
                0.0f
            };
        }
        tet_input.material0 = {
            source_tet.rest_volume,
            material.shear_modulus,
            material.bulk_modulus,
            material.shape_recovery_scale
        };
        tet_input.material1 = {
            material.volume_retention_scale,
            material.inversion_stiffness,
            material.minimum_jacobian,
            0.0f
        };

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Runtime Soft Body SVD parity requires a D3D11 WARP device.");

        const std::filesystem::path shader_path =
            test_support::fixture_directory().parent_path().parent_path() /
            "shaders" / "runtime" / "soft_body_fem.cs.hlsl";
        require(std::filesystem::is_regular_file(shader_path),
            "Production Runtime Soft Body FEM shader must be present in the repository shader package.");
        const ComPtr<ID3D11ComputeShader> shader =
            compile_compute_shader(device.Get(), shader_path, "soft_body_fem");

        struct test_case
        {
            std::vector<vec3> positions;
            bool expect_svd_fallback = false;
            bool expect_committable = true;
        };

        std::vector<vec3> inverted_positions = raw_mesh.positions;
        const vec3& tet_origin = raw_mesh.positions[source_tet.vertices[0]];
        const vec3& tet_rest_point1 = raw_mesh.positions[source_tet.vertices[1]];
        const vec3& tet_rest_point2 = raw_mesh.positions[source_tet.vertices[2]];
        const vec3& tet_rest_point3 = raw_mesh.positions[source_tet.vertices[3]];
        const vec3 edge1 = {
            tet_rest_point1.x - tet_origin.x,
            tet_rest_point1.y - tet_origin.y,
            tet_rest_point1.z - tet_origin.z
        };
        const vec3 edge2 = {
            tet_rest_point2.x - tet_origin.x,
            tet_rest_point2.y - tet_origin.y,
            tet_rest_point2.z - tet_origin.z
        };
        const vec3 edge3 = {
            tet_rest_point3.x - tet_origin.x,
            tet_rest_point3.y - tet_origin.y,
            tet_rest_point3.z - tet_origin.z
        };
        inverted_positions[source_tet.vertices[1]] = {
            tet_origin.x - 0.9f * edge1.x,
            tet_origin.y - 0.9f * edge1.y,
            tet_origin.z - 0.9f * edge1.z
        };
        inverted_positions[source_tet.vertices[2]] = {
            tet_origin.x + edge2.x,
            tet_origin.y + edge2.y,
            tet_origin.z + edge2.z
        };
        inverted_positions[source_tet.vertices[3]] = {
            tet_origin.x + 1.1f * edge3.x,
            tet_origin.y + 1.1f * edge3.y,
            tet_origin.z + 1.1f * edge3.z
        };

        const std::vector<test_case> cases = {
            {
                {
                    { 0.0f, 0.0f, 0.0f },
                    { 0.9f, 0.0f, 0.0f },
                    { 0.0f, 1.0f, 0.0f },
                    { 0.0f, 0.0f, 1.0f }
                },
                false,
                true
            },
            {
                {
                    { 0.0f, 0.0f, 0.0f },
                    { 1.0e-15f, 0.0f, 0.0f },
                    { 0.0f, 1.0f, 0.0f },
                    { 0.0f, 0.0f, 1.0f }
                },
                true,
                true
            },
            {
                inverted_positions,
                true,
                false
            }
        };

        for (const test_case& item : cases)
        {
            const skin_physics::physics::corotational_tet_result cpu =
                skin_physics::physics::evaluate_corotational_tetrahedron(
                    built.value, 0u, item.positions, material);
            require(cpu.success,
                "Runtime FEM CPU Reference must produce finite FEM diagnostics for the parity fixture.");
            require(cpu.state_committable == item.expect_committable &&
                cpu.inverted == !item.expect_committable,
                "CPU FEM must preserve its inverted-element diagnostic and non-committable state.");
            require(cpu.used_svd_fallback == item.expect_svd_fallback,
                "CPU FEM must select the expected polar or SVD path for the parity fixture.");
            require((cpu.jacobian > 0.0f) == item.expect_committable,
                "FEM parity fixture Jacobian sign must match its expected Commit eligibility.");

            std::vector<gpu_float4> gpu_positions;
            gpu_positions.reserve(item.positions.size());
            for (const vec3& position : item.positions)
            {
                gpu_positions.push_back({
                    position.x, position.y, position.z, 1.0f
                });
            }

            const std::vector<gpu_tet_input> tet_inputs = { tet_input };
            const std::vector<gpu_tet_output> initial_outputs(1u);
            const std::vector<gpu_tick_commit_control> initial_control = {
                gpu_tick_commit_control{}
            };

            const ComPtr<ID3D11Buffer> positions_buffer =
                create_structured_buffer(
                    device.Get(), gpu_positions, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> tet_buffer =
                create_structured_buffer(
                    device.Get(), tet_inputs, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> output_buffer =
                create_structured_buffer(
                    device.Get(), initial_outputs, D3D11_BIND_UNORDERED_ACCESS);
            const ComPtr<ID3D11Buffer> control_buffer =
                create_structured_buffer(
                    device.Get(), initial_control, D3D11_BIND_UNORDERED_ACCESS);

            const ComPtr<ID3D11ShaderResourceView> positions_srv =
                create_structured_srv<gpu_float4>(
                    device.Get(), positions_buffer.Get(), 4u);
            const ComPtr<ID3D11ShaderResourceView> tet_srv =
                create_structured_srv<gpu_tet_input>(
                    device.Get(), tet_buffer.Get(), 1u);
            const ComPtr<ID3D11UnorderedAccessView> output_uav =
                create_structured_uav<gpu_tet_output>(
                    device.Get(), output_buffer.Get(), 1u);
            const ComPtr<ID3D11UnorderedAccessView> control_uav =
                create_structured_uav<gpu_tick_commit_control>(
                    device.Get(), control_buffer.Get(), 1u);

            gpu_tet_dispatch_parameters dispatch_parameters;
            const ComPtr<ID3D11Buffer> constants =
                create_constant_buffer(
                    device.Get(),
                    &dispatch_parameters,
                    static_cast<UINT>(sizeof(dispatch_parameters)));

            dispatch_compute(
                context.Get(),
                shader.Get(),
                { positions_srv.Get(), tet_srv.Get() },
                { output_uav.Get(), control_uav.Get() },
                constants.Get(),
                1u);

            const std::vector<gpu_tet_output> gpu_outputs =
                readback_structured_buffer<gpu_tet_output>(
                    device.Get(), context.Get(), output_buffer.Get(), 1u);
            const std::vector<gpu_tick_commit_control> control =
                readback_structured_buffer<gpu_tick_commit_control>(
                    device.Get(), context.Get(), control_buffer.Get(), 1u);
            const gpu_tet_output& gpu = gpu_outputs[0];

            const float tolerance = 3.0e-3f;
            for (std::size_t row = 0u; row < 3u; ++row)
            {
                const float gpu_deformation[3] = {
                    gpu.deformation[row].x,
                    gpu.deformation[row].y,
                    gpu.deformation[row].z
                };
                const float gpu_rotation[3] = {
                    gpu.rotation[row].x,
                    gpu.rotation[row].y,
                    gpu.rotation[row].z
                };
                const float gpu_stress[3] = {
                    gpu.stress[row].x,
                    gpu.stress[row].y,
                    gpu.stress[row].z
                };
                for (std::size_t column = 0u; column < 3u; ++column)
                {
                    const std::size_t matrix_index = row * 3u + column;
                    require(std::abs(gpu_deformation[column] -
                        cpu.deformation_gradient.value[matrix_index]) <= 1.0e-5f,
                        "Production Runtime FEM deformation gradient must match CPU Reference.");
                    require(std::abs(gpu_rotation[column] -
                        cpu.rotation.value[matrix_index]) <= tolerance,
                        "Production Runtime FEM polar / SVD rotation must match CPU Reference.");
                    require(std::abs(gpu_stress[column] -
                        cpu.first_piola_stress.value[matrix_index]) <= tolerance,
                        "Production Runtime FEM stress must match CPU Reference.");
                }
            }

            for (std::size_t vertex = 0u; vertex < 4u; ++vertex)
            {
                const vec3& expected = cpu.internal_forces[vertex];
                const gpu_float4& actual = gpu.forces[vertex];
                require(std::abs(actual.x - expected.x) <= tolerance &&
                    std::abs(actual.y - expected.y) <= tolerance &&
                    std::abs(actual.z - expected.z) <= tolerance,
                    "Production Runtime FEM nodal forces must match CPU Reference.");
            }

            require(std::abs(gpu.metrics.x - cpu.jacobian) <= 1.0e-5f &&
                std::abs(gpu.metrics.y - cpu.current_volume) <= 1.0e-5f &&
                std::abs(gpu.metrics.z - cpu.elastic_energy) <= tolerance &&
                gpu.metrics.w == (item.expect_svd_fallback ? 2.0f : 1.0f),
                "Production Runtime FEM must report matching measures and the selected polar / SVD path.");
            require(control[0].terminal_failure ==
                    (item.expect_committable ? 0u : 1u),
                "Only positive-Jacobian FEM output may remain Commit-eligible; inverted output must publish diagnostics and set terminal failure.");
        }

        std::cout << "[PASS] R1_WARP_RUNTIME_SOFT_BODY_FEM_SVD_AND_INVERSION_PARITY\n";
    }

    struct gpu_vt_result
    {
        gpu_float4 corrected0;
        gpu_float4 corrected1;
        gpu_float4 corrected2;
        gpu_float4 corrected3;
        gpu_float4 barycentric_penetration;
        gpu_float4 normal_hit;
        gpu_float4 metrics;
    };

    struct gpu_ee_result
    {
        gpu_float4 closest_a;
        gpu_float4 closest_b;
        gpu_float4 parameters;
        gpu_float4 normal_hit;
    };

    struct gpu_aabb
    {
        gpu_float4 minimum;
        gpu_float4 maximum;
    };

    struct gpu_bvh_node
    {
        gpu_float4 minimum;
        gpu_float4 maximum;
        uint32_t left = 0;
        uint32_t right = 0;
        uint32_t primitive_id = 0;
        uint32_t is_leaf = 0;
    };

    struct gpu_uint2
    {
        uint32_t x = 0;
        uint32_t y = 0;
    };

    struct gpu_correction_primitive_input
    {
        uint32_t vertex_indices[4] = {};
        uint32_t vertex_count = 0;
        uint32_t reserved[3] = {};
    };

    struct alignas(16) gpu_correction_parameters
    {
        uint32_t primitive_count = 0;
        uint32_t vertex_count = 0;
        float contact_thickness = 0.0f;
        uint32_t reserved = 0;
    };

    struct gpu_correction_candidate_primitive
    {
        uint32_t stable_id_low = 0;
        uint32_t stable_id_high = 0;
        uint32_t mesh_id = 0;
        uint32_t primitive_id = 0;
        uint32_t role = 0;
        uint32_t vertex_count = 0;
        uint32_t vertex_id_low[4] = {};
        uint32_t vertex_id_high[4] = {};
    };

    struct gpu_correction_candidate_pair
    {
        uint32_t identity_a[4] = {};
        uint32_t identity_b[4] = {};
        uint32_t metadata[4] = {};
    };

    struct alignas(16) gpu_correction_candidate_parameters
    {
        uint32_t primitive_count = 0;
        uint32_t mesh_count = 0;
        uint32_t pair_count = 0;
        uint32_t candidate_capacity = 0;
        uint32_t sort_capacity = 0;
        uint32_t exclusion_count = 0;
        uint32_t self_collision_enabled = 0;
        uint32_t reserved = 0;
    };

    struct alignas(16) gpu_correction_sort_parameters
    {
        uint32_t sort_k = 0;
        uint32_t sort_j = 0;
        uint32_t reserved0 = 0;
        uint32_t reserved1 = 0;
    };


    struct alignas(16) gpu_collision_parameters
    {
        float contact_radius = 0.0f;
        float relaxation = 1.0f;
        float combined_thickness = 0.0f;
        uint32_t primitive_count = 0;
        uint32_t node_count = 0;
        uint32_t mesh_count = 0;
        uint32_t candidate_capacity = 0;
        uint32_t self_collision_enabled = 0;
        float expansion = 0.0f;
        uint32_t padding[3] = {};
    };

    static_assert(sizeof(gpu_vt_result) == 112, "VT output layout must match HLSL.");
    static_assert(sizeof(gpu_ee_result) == 64, "EE output layout must match HLSL.");
    static_assert(sizeof(gpu_aabb) == 32, "AABB layout must match HLSL.");
    static_assert(sizeof(gpu_bvh_node) == 48, "BVH node layout must match HLSL.");
    static_assert(sizeof(gpu_collision_parameters) == 48,
        "Collision parameter layout must match HLSL.");
    static_assert(sizeof(gpu_correction_primitive_input) == 32,
        "Correction primitive input layout must match HLSL.");
    static_assert(sizeof(gpu_correction_parameters) == 16,
        "Correction parameter layout must match HLSL.");
    static_assert(sizeof(gpu_correction_candidate_primitive) == 56,
        "Correction candidate primitive layout must match HLSL.");
    static_assert(sizeof(gpu_correction_candidate_pair) == 48,
        "Correction candidate pair layout must match HLSL.");
    static_assert(sizeof(gpu_correction_candidate_parameters) == 32,
        "Correction candidate parameter layout must match HLSL.");
    static_assert(sizeof(gpu_correction_sort_parameters) == 16,
        "Correction sort parameter layout must match HLSL.");

    gpu_float4 to_gpu_float4(const vec3& value, float w = 0.0f)
    {
        return { value.x, value.y, value.z, w };
    }

    vec3 from_gpu_float4(const gpu_float4& value)
    {
        return { value.x, value.y, value.z };
    }

    void test_phase07_warp_vertex_triangle_contact(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* shader)
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase07_collision/vt_contact.input.json"));
        std::array<vec3, 4> positions = {};
        positions[0] = read_vec3(input.at("vertex_position"));
        const auto& triangle_json = input.at("triangle").as_array();
        for (std::size_t index = 0; index < 3; ++index)
        {
            positions[index + 1] = read_vec3(triangle_json[index]);
        }
        std::array<float, 4> inverse_masses = {
            static_cast<float>(input.at("inverse_masses").at("vertex").as_number()),
            static_cast<float>(input.at("inverse_masses").at("triangle_vertices").at(0).as_number()),
            static_cast<float>(input.at("inverse_masses").at("triangle_vertices").at(1).as_number()),
            static_cast<float>(input.at("inverse_masses").at("triangle_vertices").at(2).as_number())
        };
        const float radius = static_cast<float>(input.at("vertex_radius").as_number());
        const float relaxation_value = 0.5f;

        const auto cpu_contact = skin_physics::physics::find_vertex_triangle_contact(
            positions[0], { positions[1], positions[2], positions[3] }, radius);
        require(cpu_contact.valid && cpu_contact.hit,
            "VT fixture must produce a valid CPU reference contact.");
        const auto cpu_solve = skin_physics::physics::solve_vertex_triangle_contact(
            cpu_contact, positions, inverse_masses, radius, relaxation_value);
        require(cpu_solve.success, "VT CPU position correction reference must succeed.");

        std::vector<gpu_float4> gpu_positions;
        for (std::size_t index = 0; index < positions.size(); ++index)
        {
            gpu_positions.push_back(to_gpu_float4(positions[index], inverse_masses[index]));
        }
        const std::vector<gpu_vt_result> initial_output(1);
        const ComPtr<ID3D11Buffer> positions_buffer =
            create_structured_buffer(device, gpu_positions, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(device, initial_output, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11ShaderResourceView> positions_view =
            create_structured_srv<gpu_float4>(device, positions_buffer.Get(), 4);
        const ComPtr<ID3D11UnorderedAccessView> output_view =
            create_structured_uav<gpu_vt_result>(device, output_buffer.Get(), 1);

        gpu_collision_parameters parameters;
        parameters.contact_radius = radius;
        parameters.relaxation = relaxation_value;
        const ComPtr<ID3D11Buffer> constants =
            create_constant_buffer(device, &parameters, sizeof(parameters));

        dispatch_compute(context, shader, { positions_view.Get() },
            { output_view.Get() }, constants.Get(), 1);
        const std::vector<gpu_vt_result> result =
            readback_structured_buffer<gpu_vt_result>(device, context, output_buffer.Get(), 1);
        const gpu_vt_result& gpu = result[0];

        const float tolerance = 1.0e-5f;
        require(std::abs(gpu.barycentric_penetration.x - cpu_contact.barycentric[0]) <= tolerance &&
            std::abs(gpu.barycentric_penetration.y - cpu_contact.barycentric[1]) <= tolerance &&
            std::abs(gpu.barycentric_penetration.z - cpu_contact.barycentric[2]) <= tolerance &&
            std::abs(gpu.barycentric_penetration.w - cpu_contact.penetration_depth) <= tolerance,
            "WARP VT contact barycentrics and penetration must match the CPU reference.");
        const gpu_float4 gpu_positions_after[4] = {
            gpu.corrected0, gpu.corrected1, gpu.corrected2, gpu.corrected3
        };
        for (std::size_t index = 0; index < 4; ++index)
        {
            const vec3 actual = from_gpu_float4(gpu_positions_after[index]);
            const vec3& expected = cpu_solve.positions[index];
            require(std::abs(actual.x - expected.x) <= tolerance &&
                std::abs(actual.y - expected.y) <= tolerance &&
                std::abs(actual.z - expected.z) <= tolerance,
                "WARP VT mass-weighted position corrections must match the CPU reference.");
        }
        require(std::abs(gpu.metrics.y - cpu_solve.post_solve_penetration) <= tolerance,
            "WARP VT post-solve penetration must match the CPU reference.");
        std::cout << "[PASS] P07_WARP_VT_NARROWPHASE\n";
    }

    void test_phase07_warp_edge_edge_contact(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* shader)
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase07_collision/ee_contact.input.json"));
        std::vector<gpu_float4> gpu_positions;
        std::array<vec3, 2> edge_a = {};
        std::array<vec3, 2> edge_b = {};
        for (std::size_t index = 0; index < 2; ++index)
        {
            edge_a[index] = read_vec3(input.at("edge_a").at(index));
            edge_b[index] = read_vec3(input.at("edge_b").at(index));
            gpu_positions.push_back(to_gpu_float4(edge_a[index]));
        }
        for (const vec3& value : edge_b)
        {
            gpu_positions.push_back(to_gpu_float4(value));
        }
        const float thickness = static_cast<float>(input.at("thickness").as_number());
        const auto cpu = skin_physics::physics::find_edge_edge_contact(
            edge_a, edge_b, thickness);
        require(cpu.valid && cpu.hit, "EE fixture must produce a valid CPU reference contact.");

        const std::vector<gpu_ee_result> initial_output(1);
        const ComPtr<ID3D11Buffer> positions_buffer =
            create_structured_buffer(device, gpu_positions, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(device, initial_output, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11ShaderResourceView> positions_view =
            create_structured_srv<gpu_float4>(device, positions_buffer.Get(), 4);
        const ComPtr<ID3D11UnorderedAccessView> output_view =
            create_structured_uav<gpu_ee_result>(device, output_buffer.Get(), 1);

        gpu_collision_parameters parameters;
        parameters.combined_thickness = thickness;
        const ComPtr<ID3D11Buffer> constants =
            create_constant_buffer(device, &parameters, sizeof(parameters));

        ID3D11ShaderResourceView* resources[] = { positions_view.Get() };
        ID3D11UnorderedAccessView* unordered_views[] = { nullptr, output_view.Get() };
        context->CSSetShader(shader, nullptr, 0);
        context->CSSetShaderResources(0, 1, resources);
        context->CSSetUnorderedAccessViews(0, 2, unordered_views, nullptr);
        context->CSSetConstantBuffers(0, 1, constants.GetAddressOf());
        context->Dispatch(1, 1, 1);
        ID3D11ShaderResourceView* null_resources[] = { nullptr };
        ID3D11UnorderedAccessView* null_unordered[] = { nullptr, nullptr };
        ID3D11Buffer* null_constants[] = { nullptr };
        context->CSSetShaderResources(0, 1, null_resources);
        context->CSSetUnorderedAccessViews(0, 2, null_unordered, nullptr);
        context->CSSetConstantBuffers(0, 1, null_constants);
        context->CSSetShader(nullptr, nullptr, 0);

        const std::vector<gpu_ee_result> result =
            readback_structured_buffer<gpu_ee_result>(device, context, output_buffer.Get(), 1);
        const gpu_ee_result& gpu = result[0];
        const float tolerance = 1.0e-6f;
        require(std::abs(gpu.parameters.x - cpu.parameter_a) <= tolerance &&
            std::abs(gpu.parameters.y - cpu.parameter_b) <= tolerance &&
            std::abs(gpu.parameters.z - cpu.distance) <= tolerance &&
            std::abs(gpu.parameters.w - cpu.penetration_depth) <= tolerance,
            "WARP EE closest parameters and penetration must match the CPU reference.");
        const vec3 gpu_a = from_gpu_float4(gpu.closest_a);
        const vec3 gpu_b = from_gpu_float4(gpu.closest_b);
        require(std::abs(gpu_a.x - cpu.closest_point_a.x) <= tolerance &&
            std::abs(gpu_a.y - cpu.closest_point_a.y) <= tolerance &&
            std::abs(gpu_a.z - cpu.closest_point_a.z) <= tolerance &&
            std::abs(gpu_b.x - cpu.closest_point_b.x) <= tolerance &&
            std::abs(gpu_b.y - cpu.closest_point_b.y) <= tolerance &&
            std::abs(gpu_b.z - cpu.closest_point_b.z) <= tolerance,
            "WARP EE closest points must match the CPU reference.");
        std::cout << "[PASS] P07_WARP_EE_NARROWPHASE\n";
    }

    void test_phase07_warp_hierarchical_broadphase(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* shader)
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase07_collision/broadphase.input.json"));

        std::vector<gpu_aabb> primitive_bounds;
        std::vector<uint32_t> mesh_ids;
        std::vector<skin_physics::physics::broadphase_primitive> cpu_primitives;
        for (const test_json::value& primitive : input.at("primitives").as_array())
        {
            const vec3 minimum = read_vec3(primitive.at("bounds_min"));
            const vec3 maximum = read_vec3(primitive.at("bounds_max"));
            const uint32_t mesh_id = static_cast<uint32_t>(primitive.at("mesh").as_number());
            primitive_bounds.push_back({
                to_gpu_float4(minimum), to_gpu_float4(maximum)
            });
            mesh_ids.push_back(mesh_id);
            skin_physics::physics::broadphase_primitive cpu_primitive;
            cpu_primitive.mesh_id = mesh_id;
            cpu_primitive.bounds.minimum = minimum;
            cpu_primitive.bounds.maximum = maximum;
            cpu_primitives.push_back(cpu_primitive);
        }

        std::vector<uint32_t> pair_filter(9, 0);
        std::vector<skin_physics::physics::broadphase_mesh_pair> cpu_mesh_pairs;
        for (const test_json::value& pair : input.at("mesh_pairs").as_array())
        {
            const uint32_t a = static_cast<uint32_t>(pair.at("a").as_number());
            const uint32_t b = static_cast<uint32_t>(pair.at("b").as_number());
            const bool enabled = pair.at("enabled").as_bool();
            pair_filter[a * 3 + b] = enabled ? 1u : 0u;
            pair_filter[b * 3 + a] = enabled ? 1u : 0u;
            cpu_mesh_pairs.push_back({ a, b, enabled });
        }

        const auto make_node = [](const gpu_aabb& bounds,
            uint32_t left, uint32_t right, uint32_t primitive_id, bool leaf)
        {
            gpu_bvh_node node;
            node.minimum = bounds.minimum;
            node.maximum = bounds.maximum;
            node.left = left;
            node.right = right;
            node.primitive_id = primitive_id;
            node.is_leaf = leaf ? 1u : 0u;
            return node;
        };

        gpu_aabb internal_bounds;
        internal_bounds.minimum = {
            std::min(primitive_bounds[1].minimum.x, primitive_bounds[2].minimum.x),
            std::min(primitive_bounds[1].minimum.y, primitive_bounds[2].minimum.y),
            std::min(primitive_bounds[1].minimum.z, primitive_bounds[2].minimum.z), 0.0f
        };
        internal_bounds.maximum = {
            std::max(primitive_bounds[1].maximum.x, primitive_bounds[2].maximum.x),
            std::max(primitive_bounds[1].maximum.y, primitive_bounds[2].maximum.y),
            std::max(primitive_bounds[1].maximum.z, primitive_bounds[2].maximum.z), 0.0f
        };
        gpu_aabb root_bounds;
        root_bounds.minimum = {
            std::min(primitive_bounds[0].minimum.x, internal_bounds.minimum.x),
            std::min(primitive_bounds[0].minimum.y, internal_bounds.minimum.y),
            std::min(primitive_bounds[0].minimum.z, internal_bounds.minimum.z), 0.0f
        };
        root_bounds.maximum = {
            std::max(primitive_bounds[0].maximum.x, internal_bounds.maximum.x),
            std::max(primitive_bounds[0].maximum.y, internal_bounds.maximum.y),
            std::max(primitive_bounds[0].maximum.z, internal_bounds.maximum.z), 0.0f
        };

        std::vector<gpu_bvh_node> nodes(5);
        nodes[0] = make_node(root_bounds, 1, 2, 0, false);
        nodes[1] = make_node(primitive_bounds[0], 0, 0, 0, true);
        nodes[2] = make_node(internal_bounds, 3, 4, 0, false);
        nodes[3] = make_node(primitive_bounds[1], 0, 0, 1, true);
        nodes[4] = make_node(primitive_bounds[2], 0, 0, 2, true);

        const auto cpu_reference =
            skin_physics::physics::generate_broadphase_candidates(
                cpu_primitives, cpu_mesh_pairs);
        require(cpu_reference.success && cpu_reference.candidates.size() == 1,
            "CPU broadphase reference must yield one fixture candidate.");

        const auto run_gpu_broadphase = [&](const std::vector<uint32_t>& filters,
            uint32_t capacity)
        {
            const std::vector<gpu_uint2> initial_pairs(capacity);
            const std::vector<uint32_t> initial_counters = { 0u, 0u };
            const ComPtr<ID3D11Buffer> nodes_buffer =
                create_structured_buffer(device, nodes, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> bounds_buffer =
                create_structured_buffer(device, primitive_bounds, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> mesh_ids_buffer =
                create_structured_buffer(device, mesh_ids, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> filters_buffer =
                create_structured_buffer(device, filters, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> pairs_buffer =
                create_structured_buffer(device, initial_pairs, D3D11_BIND_UNORDERED_ACCESS);
            const ComPtr<ID3D11Buffer> counters_buffer =
                create_structured_buffer(device, initial_counters, D3D11_BIND_UNORDERED_ACCESS);

            const ComPtr<ID3D11ShaderResourceView> nodes_view =
                create_structured_srv<gpu_bvh_node>(device, nodes_buffer.Get(), 5);
            const ComPtr<ID3D11ShaderResourceView> bounds_view =
                create_structured_srv<gpu_aabb>(device, bounds_buffer.Get(), 3);
            const ComPtr<ID3D11ShaderResourceView> meshes_view =
                create_structured_srv<uint32_t>(device, mesh_ids_buffer.Get(), 3);
            const ComPtr<ID3D11ShaderResourceView> filters_view =
                create_structured_srv<uint32_t>(device, filters_buffer.Get(), 9);
            const ComPtr<ID3D11UnorderedAccessView> pairs_view =
                create_structured_uav<gpu_uint2>(device, pairs_buffer.Get(), capacity);
            const ComPtr<ID3D11UnorderedAccessView> counters_view =
                create_structured_uav<uint32_t>(device, counters_buffer.Get(), 2);

            gpu_collision_parameters parameters;
            parameters.primitive_count = 3;
            parameters.node_count = 5;
            parameters.mesh_count = 3;
            parameters.candidate_capacity = capacity;
            parameters.self_collision_enabled =
                input.at("self_collision_enabled").as_bool() ? 1u : 0u;
            const ComPtr<ID3D11Buffer> constants =
                create_constant_buffer(device, &parameters, sizeof(parameters));

            ID3D11ShaderResourceView* resources[] = {
                nullptr, nullptr, nodes_view.Get(), bounds_view.Get(),
                meshes_view.Get(), filters_view.Get()
            };
            ID3D11UnorderedAccessView* unordered_views[] = {
                nullptr, nullptr, pairs_view.Get(), counters_view.Get()
            };
            context->CSSetShader(shader, nullptr, 0);
            context->CSSetShaderResources(0, 6, resources);
            context->CSSetUnorderedAccessViews(0, 4, unordered_views, nullptr);
            context->CSSetConstantBuffers(0, 1, constants.GetAddressOf());
            context->Dispatch(1, 1, 1);
            ID3D11ShaderResourceView* null_resources[6] = {};
            ID3D11UnorderedAccessView* null_unordered[4] = {};
            ID3D11Buffer* null_constants[] = { nullptr };
            context->CSSetShaderResources(0, 6, null_resources);
            context->CSSetUnorderedAccessViews(0, 4, null_unordered, nullptr);
            context->CSSetConstantBuffers(0, 1, null_constants);
            context->CSSetShader(nullptr, nullptr, 0);

            const std::vector<uint32_t> counters =
                readback_structured_buffer<uint32_t>(
                    device, context, counters_buffer.Get(), 2);
            const std::vector<gpu_uint2> pairs =
                readback_structured_buffer<gpu_uint2>(
                    device, context, pairs_buffer.Get(), capacity);
            return std::make_pair(counters, pairs);
        };

        const auto normal = run_gpu_broadphase(pair_filter, 2);
        require(normal.first[0] == 1 && normal.first[1] == 0,
            "GPU BVH traversal must produce one candidate without overflow.");
        require(normal.second[0].x == 0 && normal.second[0].y == 1,
            "GPU BVH candidate IDs must match the exact fixture reference set.");

        std::vector<uint32_t> expanded_filter = pair_filter;
        expanded_filter[0 * 3 + 2] = 1;
        expanded_filter[2 * 3 + 0] = 1;
        const auto overflow = run_gpu_broadphase(expanded_filter, 1);
        require(overflow.first[0] == 2 && overflow.first[1] == 1,
            "GPU BVH traversal must count every candidate and flag bounded-buffer overflow.");
        require(cpu_reference.candidates[0].primitive_a == 0 &&
            cpu_reference.candidates[0].primitive_b == 1,
            "Fixture CPU reference pair ordering must remain stable.");
        std::cout << "[PASS] P07_WARP_BVH_BROADPHASE_AND_OVERFLOW\n";
    }

    void test_phase12_warp_correction_swept_aabb(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* shader)
    {
        const test_json::value correction_fixture = test_json::load(
            test_support::fixture_path("design_review/correction_sweep_broadphase.json"));
        const test_json::value* crossing_case = nullptr;
        for (const test_json::value& fixture_case :
            correction_fixture.at("cases").as_array())
        {
            if (fixture_case.at("id").as_string() ==
                "DCR_49_VT_POINT_THROUGH_TRIANGLE")
            {
                crossing_case = &fixture_case;
                break;
            }
        }
        require(crossing_case != nullptr,
            "WARP correction AABB test requires the persistent VT crossing fixture.");

        const test_json::value& input = crossing_case->at("input");
        const float contact_thickness = static_cast<float>(
            input.at("contact_thickness_m").as_number());
        const auto triangle_before = input.at("triangle_vertices_before").as_array();
        const auto triangle_after = input.at("triangle_vertices_after").as_array();
        const std::vector<vec3> positions_before = {
            read_vec3(input.at("point_before")),
            read_vec3(triangle_before[0]),
            read_vec3(triangle_before[1]),
            read_vec3(triangle_before[2])
        };
        const std::vector<vec3> positions_after = {
            read_vec3(input.at("point_after")),
            read_vec3(triangle_after[0]),
            read_vec3(triangle_after[1]),
            read_vec3(triangle_after[2])
        };

        std::vector<gpu_float4> gpu_before;
        std::vector<gpu_float4> gpu_after;
        for (const vec3& position : positions_before)
        {
            gpu_before.push_back(to_gpu_float4(position));
        }
        for (const vec3& position : positions_after)
        {
            gpu_after.push_back(to_gpu_float4(position));
        }

        std::vector<gpu_correction_primitive_input> primitives(2);
        primitives[0].vertex_indices[0] = 0;
        primitives[0].vertex_count = 1;
        primitives[1].vertex_indices[0] = 1;
        primitives[1].vertex_indices[1] = 2;
        primitives[1].vertex_indices[2] = 3;
        primitives[1].vertex_count = 3;

        const std::vector<gpu_aabb> initial_bounds(2);
        const std::vector<uint32_t> initial_status(2, 0xffffffffu);
        const ComPtr<ID3D11Buffer> before_buffer =
            create_structured_buffer(device, gpu_before, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> after_buffer =
            create_structured_buffer(device, gpu_after, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> primitive_buffer =
            create_structured_buffer(device, primitives, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> bounds_buffer =
            create_structured_buffer(device, initial_bounds, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> status_buffer =
            create_structured_buffer(device, initial_status, D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11ShaderResourceView> before_view =
            create_structured_srv<gpu_float4>(device, before_buffer.Get(), 4);
        const ComPtr<ID3D11ShaderResourceView> after_view =
            create_structured_srv<gpu_float4>(device, after_buffer.Get(), 4);
        const ComPtr<ID3D11ShaderResourceView> primitive_view =
            create_structured_srv<gpu_correction_primitive_input>(
                device, primitive_buffer.Get(), 2);
        const ComPtr<ID3D11UnorderedAccessView> bounds_view =
            create_structured_uav<gpu_aabb>(device, bounds_buffer.Get(), 2);
        const ComPtr<ID3D11UnorderedAccessView> status_view =
            create_structured_uav<uint32_t>(device, status_buffer.Get(), 2);

        gpu_correction_parameters parameters;
        parameters.primitive_count = 2;
        parameters.vertex_count = 4;
        parameters.contact_thickness = contact_thickness;
        const ComPtr<ID3D11Buffer> constants =
            create_constant_buffer(device, &parameters, sizeof(parameters));

        const auto execute_correction_pass = [&](ID3D11ShaderResourceView* primitive_input,
            ID3D11UnorderedAccessView* output_bounds,
            ID3D11UnorderedAccessView* output_status)
        {
            skin_physics::simulation::d3d11_compute_graph graph;
            skin_physics::simulation::compute_pass_desc pass;
            pass.name = "correction_swept_aabb";
            pass.shader = shader;
            pass.shader_resources = {
                { 6, before_view.Get() },
                { 7, after_view.Get() },
                { 8, primitive_input }
            };
            pass.unordered_access_views = {
                { 4, output_bounds },
                { 5, output_status }
            };
            pass.constant_buffers = { { 1, constants.Get() } };
            pass.group_count_x = 1;
            std::string diagnostic;
            require(graph.add_pass(pass, diagnostic),
                "Correction AABB pass must register in the D3D11 Compute Pass Graph: " + diagnostic);
            const auto execution = graph.execute(context);
            require(execution.success && execution.executed_pass_count == 1,
                "D3D11 Compute Pass Graph must execute the correction AABB dispatch.");
        };
        execute_correction_pass(
            primitive_view.Get(), bounds_view.Get(), status_view.Get());

        const std::vector<gpu_aabb> gpu_bounds =
            readback_structured_buffer<gpu_aabb>(
                device, context, bounds_buffer.Get(), 2);
        const std::vector<uint32_t> gpu_status =
            readback_structured_buffer<uint32_t>(
                device, context, status_buffer.Get(), 2);
        require(gpu_status[0] == 0u && gpu_status[1] == 0u,
            "WARP correction Swept AABB must complete both valid primitives without diagnostics.");

        skin_physics::physics::collision_aabb cpu_point_bounds;
        skin_physics::physics::collision_aabb cpu_triangle_bounds;
        const std::vector<skin_physics::physics::correction_vertex_motion> point_motion = {
            { positions_before[0], positions_after[0] }
        };
        std::vector<skin_physics::physics::correction_vertex_motion> triangle_motion;
        for (std::size_t index = 1; index < 4; ++index)
        {
            triangle_motion.push_back({
                positions_before[index], positions_after[index]
            });
        }
        require(skin_physics::physics::build_correction_swept_aabb(
            point_motion, contact_thickness, cpu_point_bounds) &&
            skin_physics::physics::build_correction_swept_aabb(
                triangle_motion, contact_thickness, cpu_triangle_bounds),
            "CPU correction Swept AABB references must build from the same before/after inputs.");

        const auto compare_bounds = [](const gpu_aabb& actual,
            const skin_physics::physics::collision_aabb& expected)
        {
            const float actual_minimum[3] = {
                actual.minimum.x, actual.minimum.y, actual.minimum.z
            };
            const float actual_maximum[3] = {
                actual.maximum.x, actual.maximum.y, actual.maximum.z
            };
            const float expected_minimum[3] = {
                expected.minimum.x, expected.minimum.y, expected.minimum.z
            };
            const float expected_maximum[3] = {
                expected.maximum.x, expected.maximum.y, expected.maximum.z
            };
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if (actual_minimum[axis] > expected_minimum[axis] ||
                    actual_maximum[axis] < expected_maximum[axis] ||
                    std::abs(actual_minimum[axis] - expected_minimum[axis]) > 1.0e-6f ||
                    std::abs(actual_maximum[axis] - expected_maximum[axis]) > 1.0e-6f)
                {
                    return false;
                }
            }
            return true;
        };
        require(compare_bounds(gpu_bounds[0], cpu_point_bounds) &&
            compare_bounds(gpu_bounds[1], cpu_triangle_bounds),
            "WARP Correction Swept AABBs must enclose the CPU outward-rounded reference bounds.");

        primitives[1].vertex_indices[2] = 99;
        const ComPtr<ID3D11Buffer> invalid_primitive_buffer =
            create_structured_buffer(device, primitives, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> invalid_primitive_view =
            create_structured_srv<gpu_correction_primitive_input>(
                device, invalid_primitive_buffer.Get(), 2);
        const std::vector<gpu_aabb> reset_bounds(2);
        const std::vector<uint32_t> reset_status(2, 0xffffffffu);
        const ComPtr<ID3D11Buffer> invalid_bounds_buffer =
            create_structured_buffer(device, reset_bounds, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> invalid_status_buffer =
            create_structured_buffer(device, reset_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> invalid_bounds_view =
            create_structured_uav<gpu_aabb>(device, invalid_bounds_buffer.Get(), 2);
        const ComPtr<ID3D11UnorderedAccessView> invalid_status_view =
            create_structured_uav<uint32_t>(device, invalid_status_buffer.Get(), 2);
        execute_correction_pass(
            invalid_primitive_view.Get(),
            invalid_bounds_view.Get(), invalid_status_view.Get());

        const std::vector<uint32_t> invalid_status =
            readback_structured_buffer<uint32_t>(
                device, context, invalid_status_buffer.Get(), 2);
        require(invalid_status[0] == 0u && invalid_status[1] == 2u,
            "WARP Correction Swept AABB must fail closed on an out-of-range primitive vertex index.");

        std::cout << "[PASS] P12_WARP_CORRECTION_SWEPT_AABB\n";
    }

    void test_phase12_warp_correction_candidate_pipeline(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* validate_shader,
        ID3D11ComputeShader* count_shader,
        ID3D11ComputeShader* scan_shader,
        ID3D11ComputeShader* initialize_shader,
        ID3D11ComputeShader* fill_shader,
        ID3D11ComputeShader* sort_shader,
        ID3D11ComputeShader* deduplicate_shader)
    {
        using namespace skin_physics::physics;

        const collision_aabb bounds_a = {
            { -0.01f, -0.01f, -0.01f },
            { 1.01f, 1.01f, 1.01f }
        };
        const collision_aabb bounds_b = {
            { 0.40f, 0.40f, 0.40f },
            { 1.40f, 1.40f, 1.40f }
        };
        const collision_aabb edge_a_bounds = {
            { 10.0f, 10.0f, 10.0f },
            { 11.0f, 11.0f, 11.0f }
        };
        const collision_aabb edge_b_bounds = {
            { 10.5f, 10.5f, 10.5f },
            { 11.5f, 11.5f, 11.5f }
        };
        const collision_aabb excluded_edge_a_bounds = {
            { 20.0f, 20.0f, 20.0f },
            { 21.0f, 21.0f, 21.0f }
        };
        const collision_aabb excluded_edge_b_bounds = {
            { 20.5f, 20.5f, 20.5f },
            { 21.5f, 21.5f, 21.5f }
        };

        const auto make_primitive = [](
            uint64_t stable_id,
            uint32_t primitive_id,
            correction_primitive_role role,
            const collision_aabb& bounds,
            const std::vector<uint64_t>& vertex_ids)
        {
            correction_broadphase_primitive primitive;
            primitive.stable_id = stable_id;
            primitive.mesh_id = 0;
            primitive.primitive_id = primitive_id;
            primitive.role = role;
            primitive.envelope_bounds = bounds;
            primitive.vertex_stable_ids = vertex_ids;
            return primitive;
        };

        std::vector<correction_broadphase_primitive> cpu_primitives = {
            make_primitive(100, 0, correction_primitive_role::vertex,
                bounds_a, { 101 }),
            make_primitive(200, 1, correction_primitive_role::triangle,
                bounds_b, { 201, 202, 203 }),
            make_primitive(300, 2, correction_primitive_role::edge,
                edge_a_bounds, { 301, 302 }),
            make_primitive(400, 3, correction_primitive_role::edge,
                edge_b_bounds, { 401, 402 }),
            make_primitive(500, 4, correction_primitive_role::vertex,
                bounds_a, { 201 }),
            make_primitive(600, 5, correction_primitive_role::edge,
                excluded_edge_a_bounds, { 601, 602 }),
            make_primitive(700, 6, correction_primitive_role::edge,
                excluded_edge_b_bounds, { 701, 702 })
        };
        cpu_primitives[5].excluded_primitive_ids = {
            { 700, 0, 6 }
        };

        broadphase_settings cpu_settings;
        cpu_settings.cell_size = 0.5f;
        cpu_settings.contact_thickness = 0.0f;
        cpu_settings.max_cells_per_primitive = 4096;
        cpu_settings.cell_record_capacity = 4096;
        cpu_settings.candidate_capacity = 2;
        cpu_settings.self_collision_enabled = true;
        const correction_broadphase_result cpu_reference =
            generate_correction_broadphase_candidates(
                cpu_primitives, {}, cpu_settings);
        require(cpu_reference.success &&
            cpu_reference.candidates.size() == 2,
            "CPU Correction Candidate reference must preserve the VT / EE pairs and filter shared-vertex / one-ring pairs.");

        const auto to_gpu_bounds = [](const collision_aabb& bounds)
        {
            gpu_aabb result;
            result.minimum = {
                bounds.minimum.x, bounds.minimum.y, bounds.minimum.z, 0.0f
            };
            result.maximum = {
                bounds.maximum.x, bounds.maximum.y, bounds.maximum.z, 0.0f
            };
            return result;
        };

        const auto split_u64 = [](uint64_t value, uint32_t& low, uint32_t& high)
        {
            low = static_cast<uint32_t>(value & 0xffffffffull);
            high = static_cast<uint32_t>(value >> 32u);
        };

        std::vector<gpu_correction_candidate_primitive> gpu_primitives;
        std::vector<gpu_aabb> gpu_bounds;
        for (const correction_broadphase_primitive& primitive : cpu_primitives)
        {
            gpu_correction_candidate_primitive gpu_primitive;
            split_u64(primitive.stable_id,
                gpu_primitive.stable_id_low, gpu_primitive.stable_id_high);
            gpu_primitive.mesh_id = primitive.mesh_id;
            gpu_primitive.primitive_id = primitive.primitive_id;
            gpu_primitive.role = static_cast<uint32_t>(primitive.role);
            gpu_primitive.vertex_count =
                static_cast<uint32_t>(primitive.vertex_stable_ids.size());
            for (std::size_t index = 0;
                index < primitive.vertex_stable_ids.size(); ++index)
            {
                split_u64(
                    primitive.vertex_stable_ids[index],
                    gpu_primitive.vertex_id_low[index],
                    gpu_primitive.vertex_id_high[index]);
            }
            gpu_primitives.push_back(gpu_primitive);
            gpu_bounds.push_back(to_gpu_bounds(primitive.envelope_bounds));
        }

        const std::vector<uint32_t> mesh_pair_filter = { 1u };
        const std::vector<gpu_uint2> excluded_pairs = { { 5u, 6u } };
        std::vector<gpu_uint2> candidate_pair_seeds;
        for (uint32_t primitive_a = 0;
            primitive_a < gpu_primitives.size(); ++primitive_a)
        {
            for (uint32_t primitive_b = primitive_a + 1u;
                primitive_b < gpu_primitives.size(); ++primitive_b)
            {
                candidate_pair_seeds.push_back({ primitive_a, primitive_b });
            }
        }

        struct gpu_pipeline_readback
        {
            std::vector<uint32_t> totals;
            std::vector<gpu_correction_candidate_pair> candidates;
        };

        const auto run_pipeline = [&](
            const std::vector<gpu_correction_candidate_primitive>& input_primitives,
            const std::vector<gpu_uint2>& input_pair_seeds,
            uint32_t candidate_capacity,
            uint32_t sort_capacity) -> gpu_pipeline_readback
        {
            const uint32_t primitive_count =
                static_cast<uint32_t>(input_primitives.size());
            const uint32_t pair_count =
                static_cast<uint32_t>(input_pair_seeds.size());
            const uint32_t pair_buffer_count = std::max(1u, pair_count);
            std::vector<gpu_uint2> pair_seed_storage = input_pair_seeds;
            if (pair_seed_storage.empty())
            {
                pair_seed_storage.push_back({ 0u, 0u });
            }
            const std::vector<uint32_t> initial_flags(pair_buffer_count, 0u);
            const std::vector<uint32_t> initial_prefix(pair_buffer_count, 0u);
            const std::vector<uint32_t> initial_totals = { 0u, 0u };
            const gpu_correction_candidate_pair sentinel = {
                { 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu },
                { 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu },
                { 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu }
            };
            const std::vector<gpu_correction_candidate_pair> initial_output(
                sort_capacity, sentinel);

            const ComPtr<ID3D11Buffer> primitive_buffer =
                create_structured_buffer(
                    device, input_primitives, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> bounds_buffer =
                create_structured_buffer(
                    device, gpu_bounds, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> mesh_filter_buffer =
                create_structured_buffer(
                    device, mesh_pair_filter, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> exclusion_buffer =
                create_structured_buffer(
                    device, excluded_pairs, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> pair_seed_buffer =
                create_structured_buffer(
                    device, pair_seed_storage, D3D11_BIND_SHADER_RESOURCE);
            // Count / Scan outputs are consumed by later Fill passes as SRVs.
            // Keep both bindings on the same structured buffers so the compute
            // graph can transition UAV writes to SRV reads without recreating
            // or copying the intermediate arrays.
            const UINT intermediate_bind_flags =
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
            const ComPtr<ID3D11Buffer> flags_buffer =
                create_structured_buffer(
                    device, initial_flags, intermediate_bind_flags);
            const ComPtr<ID3D11Buffer> prefix_buffer =
                create_structured_buffer(
                    device, initial_prefix, intermediate_bind_flags);
            const ComPtr<ID3D11Buffer> output_buffer =
                create_structured_buffer(
                    device, initial_output, D3D11_BIND_UNORDERED_ACCESS);
            const ComPtr<ID3D11Buffer> totals_buffer =
                create_structured_buffer(
                    device, initial_totals, D3D11_BIND_UNORDERED_ACCESS);

            const ComPtr<ID3D11ShaderResourceView> primitive_view =
                create_structured_srv<gpu_correction_candidate_primitive>(
                    device, primitive_buffer.Get(), primitive_count);
            const ComPtr<ID3D11ShaderResourceView> bounds_view =
                create_structured_srv<gpu_aabb>(
                    device, bounds_buffer.Get(), primitive_count);
            const ComPtr<ID3D11ShaderResourceView> mesh_filter_view =
                create_structured_srv<uint32_t>(
                    device, mesh_filter_buffer.Get(), 1);
            const ComPtr<ID3D11ShaderResourceView> exclusion_view =
                create_structured_srv<gpu_uint2>(
                    device, exclusion_buffer.Get(), 1);
            const ComPtr<ID3D11ShaderResourceView> pair_seed_view =
                create_structured_srv<gpu_uint2>(
                    device, pair_seed_buffer.Get(), pair_buffer_count);
            const ComPtr<ID3D11UnorderedAccessView> flags_uav =
                create_structured_uav<uint32_t>(
                    device, flags_buffer.Get(), pair_buffer_count);
            const ComPtr<ID3D11UnorderedAccessView> prefix_uav =
                create_structured_uav<uint32_t>(
                    device, prefix_buffer.Get(), pair_buffer_count);
            const ComPtr<ID3D11UnorderedAccessView> output_uav =
                create_structured_uav<gpu_correction_candidate_pair>(
                    device, output_buffer.Get(), sort_capacity);
            const ComPtr<ID3D11UnorderedAccessView> totals_uav =
                create_structured_uav<uint32_t>(
                    device, totals_buffer.Get(), 2);

            const ComPtr<ID3D11ShaderResourceView> flags_srv =
                create_structured_srv<uint32_t>(
                    device, flags_buffer.Get(), pair_buffer_count);
            const ComPtr<ID3D11ShaderResourceView> prefix_srv =
                create_structured_srv<uint32_t>(
                    device, prefix_buffer.Get(), pair_buffer_count);

            gpu_correction_candidate_parameters candidate_parameters;
            candidate_parameters.primitive_count = primitive_count;
            candidate_parameters.mesh_count = 1;
            candidate_parameters.pair_count = pair_count;
            candidate_parameters.candidate_capacity = candidate_capacity;
            candidate_parameters.sort_capacity = sort_capacity;
            candidate_parameters.exclusion_count = 1;
            candidate_parameters.self_collision_enabled = 1;
            const ComPtr<ID3D11Buffer> candidate_constants =
                create_constant_buffer(
                    device, &candidate_parameters, sizeof(candidate_parameters));

            skin_physics::simulation::d3d11_compute_graph graph;
            std::string diagnostic;
            const auto add_pass = [&](const std::string& name,
                ID3D11ComputeShader* shader,
                const std::vector<skin_physics::simulation::compute_srv_binding>& srvs,
                const std::vector<skin_physics::simulation::compute_uav_binding>& uavs,
                const std::vector<skin_physics::simulation::compute_constant_buffer_binding>& constants,
                uint32_t group_count)
            {
                skin_physics::simulation::compute_pass_desc pass;
                pass.name = name;
                pass.shader = shader;
                pass.shader_resources = srvs;
                pass.unordered_access_views = uavs;
                pass.constant_buffers = constants;
                pass.group_count_x = group_count;
                require(graph.add_pass(pass, diagnostic),
                    "Correction Candidate Compute Pass Graph registration failed: " +
                        diagnostic);
            };

            const uint32_t pair_group_count =
                std::max(1u, (pair_count + 63u) / 64u);
            const uint32_t primitive_group_count =
                std::max(1u, (primitive_count + 63u) / 64u);
            const uint32_t sort_group_count = (sort_capacity + 63u) / 64u;
            const std::vector<skin_physics::simulation::compute_constant_buffer_binding>
                candidate_cb = { { 0, candidate_constants.Get() } };

            add_pass("correction_candidate_validate_primitives",
                validate_shader,
                {
                    { 0, primitive_view.Get() },
                    { 1, bounds_view.Get() }
                },
                { { 3, totals_uav.Get() } },
                candidate_cb,
                primitive_group_count);

            add_pass("correction_candidate_count",
                count_shader,
                {
                    { 0, primitive_view.Get() },
                    { 1, bounds_view.Get() },
                    { 2, mesh_filter_view.Get() },
                    { 3, pair_seed_view.Get() },
                    { 4, exclusion_view.Get() }
                },
                {
                    { 0, flags_uav.Get() },
                    { 3, totals_uav.Get() }
                },
                candidate_cb,
                pair_group_count);

            add_pass("correction_candidate_exclusive_scan",
                scan_shader,
                { { 5, flags_srv.Get() } },
                {
                    { 1, prefix_uav.Get() },
                    { 3, totals_uav.Get() }
                },
                candidate_cb,
                1);

            add_pass("correction_candidate_initialize",
                initialize_shader,
                {},
                { { 2, output_uav.Get() } },
                candidate_cb,
                sort_group_count);

            add_pass("correction_candidate_fill",
                fill_shader,
                {
                    { 0, primitive_view.Get() },
                    { 3, pair_seed_view.Get() },
                    { 5, flags_srv.Get() },
                    { 6, prefix_srv.Get() }
                },
                {
                    { 2, output_uav.Get() },
                    { 3, totals_uav.Get() }
                },
                candidate_cb,
                pair_group_count);

            for (uint32_t sort_k = 2; sort_k <= sort_capacity; sort_k <<= 1u)
            {
                for (uint32_t sort_j = sort_k >> 1u;
                    sort_j > 0u; sort_j >>= 1u)
                {
                    gpu_correction_sort_parameters sort_parameters;
                    sort_parameters.sort_k = sort_k;
                    sort_parameters.sort_j = sort_j;
                    const ComPtr<ID3D11Buffer> sort_constants =
                        create_constant_buffer(
                            device, &sort_parameters, sizeof(sort_parameters));
                    const std::vector<skin_physics::simulation::compute_constant_buffer_binding>
                        sort_cb = {
                            { 0, candidate_constants.Get() },
                            { 1, sort_constants.Get() }
                        };
                    const std::string pass_name =
                        "correction_candidate_sort_" +
                        std::to_string(sort_k) + "_" +
                        std::to_string(sort_j);
                    add_pass(pass_name,
                        sort_shader,
                        {},
                        {
                            { 2, output_uav.Get() },
                            { 3, totals_uav.Get() }
                        },
                        sort_cb,
                        sort_group_count);
                }
                if (sort_k > sort_capacity / 2u) break;
            }

            add_pass("correction_candidate_deduplicate",
                deduplicate_shader,
                {},
                {
                    { 2, output_uav.Get() },
                    { 3, totals_uav.Get() }
                },
                candidate_cb,
                1);

            const auto execution = graph.execute(context);
            require(execution.success &&
                execution.executed_pass_count == graph.pass_count(),
                "Correction Candidate Count / Scan / Fill / Sort / Dedup graph must execute every pass.");

            gpu_pipeline_readback readback;
            readback.totals = readback_structured_buffer<uint32_t>(
                device, context, totals_buffer.Get(), 2);
            readback.candidates = readback_structured_buffer<gpu_correction_candidate_pair>(
                device, context, output_buffer.Get(), sort_capacity);
            return readback;
        };

        const gpu_pipeline_readback normal =
            run_pipeline(gpu_primitives, candidate_pair_seeds, 2, 2);
        require(normal.totals[0] == cpu_reference.candidates.size() &&
            normal.totals[1] == 0u,
            "WARP Candidate Count / Scan / Capacity / Fill / Sort / Dedup must match the CPU required count without overflow.");

        const auto compare_pair = [](
            const gpu_correction_candidate_pair& gpu,
            const correction_candidate_pair_key& cpu)
        {
            const uint32_t expected_a[4] = {
                static_cast<uint32_t>(cpu.stable_id_a & 0xffffffffull),
                static_cast<uint32_t>(cpu.stable_id_a >> 32u),
                cpu.mesh_a,
                cpu.primitive_a
            };
            const uint32_t expected_b[4] = {
                static_cast<uint32_t>(cpu.stable_id_b & 0xffffffffull),
                static_cast<uint32_t>(cpu.stable_id_b >> 32u),
                cpu.mesh_b,
                cpu.primitive_b
            };
            for (std::size_t index = 0; index < 4; ++index)
            {
                if (gpu.identity_a[index] != expected_a[index] ||
                    gpu.identity_b[index] != expected_b[index])
                {
                    return false;
                }
            }
            return gpu.metadata[0] ==
                static_cast<uint32_t>(cpu.contact_type);
        };
        require(normal.totals[0] == 2u &&
            compare_pair(normal.candidates[0], cpu_reference.candidates[0]) &&
            compare_pair(normal.candidates[1], cpu_reference.candidates[1]),
            "WARP Candidate Pair output must match the CPU sorted Canonical VT / EE Stable-ID keys.");
        require(normal.candidates[0].identity_a[0] == 100u &&
            normal.candidates[0].identity_b[0] == 200u &&
            normal.candidates[1].identity_a[0] == 300u &&
            normal.candidates[1].identity_b[0] == 400u,
            "GPU Candidate output must exclude shared-vertex and explicitly excluded one-ring pairs before sorting.");

        const gpu_pipeline_readback overflow =
            run_pipeline(gpu_primitives, candidate_pair_seeds, 1, 1);
        require(overflow.totals[0] == 2u && overflow.totals[1] == 1u,
            "GPU Candidate Capacity Check must report the full required count and overflow status.");
        require(overflow.candidates.size() == 1 &&
            overflow.candidates[0].identity_a[0] == 0xffffffffu &&
            overflow.candidates[0].identity_b[0] == 0xffffffffu &&
            overflow.candidates[0].metadata[0] == 0xffffffffu,
            "GPU Candidate Fill must write no partial pairs when the Count / Scan result exceeds Capacity.");

        std::vector<gpu_correction_candidate_primitive> duplicate_vertex_ids =
            gpu_primitives;
        duplicate_vertex_ids[1].vertex_id_low[2] =
            duplicate_vertex_ids[1].vertex_id_low[1];
        duplicate_vertex_ids[1].vertex_id_high[2] =
            duplicate_vertex_ids[1].vertex_id_high[1];
        const gpu_pipeline_readback invalid_vertex_topology =
            run_pipeline(duplicate_vertex_ids, candidate_pair_seeds, 2, 2);
        require(invalid_vertex_topology.totals[1] == 2u &&
            invalid_vertex_topology.candidates[0].identity_a[0] == 0xffffffffu,
            "GPU Candidate Count must fail closed on duplicate Stable Vertex IDs inside one Primitive.");

        std::vector<gpu_correction_candidate_primitive> duplicate_primitive_identity =
            gpu_primitives;
        duplicate_primitive_identity[1].stable_id_low =
            duplicate_primitive_identity[0].stable_id_low;
        duplicate_primitive_identity[1].stable_id_high =
            duplicate_primitive_identity[0].stable_id_high;
        duplicate_primitive_identity[1].mesh_id =
            duplicate_primitive_identity[0].mesh_id;
        duplicate_primitive_identity[1].primitive_id =
            duplicate_primitive_identity[0].primitive_id;
        const gpu_pipeline_readback invalid_primitive_identity =
            run_pipeline(duplicate_primitive_identity, candidate_pair_seeds, 2, 2);
        require(invalid_primitive_identity.totals[1] == 2u &&
            invalid_primitive_identity.candidates[0].identity_a[0] == 0xffffffffu,
            "GPU Candidate Count must reject duplicate Stable / Mesh / Primitive identities without publishing a partial Candidate Set.");

        std::vector<gpu_uint2> repeated_pair_seeds =
            candidate_pair_seeds;
        repeated_pair_seeds.push_back(candidate_pair_seeds.front());
        const gpu_pipeline_readback repeated_pair_result =
            run_pipeline(gpu_primitives, repeated_pair_seeds, 2, 2);
        require(repeated_pair_result.totals[0] == 2u &&
            repeated_pair_result.totals[1] == 0u &&
            compare_pair(repeated_pair_result.candidates[0],
                cpu_reference.candidates[0]) &&
            compare_pair(repeated_pair_result.candidates[1],
                cpu_reference.candidates[1]),
            "GPU Candidate Count must ignore duplicate broadphase Pair Seeds before Capacity accounting.");

        const std::vector<gpu_correction_candidate_primitive> isolated_primitive = {
            gpu_primitives[0]
        };
        const gpu_pipeline_readback isolated_result =
            run_pipeline(isolated_primitive, {}, 1, 1);
        require(isolated_result.totals[0] == 0u &&
            isolated_result.totals[1] == 0u &&
            isolated_result.candidates[0].identity_a[0] == 0xffffffffu,
            "A valid isolated Primitive with zero Candidate Pair Seeds must complete validation and return an empty Candidate Set.");

        std::vector<gpu_correction_candidate_primitive> invalid_isolated_primitive = {
            gpu_primitives[0]
        };
        invalid_isolated_primitive[0].role = 99u;
        const gpu_pipeline_readback invalid_isolated_result =
            run_pipeline(invalid_isolated_primitive, {}, 1, 1);
        require(invalid_isolated_result.totals[0] == 0u &&
            invalid_isolated_result.totals[1] == 2u &&
            invalid_isolated_result.candidates[0].identity_a[0] == 0xffffffffu,
            "GPU Primitive validation must reject malformed input even when the Candidate Pair Seed list is empty.");

        std::cout << "[PASS] P12_WARP_CORRECTION_CANDIDATE_PIPELINE\n";
    }

    void test_phase08_warp_point_triangle_ccd()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result), "D3D11 WARP device creation must succeed.");

        const test_json::value input = test_json::load(
            test_support::fixture_path("phase08_ccd/point_triangle.input.json"));
        std::vector<gpu_float4> geometry = {
            to_gpu_float4(read_vec3(input.at("point_start")),
                static_cast<float>(input.at("interval_seconds").as_number())),
            to_gpu_float4(read_vec3(input.at("point_end"))),
            to_gpu_float4(read_vec3(input.at("triangle").at(0))),
            to_gpu_float4(read_vec3(input.at("triangle").at(1))),
            to_gpu_float4(read_vec3(input.at("triangle").at(2)))
        };
        const float combined_radius =
            static_cast<float>(input.at("point_radius").as_number() +
                input.at("triangle_thickness").as_number());

        skin_physics::physics::ccd_settings cpu_settings;
        cpu_settings.safety = 0.8f;
        cpu_settings.distance_tolerance = 1.0e-5f;
        cpu_settings.maximum_iterations = 64;
        const auto cpu = skin_physics::physics::conservative_advance_point_triangle(
            from_gpu_float4(geometry[0]),
            from_gpu_float4(geometry[1]),
            { from_gpu_float4(geometry[2]), from_gpu_float4(geometry[3]),
                from_gpu_float4(geometry[4]) },
            combined_radius,
            static_cast<float>(input.at("interval_seconds").as_number()),
            cpu_settings);
        require(cpu.status == skin_physics::physics::ccd_status::hit,
            "CCD fixture must produce a CPU conservative-advancement hit.");

        const std::vector<gpu_float4> initial_output(3);
        const ComPtr<ID3D11Buffer> geometry_buffer =
            create_structured_buffer(device.Get(), geometry, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(device.Get(), initial_output, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11ShaderResourceView> geometry_view =
            create_structured_srv<gpu_float4>(device.Get(), geometry_buffer.Get(), 5);
        const ComPtr<ID3D11UnorderedAccessView> output_view =
            create_structured_uav<gpu_float4>(device.Get(), output_buffer.Get(), 3);
        struct alignas(16) ccd_parameters
        {
            float safety = 0.8f;
            float tolerance = 1.0e-5f;
            float combined = 0.2f;
            uint32_t maximum_iterations = 64;
        } parameters;
        parameters.combined = combined_radius;
        const ComPtr<ID3D11Buffer> constants =
            create_constant_buffer(device.Get(), &parameters, sizeof(parameters));

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_path("phase08_ccd/point_triangle_ccd.cs.hlsl"),
            "point_triangle_ccd");
        dispatch_compute(context.Get(), shader.Get(), { geometry_view.Get() },
            { output_view.Get() }, constants.Get(), 1);
        const std::vector<gpu_float4> gpu =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(), output_buffer.Get(), 3);

        require(gpu[0].y == 1.0f &&
            std::abs(gpu[0].x - cpu.toi_seconds) <= 2.0e-5f &&
            std::abs(gpu[1].x - cpu.barycentric[0]) <= 1.0e-5f &&
            std::abs(gpu[1].y - cpu.barycentric[1]) <= 1.0e-5f &&
            std::abs(gpu[1].z - cpu.barycentric[2]) <= 1.0e-5f,
            "WARP conservative advancement TOI and barycentric coordinates must match CPU.");
        require(gpu[0].x >= 0.0f &&
            std::abs(gpu[0].x - 0.4f) <= 2.0e-5f,
            "WARP CCD must detect the analytical 0.4-second contact before triangle crossing.");
        std::cout << "[PASS] P08_WARP_POINT_TRIANGLE_CCD\n";
    }


    struct alignas(16) gpu_animation_parameters
    {
        gpu_float4 translation_start;
        gpu_float4 translation_end;
        gpu_float4 rotation_start;
        gpu_float4 rotation_end;
        gpu_float4 timing;
        gpu_float4 teleport_target;
    };

    struct alignas(16) gpu_paint_parameters
    {
        uint32_t vertex_count = 0;
        uint32_t paint_mode = 0;
        float target = 0.0f;
        float strength = 0.0f;
        float minimum_weight = 0.0f;
        float maximum_weight = 1.0f;
        float selected_average = 0.0f;
        float reserved = 0.0f;
    };

    void test_phase09_warp_animation_target()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result), "D3D11 WARP device creation must succeed.");

        const test_json::value input = test_json::load(
            test_support::fixture_path("phase09_animation/target_motion.input.json"));
        const auto& translation_track = input.at("translation_track");
        const auto& rotation_track = input.at("rotation_track");
        skin_physics::physics::animation_track cpu_track;
        for (std::size_t index = 0;
            index < translation_track.at("times").as_array().size(); ++index)
        {
            skin_physics::physics::animation_keyframe key;
            key.time = translation_track.at("times").at(index).as_number();
            key.translation = read_vec3(translation_track.at("values").at(index));
            const test_json::value& quaternion =
                rotation_track.at("values").at(index);
            key.rotation = {
                static_cast<float>(quaternion.at(0).as_number()),
                static_cast<float>(quaternion.at(1).as_number()),
                static_cast<float>(quaternion.at(2).as_number()),
                static_cast<float>(quaternion.at(3).as_number())
            };
            cpu_track.keyframes.push_back(key);
        }

        gpu_animation_parameters parameters;
        parameters.translation_start = to_gpu_float4(cpu_track.keyframes[0].translation);
        parameters.translation_end = to_gpu_float4(cpu_track.keyframes[1].translation);
        const auto& q0 = cpu_track.keyframes[0].rotation;
        const auto& q1 = cpu_track.keyframes[1].rotation;
        parameters.rotation_start = { q0.x, q0.y, q0.z, q0.w };
        parameters.rotation_end = { q1.x, q1.y, q1.z, q1.w };
        parameters.timing = {
            static_cast<float>(cpu_track.keyframes[0].time),
            static_cast<float>(cpu_track.keyframes[1].time),
            static_cast<float>(input.at("sample_time").as_number()),
            0.0f
        };

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_path("phase09_10_gpu/animation_paint.cs.hlsl"),
            "animation_target");
        const std::vector<gpu_float4> initial_output(3);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(device.Get(), initial_output, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> output_view =
            create_structured_uav<gpu_float4>(device.Get(), output_buffer.Get(), 3);
        const ComPtr<ID3D11Buffer> constants =
            create_constant_buffer(device.Get(), &parameters, sizeof(parameters));
        dispatch_compute(context.Get(), shader.Get(), {}, { output_view.Get() },
            constants.Get(), 1);
        const std::vector<gpu_float4> gpu =
            readback_structured_buffer<gpu_float4>(device.Get(), context.Get(),
                output_buffer.Get(), 3);

        const auto cpu = skin_physics::physics::sample_animation_track(
            cpu_track, input.at("sample_time").as_number());
        require(cpu.success, "Animation fixture CPU interpolation must succeed.");
        const float tolerance = 1.0e-5f;
        require(std::abs(gpu[0].x - cpu.value.translation.x) <= tolerance &&
            std::abs(gpu[0].y - cpu.value.translation.y) <= tolerance &&
            std::abs(gpu[0].z - cpu.value.translation.z) <= tolerance &&
            std::abs(gpu[1].x - cpu.value.translation_velocity.x) <= tolerance &&
            std::abs(gpu[1].y - cpu.value.translation_velocity.y) <= tolerance &&
            std::abs(gpu[1].z - cpu.value.translation_velocity.z) <= tolerance,
            "WARP animation translation and velocity must match CPU sampling.");
        require(std::abs(gpu[2].x - cpu.value.rotation.x) <= tolerance &&
            std::abs(gpu[2].y - cpu.value.rotation.y) <= tolerance &&
            std::abs(gpu[2].z - cpu.value.rotation.z) <= tolerance &&
            std::abs(gpu[2].w - cpu.value.rotation.w) <= tolerance,
            "WARP quaternion Slerp must match the CPU reference.");

        const test_json::value& teleport = input.at("teleport");
        skin_physics::physics::animation_sample_options options;
        options.teleport = true;
        options.zero_velocity_on_teleport = true;
        options.teleport_translation = read_vec3(teleport.at("to"));
        const auto cpu_teleport = skin_physics::physics::sample_animation_track(
            cpu_track, input.at("sample_time").as_number(), options);
        require(cpu_teleport.success, "Animation teleport reference must succeed.");

        parameters.timing.w = 1.0f;
        parameters.teleport_target = to_gpu_float4(options.teleport_translation, 1.0f);
        const ComPtr<ID3D11Buffer> teleport_constants =
            create_constant_buffer(device.Get(), &parameters, sizeof(parameters));
        dispatch_compute(context.Get(), shader.Get(), {}, { output_view.Get() },
            teleport_constants.Get(), 1);
        const std::vector<gpu_float4> teleport_gpu =
            readback_structured_buffer<gpu_float4>(device.Get(), context.Get(),
                output_buffer.Get(), 3);
        require(std::abs(teleport_gpu[0].x - cpu_teleport.value.translation.x) <= tolerance &&
            std::abs(teleport_gpu[0].y - cpu_teleport.value.translation.y) <= tolerance &&
            std::abs(teleport_gpu[0].z - cpu_teleport.value.translation.z) <= tolerance &&
            std::abs(teleport_gpu[1].x) <= tolerance &&
            std::abs(teleport_gpu[1].y) <= tolerance &&
            std::abs(teleport_gpu[1].z) <= tolerance,
            "WARP teleport policy must replace the target and zero animation velocity.");
        std::cout << "[PASS] P09_WARP_ANIMATION_AND_TELEPORT\n";
    }

    void test_phase10_warp_attribute_paint()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result), "D3D11 WARP device creation must succeed.");

        const test_json::value input = test_json::load(
            test_support::fixture_path("phase10_paint/paint.input.json"));
        std::vector<float> weights;
        std::vector<std::vector<uint32_t>> adjacency;
        std::vector<uint32_t> selected_vertices;
        for (const test_json::value& weight : input.at("initial_weights").as_array())
        {
            weights.push_back(static_cast<float>(weight.as_number()));
        }
        for (const test_json::value& neighbors : input.at("adjacency").as_array())
        {
            std::vector<uint32_t> values;
            for (const test_json::value& neighbor : neighbors.as_array())
            {
                values.push_back(static_cast<uint32_t>(neighbor.as_number()));
            }
            adjacency.push_back(std::move(values));
        }
        std::vector<uint32_t> selected_mask(weights.size(), 1u);
        for (std::size_t index = 0; index < weights.size(); ++index)
        {
            selected_vertices.push_back(static_cast<uint32_t>(index));
        }
        const float minimum_weight = static_cast<float>(input.at("clamp_range").at(0).as_number());
        const float maximum_weight = static_cast<float>(input.at("clamp_range").at(1).as_number());

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_path("phase09_10_gpu/animation_paint.cs.hlsl"),
            "attribute_paint");

        const auto& operations = input.at("operations").as_array();
        for (const test_json::value& operation : operations)
        {
            const std::string& op_name = operation.at("op").as_string();
            skin_physics::physics::attribute_paint_mode mode =
                skin_physics::physics::attribute_paint_mode::paint;
            uint32_t gpu_mode = 0;
            if (op_name == "smooth")
            {
                mode = skin_physics::physics::attribute_paint_mode::smooth;
                gpu_mode = 1;
            }
            else if (op_name == "average")
            {
                mode = skin_physics::physics::attribute_paint_mode::average;
                gpu_mode = 2;
            }
            else if (op_name == "erase")
            {
                mode = skin_physics::physics::attribute_paint_mode::erase;
                gpu_mode = 3;
            }

            const auto target_field = operation.as_object().find("target");
            const auto strength_field = operation.as_object().find("strength");
            const float target = target_field != operation.as_object().end()
                ? static_cast<float>(target_field->second.as_number()) : 0.0f;
            const float strength = strength_field != operation.as_object().end()
                ? static_cast<float>(strength_field->second.as_number()) : 1.0f;
            float selected_average = 0.0f;
            for (uint32_t selected : selected_vertices)
            {
                selected_average += weights[selected];
            }
            selected_average /= static_cast<float>(selected_vertices.size());

            const auto cpu = skin_physics::physics::apply_attribute_paint(
                weights, adjacency, selected_vertices, mode, target, strength,
                minimum_weight, maximum_weight);
            require(cpu.success, "Attribute paint CPU reference operation must succeed.");

            const ComPtr<ID3D11Buffer> source_buffer =
                create_structured_buffer(device.Get(), weights, D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> selection_buffer =
                create_structured_buffer(device.Get(), selected_mask, D3D11_BIND_SHADER_RESOURCE);
            const std::vector<float> initial_output(weights.size(), -1.0f);
            const ComPtr<ID3D11Buffer> output_buffer =
                create_structured_buffer(device.Get(), initial_output, D3D11_BIND_UNORDERED_ACCESS);
            const ComPtr<ID3D11ShaderResourceView> source_view =
                create_structured_srv<float>(device.Get(), source_buffer.Get(),
                    static_cast<UINT>(weights.size()));
            const ComPtr<ID3D11ShaderResourceView> selection_view =
                create_structured_srv<uint32_t>(device.Get(), selection_buffer.Get(),
                    static_cast<UINT>(selected_mask.size()));
            const ComPtr<ID3D11UnorderedAccessView> output_view =
                create_structured_uav<float>(device.Get(), output_buffer.Get(),
                    static_cast<UINT>(weights.size()));

            gpu_paint_parameters parameters;
            parameters.vertex_count = static_cast<uint32_t>(weights.size());
            parameters.paint_mode = gpu_mode;
            parameters.target = target;
            parameters.strength = strength;
            parameters.minimum_weight = minimum_weight;
            parameters.maximum_weight = maximum_weight;
            parameters.selected_average = selected_average;
            const ComPtr<ID3D11Buffer> constants =
                create_constant_buffer(device.Get(), &parameters, sizeof(parameters));

            ID3D11ShaderResourceView* resources[] = {
                source_view.Get(), selection_view.Get()
            };
            ID3D11UnorderedAccessView* unordered_views[] = { output_view.Get() };
            ID3D11Buffer* constant_buffers[] = { nullptr, constants.Get() };
            context->CSSetShader(shader.Get(), nullptr, 0);
            context->CSSetShaderResources(0, 2, resources);
            context->CSSetUnorderedAccessViews(0, 1, unordered_views, nullptr);
            context->CSSetConstantBuffers(0, 2, constant_buffers);
            context->Dispatch(1, 1, 1);
            ID3D11ShaderResourceView* null_resources[] = { nullptr, nullptr };
            ID3D11UnorderedAccessView* null_unordered[] = { nullptr };
            ID3D11Buffer* null_constants[] = { nullptr, nullptr };
            context->CSSetShaderResources(0, 2, null_resources);
            context->CSSetUnorderedAccessViews(0, 1, null_unordered, nullptr);
            context->CSSetConstantBuffers(0, 2, null_constants);
            context->CSSetShader(nullptr, nullptr, 0);

            const std::vector<float> gpu = readback_structured_buffer<float>(
                device.Get(), context.Get(), output_buffer.Get(), weights.size());
            for (std::size_t index = 0; index < gpu.size(); ++index)
            {
                require(std::abs(gpu[index] - cpu.weights[index]) <= 1.0e-6f,
                    "WARP attribute paint result must match the CPU operation.");
            }
            weights = gpu;
        }
        std::cout << "[PASS] P10_WARP_ATTRIBUTE_PAINT\n";
    }


    void test_phase11_warp_timestamp_profiler_and_dxgi_memory()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result), "D3D11 WARP device creation must succeed.");

        skin_physics::simulation::d3d11_timestamp_profiler profiler(
            device.Get(), context.Get(), 3);
        require(profiler.valid(),
            "D3D11 timestamp profiler must initialize with a WARP device.");
        require(profiler.register_pass("FixtureClear"),
            "Timestamp pass registration must succeed.");

        const std::vector<gpu_float4> initial_values(4);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(device.Get(), initial_values,
                D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> output_view =
            create_structured_uav<gpu_float4>(
                device.Get(), output_buffer.Get(), 4);

        require(profiler.begin_frame(), "Timestamp frame begin must succeed.");
        require(profiler.begin_pass("FixtureClear"), "Timestamp pass begin must succeed.");
        const float clear_value[4] = { 1.0f, 2.0f, 3.0f, 4.0f };
        context->ClearUnorderedAccessViewFloat(output_view.Get(), clear_value);
        require(profiler.end_pass("FixtureClear"), "Timestamp pass end must succeed.");
        require(profiler.end_frame(), "Timestamp frame end must succeed.");

        context->Flush();
        skin_physics::simulation::timestamp_collection_result collected;
        for (uint32_t attempt = 0; attempt < 128; ++attempt)
        {
            collected = profiler.try_collect();
            if (collected.status !=
                skin_physics::simulation::timestamp_collection_status::not_ready)
            {
                break;
            }
            context->Flush();
        }
        require(collected.status ==
            skin_physics::simulation::timestamp_collection_status::collected,
            "Completed WARP timestamp queries must become available through non-blocking collection.");
        require(collected.pass_timings.size() == 1 &&
            collected.pass_timings[0].pass_name == "FixtureClear" &&
            std::isfinite(collected.pass_timings[0].milliseconds) &&
            collected.pass_timings[0].milliseconds >= 0.0,
            "Timestamp collector must return the named GPU pass duration.");
        const auto empty = profiler.try_collect();
        require(empty.status ==
            skin_physics::simulation::timestamp_collection_status::not_ready,
            "Collected timestamp slots must be released and not returned twice.");

        ComPtr<IDXGIDevice> dxgi_device;
        const HRESULT dxgi_result = device.As(&dxgi_device);
        require(SUCCEEDED(dxgi_result),
            "WARP D3D11 device must expose IDXGIDevice for adapter discovery.");
        ComPtr<IDXGIAdapter> adapter;
        require(SUCCEEDED(dxgi_device->GetAdapter(adapter.GetAddressOf())),
            "WARP adapter discovery must succeed.");
        const auto memory =
            skin_physics::simulation::query_local_video_memory(adapter.Get());
        if (!memory.available)
        {
            require(!memory.diagnostic.empty(),
                "Unavailable DXGI memory information must include an explicit reason.");
        }
        std::cout << "[PASS] P11_WARP_TIMESTAMP_AND_DXGI_MEMORY\n";
    }


    void test_phase11_warp_compute_pass_graph()
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result), "D3D11 WARP device creation must succeed.");

        const test_json::value input = test_json::load(
            test_support::fixture_path("phase10_paint/paint.input.json"));
        std::vector<float> weights;
        std::vector<std::vector<uint32_t>> adjacency;
        std::vector<uint32_t> selected_vertices;
        std::vector<uint32_t> selected_mask;
        for (const test_json::value& weight : input.at("initial_weights").as_array())
        {
            weights.push_back(static_cast<float>(weight.as_number()));
        }
        for (const test_json::value& neighbors : input.at("adjacency").as_array())
        {
            std::vector<uint32_t> values;
            for (const test_json::value& neighbor : neighbors.as_array())
            {
                values.push_back(static_cast<uint32_t>(neighbor.as_number()));
            }
            adjacency.push_back(std::move(values));
        }
        for (const test_json::value& vertex : input.at("operations").at(0).at("vertices").as_array())
        {
            const uint32_t index = static_cast<uint32_t>(vertex.as_number());
            selected_vertices.push_back(index);
        }
        selected_mask.resize(weights.size(), 0u);
        for (uint32_t vertex : selected_vertices)
        {
            selected_mask[vertex] = 1u;
        }

        const float minimum_weight =
            static_cast<float>(input.at("clamp_range").at(0).as_number());
        const float maximum_weight =
            static_cast<float>(input.at("clamp_range").at(1).as_number());

        const auto cpu_paint = skin_physics::physics::apply_attribute_paint(
            weights, adjacency, selected_vertices,
            skin_physics::physics::attribute_paint_mode::paint,
            static_cast<float>(input.at("operations").at(0).at("target").as_number()),
            static_cast<float>(input.at("operations").at(0).at("strength").as_number()),
            minimum_weight, maximum_weight);
        require(cpu_paint.success, "Compute graph Paint CPU reference must succeed.");
        const auto cpu_erase = skin_physics::physics::apply_attribute_paint(
            cpu_paint.weights, adjacency, selected_vertices,
            skin_physics::physics::attribute_paint_mode::erase, 0.0f, 0.5f,
            minimum_weight, maximum_weight);
        require(cpu_erase.success, "Compute graph Erase CPU reference must succeed.");

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_path("phase09_10_gpu/animation_paint.cs.hlsl"),
            "attribute_paint");

        const ComPtr<ID3D11Buffer> source_buffer =
            create_structured_buffer(device.Get(), weights, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> selected_buffer =
            create_structured_buffer(device.Get(), selected_mask, D3D11_BIND_SHADER_RESOURCE);
        const std::vector<float> empty_values(weights.size(), 0.0f);
        const ComPtr<ID3D11Buffer> intermediate_buffer =
            create_structured_buffer(device.Get(), empty_values,
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(device.Get(), empty_values,
                D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11ShaderResourceView> source_view =
            create_structured_srv<float>(device.Get(), source_buffer.Get(),
                static_cast<UINT>(weights.size()));
        const ComPtr<ID3D11ShaderResourceView> selected_view =
            create_structured_srv<uint32_t>(device.Get(), selected_buffer.Get(),
                static_cast<UINT>(selected_mask.size()));
        const ComPtr<ID3D11ShaderResourceView> intermediate_read_view =
            create_structured_srv<float>(device.Get(), intermediate_buffer.Get(),
                static_cast<UINT>(weights.size()));
        const ComPtr<ID3D11UnorderedAccessView> intermediate_write_view =
            create_structured_uav<float>(device.Get(), intermediate_buffer.Get(),
                static_cast<UINT>(weights.size()));
        const ComPtr<ID3D11UnorderedAccessView> output_write_view =
            create_structured_uav<float>(device.Get(), output_buffer.Get(),
                static_cast<UINT>(weights.size()));

        gpu_paint_parameters first_parameters;
        first_parameters.vertex_count = static_cast<uint32_t>(weights.size());
        first_parameters.paint_mode = 0;
        first_parameters.target =
            static_cast<float>(input.at("operations").at(0).at("target").as_number());
        first_parameters.strength =
            static_cast<float>(input.at("operations").at(0).at("strength").as_number());
        first_parameters.minimum_weight = minimum_weight;
        first_parameters.maximum_weight = maximum_weight;
        const ComPtr<ID3D11Buffer> first_constants =
            create_constant_buffer(device.Get(), &first_parameters, sizeof(first_parameters));

        gpu_paint_parameters second_parameters = first_parameters;
        second_parameters.paint_mode = 3;
        second_parameters.target = 0.0f;
        second_parameters.strength = 0.5f;
        const ComPtr<ID3D11Buffer> second_constants =
            create_constant_buffer(device.Get(), &second_parameters, sizeof(second_parameters));

        skin_physics::simulation::d3d11_compute_graph graph;
        skin_physics::simulation::compute_pass_desc first_pass;
        first_pass.name = "Paint";
        first_pass.shader = shader.Get();
        first_pass.shader_resources = {
            { 0, source_view.Get() },
            { 1, selected_view.Get() }
        };
        first_pass.unordered_access_views = {
            { 0, intermediate_write_view.Get(), 0xffffffffu }
        };
        first_pass.constant_buffers = {
            { 1, first_constants.Get() }
        };
        first_pass.group_count_x = 1;

        skin_physics::simulation::compute_pass_desc second_pass;
        second_pass.name = "Erase";
        second_pass.shader = shader.Get();
        second_pass.shader_resources = {
            { 0, intermediate_read_view.Get() },
            { 1, selected_view.Get() }
        };
        second_pass.unordered_access_views = {
            { 0, output_write_view.Get(), 0xffffffffu }
        };
        second_pass.constant_buffers = {
            { 1, second_constants.Get() }
        };
        second_pass.group_count_x = 1;

        std::string diagnostic;
        require(graph.add_pass(first_pass, diagnostic),
            "Compute graph must accept a valid Paint pass: " + diagnostic);
        require(graph.add_pass(second_pass, diagnostic),
            "Compute graph must accept a sequential Erase pass: " + diagnostic);
        require(graph.pass_count() == 2, "Compute graph must preserve explicit pass order.");
        require(!graph.add_pass(first_pass, diagnostic) &&
            !diagnostic.empty(),
            "Compute graph must reject duplicate pass names.");

        const ComPtr<ID3D11Buffer> alias_buffer =
            create_structured_buffer(device.Get(), empty_values,
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11ShaderResourceView> alias_read_view =
            create_structured_srv<float>(device.Get(), alias_buffer.Get(),
                static_cast<UINT>(weights.size()));
        const ComPtr<ID3D11UnorderedAccessView> alias_write_view =
            create_structured_uav<float>(device.Get(), alias_buffer.Get(),
                static_cast<UINT>(weights.size()));
        skin_physics::simulation::compute_pass_desc invalid_alias;
        invalid_alias.name = "InvalidAlias";
        invalid_alias.shader = shader.Get();
        invalid_alias.shader_resources = { { 0, alias_read_view.Get() } };
        invalid_alias.unordered_access_views = {
            { 0, alias_write_view.Get(), 0xffffffffu }
        };
        require(!graph.add_pass(invalid_alias, diagnostic) &&
            diagnostic.find("same resource") != std::string::npos,
            "Compute graph must reject SRV/UAV aliasing within a pass.");

        const auto graph_result = graph.execute(context.Get());
        require(graph_result.success && graph_result.executed_pass_count == 2,
            "Compute graph must dispatch both passes in insertion order.");
        const std::vector<float> gpu = readback_structured_buffer<float>(
            device.Get(), context.Get(), output_buffer.Get(), weights.size());
        for (std::size_t index = 0; index < gpu.size(); ++index)
        {
            require(std::abs(gpu[index] - cpu_erase.weights[index]) <= 1.0e-6f,
                "Compute graph SRV-to-UAV pass transition must match the CPU reference.");
        }
        const auto invalid_context = graph.execute(nullptr);
        require(!invalid_context.success && !invalid_context.diagnostic.empty(),
            "Compute graph must reject a null immediate context.");

        std::cout << "[PASS] P11_WARP_COMPUTE_PASS_GRAPH\n";
    }


    void test_phase12_warp_gpu_generated_indirect_dispatch(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* produce_count_shader,
        ID3D11ComputeShader* build_arguments_shader,
        ID3D11ComputeShader* consume_arguments_shader)
    {
        using namespace skin_physics::simulation;

        const std::vector<uint32_t> zero_count = { 0u };
        const ComPtr<ID3D11Buffer> count_buffer =
            create_structured_buffer(
                device, zero_count,
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> count_uav =
            create_structured_uav<uint32_t>(
                device, count_buffer.Get(), 1);
        const ComPtr<ID3D11ShaderResourceView> count_srv =
            create_structured_srv<uint32_t>(
                device, count_buffer.Get(), 1);

        D3D11_BUFFER_DESC argument_buffer_description = {};
        argument_buffer_description.ByteWidth = sizeof(uint32_t) * 3u;
        argument_buffer_description.Usage = D3D11_USAGE_DEFAULT;
        argument_buffer_description.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
        argument_buffer_description.MiscFlags =
            D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS;

        ComPtr<ID3D11Buffer> argument_buffer;
        require(SUCCEEDED(device->CreateBuffer(
                &argument_buffer_description, nullptr,
                argument_buffer.GetAddressOf())),
            "WARP must create a GPU-writable DispatchIndirect argument buffer.");

        D3D11_UNORDERED_ACCESS_VIEW_DESC argument_uav_description = {};
        argument_uav_description.Format = DXGI_FORMAT_R32_UINT;
        argument_uav_description.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
        argument_uav_description.Buffer.FirstElement = 0;
        argument_uav_description.Buffer.NumElements = 3;

        ComPtr<ID3D11UnorderedAccessView> argument_uav;
        require(SUCCEEDED(device->CreateUnorderedAccessView(
                argument_buffer.Get(), &argument_uav_description,
                argument_uav.GetAddressOf())),
            "WARP must create a typed UAV for the DispatchIndirect argument buffer.");

        const std::vector<uint32_t> initial_output(3, 0u);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(device, initial_output, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> output_uav =
            create_structured_uav<uint32_t>(
                device, output_buffer.Get(),
                static_cast<UINT>(initial_output.size()));

        d3d11_compute_graph graph;
        compute_pass_desc count_pass;
        count_pass.name = "ProduceCandidateCount";
        count_pass.shader = produce_count_shader;
        count_pass.unordered_access_views = {
            { 0, count_uav.Get(), 0xffffffffu }
        };

        compute_pass_desc build_pass;
        build_pass.name = "BuildDispatchArguments";
        build_pass.shader = build_arguments_shader;
        build_pass.shader_resources = {
            { 0, count_srv.Get() }
        };
        build_pass.unordered_access_views = {
            { 1, argument_uav.Get(), 0xffffffffu }
        };

        compute_pass_desc consume_pass;
        consume_pass.name = "ConsumeDispatchArguments";
        consume_pass.shader = consume_arguments_shader;
        consume_pass.unordered_access_views = {
            { 2, output_uav.Get(), 0xffffffffu }
        };
        consume_pass.indirect_arguments = argument_buffer.Get();

        compute_pass_desc invalid_alignment = consume_pass;
        invalid_alignment.name = "InvalidIndirectAlignment";
        invalid_alignment.indirect_argument_offset = 2u;

        compute_pass_desc invalid_range = consume_pass;
        invalid_range.name = "InvalidIndirectRange";
        invalid_range.indirect_argument_offset = 4u;

        compute_pass_desc invalid_argument_flags = consume_pass;
        invalid_argument_flags.name = "InvalidIndirectFlags";
        invalid_argument_flags.indirect_arguments = output_buffer.Get();

        compute_pass_desc invalid_uav_alias = consume_pass;
        invalid_uav_alias.name = "InvalidIndirectUavAlias";
        invalid_uav_alias.unordered_access_views.push_back(
            { 1, argument_uav.Get(), 0xffffffffu });

        std::string diagnostic;
        require(!graph.add_pass(invalid_alignment, diagnostic) &&
            diagnostic.find("DWORD-aligned") != std::string::npos,
            "Compute graph must reject a non-DWORD-aligned DispatchIndirect offset.");
        require(!graph.add_pass(invalid_range, diagnostic) &&
            diagnostic.find("twelve readable argument bytes") != std::string::npos,
            "Compute graph must reject an indirect argument range shorter than three UINT values.");
        require(!graph.add_pass(invalid_argument_flags, diagnostic) &&
            diagnostic.find("DRAWINDIRECT_ARGS") != std::string::npos,
            "Compute graph must reject a buffer without the indirect-argument misc flag.");
        require(!graph.add_pass(invalid_uav_alias, diagnostic) &&
            diagnostic.find("unbound from UAV") != std::string::npos,
            "Compute graph must reject an indirect argument buffer also bound as UAV in the consumer pass.");

        require(graph.add_pass(count_pass, diagnostic),
            "Compute graph must accept a GPU Candidate Count pass: " + diagnostic);
        require(graph.add_pass(build_pass, diagnostic),
            "Compute graph must accept a GPU argument builder pass: " + diagnostic);
        require(graph.add_pass(consume_pass, diagnostic),
            "Compute graph must accept an indirect consumer pass: " + diagnostic);
        const compute_graph_result execution = graph.execute(context);
        require(execution.success && execution.executed_pass_count == 3,
            "Compute graph must execute Count, argument generation and indirect consumption in explicit order.");

        const std::vector<uint32_t> actual_output =
            readback_structured_buffer<uint32_t>(
                device, context, output_buffer.Get(), initial_output.size());
        require(actual_output == std::vector<uint32_t>{ 0x5a00u, 0x5a01u, 0x5a02u },
            "GPU Candidate Count must generate DispatchIndirect arguments without CPU group-count readback.");

        std::cout << "[PASS] P12_WARP_GPU_GENERATED_INDIRECT_DISPATCH\\n";
    }


    void test_phase09_warp_animation_trajectory_packet_abi()
    {
        using skin_physics::physics::animation_blend_gpu_record;
        using skin_physics::physics::animation_channel_gpu_record;
        using skin_physics::physics::animation_span_gpu_record;
        using skin_physics::physics::animation_span_cursor_gpu_record;
        using skin_physics::physics::animation_track_gpu_record;
        using skin_physics::physics::gpu_float4;
        using skin_physics::physics::gpu_matrix4x4;
        using skin_physics::physics::kinematic_collider_gpu_record;
        using skin_physics::physics::root_transform_gpu_record;
        using skin_physics::physics::root_transform_flag_valid_mask;

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Animation Trajectory ABI regression requires a D3D11 WARP Feature Level 11_0 device.");

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_path(
                "phase09_animation/animation_trajectory_abi.cs.hlsl"),
            "copy_animation_trajectory_abi");

        animation_span_gpu_record span;
        span.span_id = 0x10203040u;
        span.animation_input_version_low = 0x55667788u;
        span.animation_input_version_high = 0x11223344u;
        span.first_track_record = 3u;
        span.track_record_count = 2u;
        span.first_channel_record = 5u;
        span.channel_record_count = 8u;
        span.first_blend_record = 7u;
        span.blend_record_count = 1u;
        span.first_collider_record = 9u;
        span.collider_record_count = 4u;
        span.flags = 0x1234u;
        span.span_duration_seconds = 0.125f;
        span.inverse_span_duration = 8.0f;
        span.speed_bound = 12.5f;
        span.acceleration_bound = 31.25f;
        span.chord_deviation_bound = 0.03125f;
        span.quaternion_normalization_lower_bound = 0.125f;
        span.reserved0 = 0.5f;
        span.reserved1 = 0.75f;

        animation_span_cursor_gpu_record cursor;
        cursor.span_record_index = 11u;
        cursor.animation_input_version_low = span.animation_input_version_low;
        cursor.animation_input_version_high = span.animation_input_version_high;
        cursor.u_segment_start = 0.625f;

        animation_track_gpu_record track;
        track.clip_index = 13u;
        track.first_channel_record = 17u;
        track.channel_record_count = 19u;
        track.flags = 3u;
        track.clip_time_start = 0.25f;
        track.clip_time_delta = -0.125f;
        track.playback_speed = -0.5f;
        track.reserved0 = 0.875f;

        animation_channel_gpu_record channel;
        channel.node_index = 2u;
        channel.parent_index = 1u;
        channel.inverse_bind_record_index = 4u;
        channel.interpolation_flags = 0x21u;
        channel.channel_mask = 7u;
        channel.node_role_flags = 0x1fu;
        channel.reserved0 = 0x12345678u;
        channel.reserved1 = 0x87654321u;
        channel.translation_c0 = { 1.0f, 2.0f, 3.0f, 4.0f };
        channel.translation_c1 = { 5.0f, 6.0f, 7.0f, 8.0f };
        channel.translation_c2 = { 9.0f, 10.0f, 11.0f, 12.0f };
        channel.translation_c3 = { 13.0f, 14.0f, 15.0f, 16.0f };
        channel.rotation_parameter0 = { 17.0f, 18.0f, 19.0f, 20.0f };
        channel.rotation_parameter1 = { 21.0f, 22.0f, 23.0f, 24.0f };
        channel.rotation_parameter2 = { 25.0f, 26.0f, 27.0f, 28.0f };
        channel.rotation_parameter3 = { 29.0f, 30.0f, 31.0f, 32.0f };
        channel.scale_c0 = { 33.0f, 34.0f, 35.0f, 36.0f };
        channel.scale_c1 = { 37.0f, 38.0f, 39.0f, 40.0f };
        channel.scale_c2 = { 41.0f, 42.0f, 43.0f, 44.0f };
        channel.scale_c3 = { 45.0f, 46.0f, 47.0f, 48.0f };

        animation_blend_gpu_record blend;
        blend.source_track_index = 0u;
        blend.target_track_index = 1u;
        blend.weight_curve_flags = 1u;
        blend.reserved0 = 0xaabbccddu;
        blend.weight_coefficients = { 0.125f, 0.25f, 0.5f, 0.75f };
        blend.duration_seconds = 2.0f;
        blend.inverse_duration = 0.5f;
        blend.blend_start_offset_seconds = 0.375f;
        blend.reserved2 = 0.875f;

        gpu_matrix4x4 inverse_bind;
        inverse_bind.column0 = { 1.0f, 2.0f, 3.0f, 4.0f };
        inverse_bind.column1 = { 5.0f, 6.0f, 7.0f, 8.0f };
        inverse_bind.column2 = { 9.0f, 10.0f, 11.0f, 12.0f };
        inverse_bind.column3 = { 13.0f, 14.0f, 15.0f, 16.0f };

        gpu_matrix4x4 static_node;
        static_node.column0 = { 16.0f, 15.0f, 14.0f, 13.0f };
        static_node.column1 = { 12.0f, 11.0f, 10.0f, 9.0f };
        static_node.column2 = { 8.0f, 7.0f, 6.0f, 5.0f };
        static_node.column3 = { 4.0f, 3.0f, 2.0f, 1.0f };

        kinematic_collider_gpu_record collider;
        collider.stable_collider_id_low = 0x10293847u;
        collider.stable_collider_id_high = 0x56473829u;
        collider.span_sample_pair_id = 0xabcdef01u;
        collider.flags = 3u;
        collider.translation_start = { 1.0f, 2.0f, 3.0f, 4.0f };
        collider.translation_end = { 5.0f, 6.0f, 7.0f, 8.0f };
        collider.rotation_start = { 9.0f, 10.0f, 11.0f, 12.0f };
        collider.rotation_end = { 13.0f, 14.0f, 15.0f, 16.0f };
        collider.angular_axis_and_angle = { 17.0f, 18.0f, 19.0f, 20.0f };
        collider.duration_seconds = 0.25f;
        collider.inverse_duration = 4.0f;
        collider.linear_speed_bound = 21.0f;
        collider.angular_speed_bound = 22.0f;
        collider.maximum_local_radius = 23.0f;
        collider.sagitta_bound = 24.0f;
        collider.reserved0 = 25.0f;
        collider.reserved1 = 26.0f;

        const std::vector<animation_span_gpu_record> span_inputs = { span };
        const std::vector<animation_span_cursor_gpu_record> cursor_inputs = { cursor };
        const std::vector<animation_track_gpu_record> track_inputs = { track };
        const std::vector<animation_channel_gpu_record> channel_inputs = { channel };
        const std::vector<animation_blend_gpu_record> blend_inputs = { blend };
        const std::vector<gpu_matrix4x4> inverse_bind_inputs = { inverse_bind };
        const std::vector<gpu_matrix4x4> static_node_inputs = { static_node };
        const std::vector<kinematic_collider_gpu_record> collider_inputs = { collider };

        const ComPtr<ID3D11Buffer> span_input_buffer =
            create_structured_buffer(device.Get(), span_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> cursor_input_buffer =
            create_structured_buffer(device.Get(), cursor_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> track_input_buffer =
            create_structured_buffer(device.Get(), track_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> channel_input_buffer =
            create_structured_buffer(device.Get(), channel_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> blend_input_buffer =
            create_structured_buffer(device.Get(), blend_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> inverse_bind_input_buffer =
            create_structured_buffer(device.Get(), inverse_bind_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> static_node_input_buffer =
            create_structured_buffer(device.Get(), static_node_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> collider_input_buffer =
            create_structured_buffer(device.Get(), collider_inputs, D3D11_BIND_SHADER_RESOURCE);

        const ComPtr<ID3D11ShaderResourceView> span_input_view =
            create_structured_srv<animation_span_gpu_record>(
                device.Get(), span_input_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> cursor_input_view =
            create_structured_srv<animation_span_cursor_gpu_record>(
                device.Get(), cursor_input_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> track_input_view =
            create_structured_srv<animation_track_gpu_record>(
                device.Get(), track_input_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> channel_input_view =
            create_structured_srv<animation_channel_gpu_record>(
                device.Get(), channel_input_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> blend_input_view =
            create_structured_srv<animation_blend_gpu_record>(
                device.Get(), blend_input_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> inverse_bind_input_view =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), inverse_bind_input_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> static_node_input_view =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), static_node_input_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> collider_input_view =
            create_structured_srv<kinematic_collider_gpu_record>(
                device.Get(), collider_input_buffer.Get(), 1u);

        const std::vector<animation_span_gpu_record> empty_spans(1u);
        const std::vector<animation_span_cursor_gpu_record> empty_cursors(1u);
        const std::vector<animation_track_gpu_record> empty_tracks(1u);
        const std::vector<animation_channel_gpu_record> empty_channels(1u);
        const std::vector<animation_blend_gpu_record> empty_blends(1u);
        const std::vector<gpu_matrix4x4> empty_inverse_binds(1u);
        const std::vector<gpu_matrix4x4> empty_static_nodes(1u);
        const std::vector<kinematic_collider_gpu_record> empty_colliders(1u);
        const ComPtr<ID3D11Buffer> span_output_buffer =
            create_structured_buffer(device.Get(), empty_spans, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> cursor_output_buffer =
            create_structured_buffer(device.Get(), empty_cursors, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> track_output_buffer =
            create_structured_buffer(device.Get(), empty_tracks, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> channel_output_buffer =
            create_structured_buffer(device.Get(), empty_channels, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> blend_output_buffer =
            create_structured_buffer(device.Get(), empty_blends, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> inverse_bind_output_buffer =
            create_structured_buffer(device.Get(), empty_inverse_binds, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> static_node_output_buffer =
            create_structured_buffer(device.Get(), empty_static_nodes, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> collider_output_buffer =
            create_structured_buffer(device.Get(), empty_colliders, D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11UnorderedAccessView> span_output_view =
            create_structured_uav<animation_span_gpu_record>(
                device.Get(), span_output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> cursor_output_view =
            create_structured_uav<animation_span_cursor_gpu_record>(
                device.Get(), cursor_output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> track_output_view =
            create_structured_uav<animation_track_gpu_record>(
                device.Get(), track_output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> channel_output_view =
            create_structured_uav<animation_channel_gpu_record>(
                device.Get(), channel_output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> blend_output_view =
            create_structured_uav<animation_blend_gpu_record>(
                device.Get(), blend_output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> inverse_bind_output_view =
            create_structured_uav<gpu_matrix4x4>(
                device.Get(), inverse_bind_output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> static_node_output_view =
            create_structured_uav<gpu_matrix4x4>(
                device.Get(), static_node_output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> collider_output_view =
            create_structured_uav<kinematic_collider_gpu_record>(
                device.Get(), collider_output_buffer.Get(), 1u);

        dispatch_compute(
            context.Get(),
            shader.Get(),
            {
                span_input_view.Get(),
                cursor_input_view.Get(),
                track_input_view.Get(),
                channel_input_view.Get(),
                blend_input_view.Get(),
                inverse_bind_input_view.Get(),
                static_node_input_view.Get(),
                collider_input_view.Get()
            },
            {
                span_output_view.Get(),
                cursor_output_view.Get(),
                track_output_view.Get(),
                channel_output_view.Get(),
                blend_output_view.Get(),
                inverse_bind_output_view.Get(),
                static_node_output_view.Get(),
                collider_output_view.Get()
            },
            nullptr,
            1u);

        const std::vector<animation_span_gpu_record> span_output =
            readback_structured_buffer<animation_span_gpu_record>(
                device.Get(), context.Get(), span_output_buffer.Get(), 1u);
        const std::vector<animation_span_cursor_gpu_record> cursor_output =
            readback_structured_buffer<animation_span_cursor_gpu_record>(
                device.Get(), context.Get(), cursor_output_buffer.Get(), 1u);
        const std::vector<animation_track_gpu_record> track_output =
            readback_structured_buffer<animation_track_gpu_record>(
                device.Get(), context.Get(), track_output_buffer.Get(), 1u);
        const std::vector<animation_channel_gpu_record> channel_output =
            readback_structured_buffer<animation_channel_gpu_record>(
                device.Get(), context.Get(), channel_output_buffer.Get(), 1u);
        const std::vector<animation_blend_gpu_record> blend_output =
            readback_structured_buffer<animation_blend_gpu_record>(
                device.Get(), context.Get(), blend_output_buffer.Get(), 1u);
        const std::vector<gpu_matrix4x4> inverse_bind_output =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), inverse_bind_output_buffer.Get(), 1u);
        const std::vector<gpu_matrix4x4> static_node_output =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), static_node_output_buffer.Get(), 1u);
        const std::vector<kinematic_collider_gpu_record> collider_output =
            readback_structured_buffer<kinematic_collider_gpu_record>(
                device.Get(), context.Get(), collider_output_buffer.Get(), 1u);

        require(std::memcmp(span_inputs.data(), span_output.data(),
                sizeof(animation_span_gpu_record)) == 0 &&
            std::memcmp(cursor_inputs.data(), cursor_output.data(),
                sizeof(animation_span_cursor_gpu_record)) == 0 &&
            std::memcmp(track_inputs.data(), track_output.data(),
                sizeof(animation_track_gpu_record)) == 0 &&
            std::memcmp(channel_inputs.data(), channel_output.data(),
                sizeof(animation_channel_gpu_record)) == 0 &&
            std::memcmp(blend_inputs.data(), blend_output.data(),
                sizeof(animation_blend_gpu_record)) == 0 &&
            std::memcmp(inverse_bind_inputs.data(), inverse_bind_output.data(),
                sizeof(gpu_matrix4x4)) == 0 &&
            std::memcmp(static_node_inputs.data(), static_node_output.data(),
                sizeof(gpu_matrix4x4)) == 0 &&
            std::memcmp(collider_inputs.data(), collider_output.data(),
                sizeof(kinematic_collider_gpu_record)) == 0,
            "D3D11 WARP StructuredBuffer / UAV Readback must preserve every fixed Animation Trajectory ABI field byte-for-byte.");
        std::cout << "[PASS] P09_WARP_ANIMATION_TRAJECTORY_PACKET_ABI\n";
    }


    void test_phase09_warp_animation_skin_abi()
    {
        using skin_physics::physics::animation_skin_gpu_record;
        using skin_physics::physics::animation_skin_joint_gpu_record;

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Animation Skin ABI regression requires a D3D11 WARP Feature Level 11_0 device.");

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_path(
                "phase09_animation/animation_skin_abi.cs.hlsl"),
            "copy_animation_skin_abi");

        const std::vector<animation_skin_gpu_record> skin_inputs = {
            { 17u, 3u, 11u, 5u }
        };
        const std::vector<animation_skin_joint_gpu_record> joint_inputs = {
            { 19u, 23u, 29u, 31u }
        };

        const ComPtr<ID3D11Buffer> skin_input_buffer =
            create_structured_buffer(
                device.Get(), skin_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> joint_input_buffer =
            create_structured_buffer(
                device.Get(), joint_inputs, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> skin_input_view =
            create_structured_srv<animation_skin_gpu_record>(
                device.Get(), skin_input_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> joint_input_view =
            create_structured_srv<animation_skin_joint_gpu_record>(
                device.Get(), joint_input_buffer.Get(), 1u);

        const std::vector<animation_skin_gpu_record> empty_skins(1u);
        const std::vector<animation_skin_joint_gpu_record> empty_joints(1u);
        const ComPtr<ID3D11Buffer> skin_output_buffer =
            create_structured_buffer(
                device.Get(), empty_skins, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> joint_output_buffer =
            create_structured_buffer(
                device.Get(), empty_joints, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> skin_output_view =
            create_structured_uav<animation_skin_gpu_record>(
                device.Get(), skin_output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> joint_output_view =
            create_structured_uav<animation_skin_joint_gpu_record>(
                device.Get(), joint_output_buffer.Get(), 1u);

        dispatch_compute(
            context.Get(),
            shader.Get(),
            { skin_input_view.Get(), joint_input_view.Get() },
            { skin_output_view.Get(), joint_output_view.Get() },
            nullptr,
            1u);

        const std::vector<animation_skin_gpu_record> skin_outputs =
            readback_structured_buffer<animation_skin_gpu_record>(
                device.Get(), context.Get(), skin_output_buffer.Get(), 1u);
        const std::vector<animation_skin_joint_gpu_record> joint_outputs =
            readback_structured_buffer<animation_skin_joint_gpu_record>(
                device.Get(), context.Get(), joint_output_buffer.Get(), 1u);

        require(std::memcmp(
                skin_inputs.data(), skin_outputs.data(),
                sizeof(animation_skin_gpu_record)) == 0 &&
            std::memcmp(
                joint_inputs.data(), joint_outputs.data(),
                sizeof(animation_skin_joint_gpu_record)) == 0,
            "WARP StructuredBuffer / UAV Readback must preserve every Skin / Joint mapping field byte-for-byte.");
        std::cout << "[PASS] P09_WARP_ANIMATION_SKIN_JOINT_ABI\n";
    }


    void test_phase09_warp_animation_trajectory_packet_upload()
    {
        using skin_physics::physics::animation_blend_gpu_record;
        using skin_physics::physics::animation_channel_gpu_record;
        using skin_physics::physics::animation_skin_gpu_record;
        using skin_physics::physics::animation_skin_joint_gpu_record;
        using skin_physics::physics::animation_span_cursor_gpu_record;
        using skin_physics::physics::animation_span_flag_bounds_certified;
        using skin_physics::physics::animation_span_flag_has_blend;
        using skin_physics::physics::animation_span_flag_has_kinematic_collider;
        using skin_physics::physics::animation_span_gpu_record;
        using skin_physics::physics::animation_track_gpu_record;
        using skin_physics::physics::animation_trajectory_gpu_resources;
        using skin_physics::physics::animation_trajectory_packet;
        using skin_physics::physics::gpu_matrix4x4;
        using skin_physics::physics::kinematic_collider_flag_valid;
        using skin_physics::physics::kinematic_collider_gpu_record;
        using skin_physics::physics::root_transform_gpu_record;
        using skin_physics::physics::root_transform_flag_valid_mask;

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Animation Trajectory Packet Upload regression requires a D3D11 WARP Feature Level 11_0 device.");

        animation_trajectory_packet packet;
        packet.animation_input_version = 0x1122334455667788ull;
        packet.span_world_time_starts = { 3.25 };
        packet.span_records.resize(1u);
        packet.track_records.resize(1u);
        packet.channel_records.resize(1u);
        packet.inverse_bind_matrices.resize(1u);
        packet.static_node_matrices.resize(2u);
        packet.skin_records.resize(1u);
        packet.skin_joint_records.resize(1u);
        packet.collider_records.resize(1u);
        packet.root_transform_records.resize(1u);

        animation_span_gpu_record& span = packet.span_records[0u];
        span.span_id = 0x1234u;
        span.flags =
            animation_span_flag_bounds_certified |
            animation_span_flag_has_kinematic_collider |
            skin_physics::physics::animation_span_flag_has_root_motion;
        span.animation_input_version_low =
            static_cast<uint32_t>(packet.animation_input_version & 0xffffffffull);
        span.animation_input_version_high =
            static_cast<uint32_t>(packet.animation_input_version >> 32u);
        span.first_track_record = 0u;
        span.track_record_count = 1u;
        span.first_channel_record = 0u;
        span.channel_record_count = 1u;
        span.first_blend_record = 0u;
        span.blend_record_count = 0u;
        span.first_collider_record = 0u;
        span.collider_record_count = 1u;
        span.span_duration_seconds = 0.5f;
        span.inverse_span_duration = 2.0f;
        span.speed_bound = 3.0f;
        span.acceleration_bound = 4.0f;
        span.chord_deviation_bound = 0.125f;
        span.quaternion_normalization_lower_bound = 0.75f;

        animation_track_gpu_record& track = packet.track_records[0u];
        track.clip_index = 2u;
        track.first_channel_record = 0u;
        track.channel_record_count = 1u;
        track.clip_time_start = 0.25f;
        track.clip_time_delta = 0.5f;
        track.playback_speed = 1.0f;

        animation_channel_gpu_record& channel = packet.channel_records[0u];
        channel.node_index = 1u;
        channel.parent_index = skin_physics::asset::invalid_index;
        channel.inverse_bind_record_index = 0u;
        channel.channel_mask =
            skin_physics::physics::animation_channel_mask_translation;
        channel.node_role_flags =
            skin_physics::physics::animation_node_role_joint;
        channel.translation_c0 = { 1.0f, 2.0f, 3.0f, 0.0f };
        channel.translation_c1 = { 4.0f, 5.0f, 6.0f, 0.0f };

        gpu_matrix4x4& inverse_bind = packet.inverse_bind_matrices[0u];
        inverse_bind.column0 = { 1.0f, 0.0f, 0.0f, 0.0f };
        inverse_bind.column1 = { 0.0f, 1.0f, 0.0f, 0.0f };
        inverse_bind.column2 = { 0.0f, 0.0f, 1.0f, 0.0f };
        inverse_bind.column3 = { 2.0f, 3.0f, 4.0f, 1.0f };
        packet.static_node_matrices[0u] = gpu_matrix4x4{};
        packet.static_node_matrices[1u] = inverse_bind;

        packet.skin_records[0u] = { 0u, 0u, 0u, 1u };
        packet.skin_joint_records[0u] = { 0u, 0u, 1u, 0u };

        kinematic_collider_gpu_record& collider =
            packet.collider_records[0u];
        collider.stable_collider_id_low = 5u;
        collider.stable_collider_id_high = 2u;
        collider.span_sample_pair_id = 7u;
        collider.flags = kinematic_collider_flag_valid;
        collider.translation_start = { 0.0f, 0.0f, 0.0f, 1.0f };
        collider.translation_end = { 1.0f, 0.0f, 0.0f, 1.0f };
        collider.rotation_start = { 0.0f, 0.0f, 0.0f, 1.0f };
        collider.rotation_end = { 0.0f, 0.0f, 0.0f, 1.0f };
        collider.angular_axis_and_angle = {};
        collider.duration_seconds = 0.5f;
        collider.inverse_duration = 2.0f;
        collider.linear_speed_bound = 2.0f;
        collider.angular_speed_bound = 0.0f;
        collider.maximum_local_radius = 1.0f;
        collider.sagitta_bound = 0.0f;

        root_transform_gpu_record& root =
            packet.root_transform_records[0u];
        root.translation_start = { 1.0f, 2.0f, 3.0f, 1.0f };
        root.translation_end = { 2.0f, 2.0f, 3.0f, 1.0f };
        root.rotation_start = { 0.0f, 0.0f, 0.0f, 1.0f };
        root.rotation_end = {
            0.0f, 0.0f,
            std::sin(3.14159265358979323846f / 4.0f),
            std::cos(3.14159265358979323846f / 4.0f)
        };
        root.angular_axis_and_angle = {
            0.0f, 0.0f, 1.0f,
            3.14159265358979323846f / 2.0f
        };
        root.duration_seconds = 0.5f;
        root.inverse_duration = 2.0f;
        root.flags = root_transform_flag_valid_mask;

        animation_span_cursor_gpu_record cursor;
        cursor.span_record_index = 0u;
        cursor.animation_input_version_low = span.animation_input_version_low;
        cursor.animation_input_version_high = span.animation_input_version_high;
        cursor.u_segment_start = 0.375f;

        animation_trajectory_gpu_resources resources;
        std::string diagnostic;
        const bool uploaded = skin_physics::physics::upload_animation_trajectory_packet(
            device.Get(), packet, cursor, resources, diagnostic);
        require(uploaded && resources.valid(),
            "A valid CPU Animation Packet must upload every immutable StructuredBuffer and its independent mutable Cursor: " +
            diagnostic);
        require(resources.animation_input_version() == packet.animation_input_version &&
            resources.counts().spans == 1u &&
            resources.counts().tracks == 1u &&
            resources.counts().channels == 1u &&
            resources.counts().blends == 0u &&
            resources.counts().inverse_bind_matrices == 1u &&
            resources.counts().static_node_matrices == 2u &&
            resources.counts().skins == 1u &&
            resources.counts().skin_joints == 1u &&
            resources.counts().colliders == 1u &&
            resources.counts().root_transforms == 1u,
            "GPU resource metadata must preserve the Packet Version and every structured record count.");

        require_immutable_structured_buffer(
            resources.span_buffer(), 1u, sizeof(animation_span_gpu_record), "Span");
        require_immutable_structured_buffer(
            resources.track_buffer(), 1u, sizeof(animation_track_gpu_record), "Track");
        require_immutable_structured_buffer(
            resources.channel_buffer(), 1u, sizeof(animation_channel_gpu_record), "Channel");
        require_immutable_structured_buffer(
            resources.inverse_bind_matrix_buffer(), 1u, sizeof(gpu_matrix4x4), "Inverse Bind Matrix");
        require_immutable_structured_buffer(
            resources.static_node_matrix_buffer(), 2u, sizeof(gpu_matrix4x4), "Static Node Matrix");
        require_immutable_structured_buffer(
            resources.skin_buffer(), 1u, sizeof(animation_skin_gpu_record), "Skin");
        require_immutable_structured_buffer(
            resources.skin_joint_buffer(), 1u, sizeof(animation_skin_joint_gpu_record), "Skin Joint");
        require_immutable_structured_buffer(
            resources.collider_buffer(), 1u, sizeof(kinematic_collider_gpu_record), "Kinematic Collider");
        require_immutable_structured_buffer(
            resources.root_transform_buffer(), 1u, sizeof(root_transform_gpu_record), "Root Transform");

        require_structured_srv_element_count(
            resources.span_srv(), 1u, "Span");
        require_structured_srv_element_count(
            resources.track_srv(), 1u, "Track");
        require_structured_srv_element_count(
            resources.channel_srv(), 1u, "Channel");
        require_structured_srv_element_count(
            resources.inverse_bind_matrix_srv(), 1u, "Inverse Bind Matrix");
        require_structured_srv_element_count(
            resources.static_node_matrix_srv(), 2u, "Static Node Matrix");
        require_structured_srv_element_count(
            resources.skin_srv(), 1u, "Skin");
        require_structured_srv_element_count(
            resources.skin_joint_srv(), 1u, "Skin Joint");
        require_structured_srv_element_count(
            resources.collider_srv(), 1u, "Kinematic Collider");
        require_structured_srv_element_count(
            resources.root_transform_srv(), 1u, "Root Transform");
        require(resources.blend_buffer() == nullptr &&
            resources.blend_srv() == nullptr &&
            resources.collider_buffer() != nullptr &&
            resources.collider_srv() != nullptr &&
            resources.root_transform_buffer() != nullptr &&
            resources.root_transform_srv() != nullptr,
            "Optional Blend resources must remain absent while the immutable Kinematic Collider record is uploaded.");

        D3D11_BUFFER_DESC cursor_description = {};
        resources.cursor_buffer()->GetDesc(&cursor_description);
        require(cursor_description.ByteWidth == sizeof(animation_span_cursor_gpu_record) &&
            cursor_description.Usage == D3D11_USAGE_DEFAULT &&
            cursor_description.BindFlags ==
                (D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS) &&
            cursor_description.CPUAccessFlags == 0u &&
            cursor_description.MiscFlags == D3D11_RESOURCE_MISC_BUFFER_STRUCTURED &&
            cursor_description.StructureByteStride ==
                sizeof(animation_span_cursor_gpu_record) &&
            resources.cursor_srv() != nullptr &&
            resources.cursor_uav() != nullptr,
            "The per-World Segment Cursor must be a separate mutable StructuredBuffer with SRV / UAV views.");

        const std::vector<animation_span_cursor_gpu_record> cursor_readback =
            readback_structured_buffer<animation_span_cursor_gpu_record>(
                device.Get(), context.Get(), resources.cursor_buffer(), 1u);
        require(std::memcmp(
                &cursor_readback[0u], &cursor,
                sizeof(animation_span_cursor_gpu_record)) == 0,
            "GPU-uploaded Segment Cursor must preserve the immutable Packet Version and initial normalized coordinate.");

        const std::vector<animation_span_gpu_record> span_readback =
            readback_structured_buffer<animation_span_gpu_record>(
                device.Get(), context.Get(), resources.span_buffer(), 1u);
        const std::vector<animation_track_gpu_record> track_readback =
            readback_structured_buffer<animation_track_gpu_record>(
                device.Get(), context.Get(), resources.track_buffer(), 1u);
        const std::vector<animation_channel_gpu_record> channel_readback =
            readback_structured_buffer<animation_channel_gpu_record>(
                device.Get(), context.Get(), resources.channel_buffer(), 1u);
        const std::vector<animation_skin_gpu_record> skin_readback =
            readback_structured_buffer<animation_skin_gpu_record>(
                device.Get(), context.Get(), resources.skin_buffer(), 1u);
        const std::vector<animation_skin_joint_gpu_record> joint_readback =
            readback_structured_buffer<animation_skin_joint_gpu_record>(
                device.Get(), context.Get(), resources.skin_joint_buffer(), 1u);
        const std::vector<kinematic_collider_gpu_record> collider_readback =
            readback_structured_buffer<kinematic_collider_gpu_record>(
                device.Get(), context.Get(), resources.collider_buffer(), 1u);
        const std::vector<root_transform_gpu_record> root_transform_readback =
            readback_structured_buffer<root_transform_gpu_record>(
                device.Get(), context.Get(), resources.root_transform_buffer(), 1u);
        require(std::memcmp(
                span_readback.data(), packet.span_records.data(),
                sizeof(animation_span_gpu_record)) == 0 &&
            std::memcmp(
                track_readback.data(), packet.track_records.data(),
                sizeof(animation_track_gpu_record)) == 0 &&
            std::memcmp(
                channel_readback.data(), packet.channel_records.data(),
                sizeof(animation_channel_gpu_record)) == 0 &&
            std::memcmp(
                skin_readback.data(), packet.skin_records.data(),
                sizeof(animation_skin_gpu_record)) == 0 &&
            std::memcmp(
                joint_readback.data(), packet.skin_joint_records.data(),
                sizeof(animation_skin_joint_gpu_record)) == 0 &&
            std::memcmp(
                collider_readback.data(), packet.collider_records.data(),
                sizeof(kinematic_collider_gpu_record)) == 0 &&
            std::memcmp(
                root_transform_readback.data(),
                packet.root_transform_records.data(),
                sizeof(root_transform_gpu_record)) == 0,
            "Immutable Packet SRV backing buffers must retain their exact CPU record bytes, including Kinematic Collider trajectories, after GPU upload.");

        ID3D11Buffer* const committed_span_buffer = resources.span_buffer();
        const uint64_t committed_version = resources.animation_input_version();
        animation_span_cursor_gpu_record wrong_version_cursor = cursor;
        wrong_version_cursor.animation_input_version_low ^= 1u;
        const bool rejected_wrong_cursor =
            skin_physics::physics::upload_animation_trajectory_packet(
                device.Get(), packet, wrong_version_cursor, resources, diagnostic);
        require(!rejected_wrong_cursor &&
            resources.valid() &&
            resources.span_buffer() == committed_span_buffer &&
            resources.animation_input_version() == committed_version,
            "A Cursor from another immutable Packet Version must be rejected without replacing the previously uploaded GPU resources.");

        animation_trajectory_packet invalid_blend_packet = packet;
        animation_blend_gpu_record invalid_blend;
        invalid_blend.source_track_index = 0u;
        invalid_blend.target_track_index = 0u;
        invalid_blend.duration_seconds = 0.5f;
        invalid_blend.inverse_duration = 2.0f;
        invalid_blend_packet.blend_records.push_back(invalid_blend);
        invalid_blend_packet.span_records[0u].blend_record_count = 1u;
        invalid_blend_packet.span_records[0u].flags |=
            animation_span_flag_has_blend;
        const bool rejected_invalid_blend =
            skin_physics::physics::upload_animation_trajectory_packet(
                device.Get(), invalid_blend_packet, cursor, resources, diagnostic);
        require(!rejected_invalid_blend &&
            resources.span_buffer() == committed_span_buffer &&
            resources.animation_input_version() == committed_version,
            "A Blend Record referencing one Track as both Source and Target must fail before upload and preserve the previous GPU Packet.");

        animation_trajectory_packet skinning_only_packet = packet;
        skinning_only_packet.span_records[0u].flags &=
            ~animation_span_flag_bounds_certified;
        skinning_only_packet.span_records[0u].speed_bound = 0.0f;
        skinning_only_packet.span_records[0u].acceleration_bound = 0.0f;
        skinning_only_packet.span_records[0u].chord_deviation_bound = 0.0f;
        skinning_only_packet.span_records[0u].quaternion_normalization_lower_bound = 0.0f;
        const bool uploaded_skinning_only_packet =
            skin_physics::physics::upload_animation_trajectory_packet(
                device.Get(), skinning_only_packet, cursor, resources, diagnostic);
        require(uploaded_skinning_only_packet &&
            resources.valid() &&
            resources.animation_input_version() == committed_version,
            "An explicitly uncertified skinning-only Packet must upload while preserving zero motion Bounds for downstream CCD rejection.");

        std::cout << "[PASS] P09_WARP_ANIMATION_TRAJECTORY_PACKET_UPLOAD\n";
    }


    struct gpu_skin_weight_record
    {
        uint32_t joints0[4] = {};
        uint32_t joints1[4] = {};
        float weights0[4] = {};
        float weights1[4] = {};
    };

    static_assert(sizeof(gpu_skin_weight_record) == 64u,
        "GPU Skin Weight WARP fixture must match the Runtime 64-byte Physics Vertex record.");

    struct alignas(16) gpu_skinning_parameters
    {
        uint32_t vertex_count = 0u;
        uint32_t joint_count = 0u;
        uint32_t reserved[2] = {};
    };

    static_assert(sizeof(gpu_skinning_parameters) == 16u);

    void test_phase09_warp_physics_mesh_skinning_targets()
    {
        using skin_physics::asset::mat4;
        using skin_physics::physics::animation_skin_pose;
        using skin_physics::physics::evaluate_physics_mesh_skinning;
        using skin_physics::physics::gpu_matrix4x4;
        using skin_physics::physics::physics_mesh;

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Physics Mesh LBS WARP regression requires a D3D11 WARP Feature Level 11_0 device.");

        animation_skin_pose pose;
        pose.mesh_node_index = 7u;
        pose.skin_index = 3u;
        mat4 joint0;
        joint0.value[12u] = 2.0f;
        mat4 joint1;
        joint1.value[12u] = -1.0f;
        pose.skin_matrices = { joint0, joint1 };
        mat4 joint0_derivative;
        joint0_derivative.value.fill(0.0f);
        joint0_derivative.value[12u] = 2.0f;
        mat4 joint1_derivative;
        joint1_derivative.value.fill(0.0f);
        joint1_derivative.value[12u] = -1.0f;
        pose.skin_matrix_derivatives = { joint0_derivative, joint1_derivative };

        physics_mesh cpu_mesh;
        cpu_mesh.rest_positions = {
            { 1.0f, 2.0f, 3.0f },
            { 0.0f, 1.0f, 0.0f }
        };
        cpu_mesh.has_skin_weights = true;
        cpu_mesh.skin_weights.resize(2u);
        cpu_mesh.skin_weights[0u].joints[0u] = 0u;
        cpu_mesh.skin_weights[0u].weights[0u] = 0.25f;
        cpu_mesh.skin_weights[0u].joints[1u] = 1u;
        cpu_mesh.skin_weights[0u].weights[1u] = 0.75f;
        cpu_mesh.skin_weights[1u].joints[0u] = 1u;
        cpu_mesh.skin_weights[1u].weights[0u] = 1.0f;
        const auto cpu_result = evaluate_physics_mesh_skinning(
            cpu_mesh, 7u, 3u, pose);
        require(cpu_result.success,
            "Physics Mesh CPU LBS reference must evaluate before GPU parity comparison.");

        const std::vector<gpu_float4> rest_positions = {
            { 1.0f, 2.0f, 3.0f, 1.0f },
            { 0.0f, 1.0f, 0.0f, 1.0f }
        };
        std::vector<gpu_skin_weight_record> skin_weights(2u);
        skin_weights[0u].joints0[0u] = 0u;
        skin_weights[0u].weights0[0u] = 0.25f;
        skin_weights[0u].joints0[1u] = 1u;
        skin_weights[0u].weights0[1u] = 0.75f;
        skin_weights[1u].joints0[0u] = 1u;
        skin_weights[1u].weights0[0u] = 1.0f;

        const auto to_gpu_matrix = [](const mat4& source)
        {
            gpu_matrix4x4 result;
            result.column0 = {
                source.value[0u], source.value[1u],
                source.value[2u], source.value[3u]
            };
            result.column1 = {
                source.value[4u], source.value[5u],
                source.value[6u], source.value[7u]
            };
            result.column2 = {
                source.value[8u], source.value[9u],
                source.value[10u], source.value[11u]
            };
            result.column3 = {
                source.value[12u], source.value[13u],
                source.value[14u], source.value[15u]
            };
            return result;
        };

        std::vector<gpu_matrix4x4> skin_matrices = {
            to_gpu_matrix(joint0), to_gpu_matrix(joint1)
        };
        std::vector<gpu_matrix4x4> skin_matrix_derivatives = {
            to_gpu_matrix(joint0_derivative),
            to_gpu_matrix(joint1_derivative)
        };
        const std::vector<gpu_float4> zero_targets(2u);
        const std::vector<uint32_t> zero_status(1u, 0u);

        const ComPtr<ID3D11Buffer> rest_buffer =
            create_structured_buffer(
                device.Get(), rest_positions, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> weights_buffer =
            create_structured_buffer(
                device.Get(), skin_weights, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> skin_matrices_buffer =
            create_structured_buffer(
                device.Get(), skin_matrices, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> skin_derivatives_buffer =
            create_structured_buffer(
                device.Get(), skin_matrix_derivatives, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> target_positions_buffer =
            create_structured_buffer(
                device.Get(), zero_targets, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> target_velocities_buffer =
            create_structured_buffer(
                device.Get(), zero_targets, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> target_status_buffer =
            create_structured_buffer(
                device.Get(), zero_status, D3D11_BIND_UNORDERED_ACCESS);

        const ComPtr<ID3D11ShaderResourceView> rest_srv =
            create_structured_srv<gpu_float4>(
                device.Get(), rest_buffer.Get(), 2u);
        const ComPtr<ID3D11ShaderResourceView> weights_srv =
            create_structured_srv<gpu_skin_weight_record>(
                device.Get(), weights_buffer.Get(), 2u);
        const ComPtr<ID3D11ShaderResourceView> matrices_srv =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), skin_matrices_buffer.Get(), 2u);
        const ComPtr<ID3D11ShaderResourceView> derivatives_srv =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), skin_derivatives_buffer.Get(), 2u);
        const ComPtr<ID3D11UnorderedAccessView> target_positions_uav =
            create_structured_uav<gpu_float4>(
                device.Get(), target_positions_buffer.Get(), 2u);
        const ComPtr<ID3D11UnorderedAccessView> target_velocities_uav =
            create_structured_uav<gpu_float4>(
                device.Get(), target_velocities_buffer.Get(), 2u);
        const ComPtr<ID3D11UnorderedAccessView> target_status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), target_status_buffer.Get(), 1u);

        gpu_skinning_parameters parameters;
        parameters.vertex_count = 2u;
        parameters.joint_count = 2u;
        const ComPtr<ID3D11Buffer> constants =
            create_constant_buffer(device.Get(), &parameters, sizeof(parameters));
        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_directory().parent_path().parent_path() /
                "shaders" / "runtime" / "soft_body_animation_skinning.cs.hlsl",
            "main");

        dispatch_compute(
            context.Get(), shader.Get(),
            {
                rest_srv.Get(), weights_srv.Get(),
                matrices_srv.Get(), derivatives_srv.Get()
            },
            {
                target_positions_uav.Get(),
                target_velocities_uav.Get(),
                target_status_uav.Get()
            },
            constants.Get(),
            1u);

        const std::vector<gpu_float4> gpu_positions =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(), target_positions_buffer.Get(), 2u);
        const std::vector<gpu_float4> gpu_velocities =
            readback_structured_buffer<gpu_float4>(
                device.Get(), context.Get(), target_velocities_buffer.Get(), 2u);
        const std::vector<uint32_t> gpu_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), target_status_buffer.Get(), 1u);
        require(gpu_status[0u] == 0u,
            "Valid Physics Vertex Skin Weights / Skin Matrix inputs must not set the GPU Target status failure bit.");

        for (std::size_t vertex_index = 0u; vertex_index < 2u; ++vertex_index)
        {
            const skin_physics::asset::vec3& expected_position =
                cpu_result.target_positions[vertex_index];
            const skin_physics::asset::vec3& expected_velocity =
                cpu_result.target_velocities[vertex_index];
            require(std::abs(gpu_positions[vertex_index].x - expected_position.x) <= 1.0e-6f &&
                std::abs(gpu_positions[vertex_index].y - expected_position.y) <= 1.0e-6f &&
                std::abs(gpu_positions[vertex_index].z - expected_position.z) <= 1.0e-6f &&
                std::abs(gpu_positions[vertex_index].w - 1.0f) <= 1.0e-6f,
                "WARP LBS Target Position must match the CPU double-precision oracle.");
            require(std::abs(gpu_velocities[vertex_index].x - expected_velocity.x) <= 1.0e-6f &&
                std::abs(gpu_velocities[vertex_index].y - expected_velocity.y) <= 1.0e-6f &&
                std::abs(gpu_velocities[vertex_index].z - expected_velocity.z) <= 1.0e-6f &&
                std::abs(gpu_velocities[vertex_index].w) <= 1.0e-6f,
                "WARP LBS Target Velocity must match the analytic Skin Matrix derivative oracle.");
        }

        std::vector<gpu_skin_weight_record> invalid_weights = skin_weights;
        invalid_weights[0u].joints0[0u] = 2u;
        const ComPtr<ID3D11Buffer> invalid_weights_buffer =
            create_structured_buffer(
                device.Get(), invalid_weights, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> invalid_weights_srv =
            create_structured_srv<gpu_skin_weight_record>(
                device.Get(), invalid_weights_buffer.Get(), 2u);
        const ComPtr<ID3D11Buffer> invalid_status_buffer =
            create_structured_buffer(
                device.Get(), zero_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> invalid_status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), invalid_status_buffer.Get(), 1u);
        dispatch_compute(
            context.Get(), shader.Get(),
            {
                rest_srv.Get(), invalid_weights_srv.Get(),
                matrices_srv.Get(), derivatives_srv.Get()
            },
            {
                target_positions_uav.Get(),
                target_velocities_uav.Get(),
                invalid_status_uav.Get()
            },
            constants.Get(),
            1u);
        const std::vector<uint32_t> invalid_gpu_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), invalid_status_buffer.Get(), 1u);
        require(invalid_gpu_status[0u] == 16u,
            "An influence outside the ordered Skin Joint range must set the GPU LBS-stage diagnostic bit and reject the generated target stream.");

        std::cout << "[PASS] P09_WARP_PHYSICS_MESH_SKINNING_TARGETS\n";
    }

    struct alignas(16) gpu_animation_channel_evaluation_parameters
    {
        uint32_t channel_count = 0u;
        uint32_t first_channel_record = 0u;
        uint32_t expected_span_record_index = 0u;
        uint32_t reserved = 0u;
    };

    struct alignas(16) gpu_animation_node_local_transform_record
    {
        gpu_float4 translation;
        gpu_float4 translation_velocity;
        gpu_float4 rotation;
        gpu_float4 rotation_velocity;
        gpu_float4 scale;
        gpu_float4 scale_velocity;
        uint32_t node_index = 0u;
        uint32_t parent_index = 0u;
        uint32_t node_role_flags = 0u;
        uint32_t status = 0u;
    };

    static_assert(sizeof(gpu_animation_channel_evaluation_parameters) == 16u);
    static_assert(sizeof(gpu_animation_node_local_transform_record) == 112u);

    void test_phase09_warp_animation_channel_evaluation()
    {
        using skin_physics::asset::invalid_index;
        using skin_physics::physics::animation_channel_gpu_record;
        using skin_physics::physics::animation_span_gpu_record;
        using skin_physics::physics::animation_span_cursor_gpu_record;
        using skin_physics::physics::animation_channel_mask_rotation;
        using skin_physics::physics::animation_channel_mask_scale;
        using skin_physics::physics::animation_channel_mask_translation;

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Animation Channel GPU Evaluator requires a D3D11 WARP Feature Level 11_0 device.");

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_directory().parent_path().parent_path() /
                "shaders" / "runtime" / "animation_channel_evaluate.cs.hlsl",
            "main");

        std::vector<animation_channel_gpu_record> channels(3u);
        animation_channel_gpu_record& linear_cubic = channels[0u];
        linear_cubic.node_index = 0u;
        linear_cubic.parent_index = invalid_index;
        linear_cubic.channel_mask =
            animation_channel_mask_translation |
            animation_channel_mask_rotation |
            animation_channel_mask_scale;
        linear_cubic.interpolation_flags =
            1u | (1u << 2u) | (2u << 4u);
        linear_cubic.translation_c0 = { 0.0f, 1.0f, 2.0f, 0.0f };
        linear_cubic.translation_c1 = { 2.0f, 0.0f, 0.0f, 0.0f };
        linear_cubic.rotation_parameter0 = {
            0.0f, 0.0f, 0.0f, 1.0f
        };
        linear_cubic.rotation_parameter1 = {
            0.0f, 0.0f,
            std::sin(3.14159265358979323846f / 4.0f),
            std::cos(3.14159265358979323846f / 4.0f)
        };
        linear_cubic.scale_c0 = { 1.0f, 1.0f, 1.0f, 0.0f };
        linear_cubic.scale_c2 = { 2.0f, 0.0f, 0.0f, 0.0f };

        animation_channel_gpu_record& step = channels[1u];
        step.node_index = 1u;
        step.parent_index = 0u;
        step.channel_mask =
            animation_channel_mask_translation |
            animation_channel_mask_rotation |
            animation_channel_mask_scale;
        step.translation_c0 = { 5.0f, 6.0f, 7.0f, 0.0f };
        step.rotation_parameter0 = { 0.0f, 0.0f, 0.0f, 1.0f };
        step.scale_c0 = { 2.0f, 2.0f, 2.0f, 0.0f };

        animation_channel_gpu_record& cubic_rotation = channels[2u];
        cubic_rotation.node_index = 2u;
        cubic_rotation.parent_index = 0u;
        cubic_rotation.channel_mask = animation_channel_mask_rotation;
        cubic_rotation.interpolation_flags = 2u << 2u;
        cubic_rotation.rotation_parameter0 = {
            0.0f, 0.0f, 0.0f, 1.0f
        };
        cubic_rotation.rotation_parameter1 = {
            0.0f, 0.0f, 1.0f, 0.0f
        };

        const ComPtr<ID3D11Buffer> channel_buffer =
            create_structured_buffer(
                device.Get(), channels, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> channel_srv =
            create_structured_srv<animation_channel_gpu_record>(
                device.Get(), channel_buffer.Get(),
                static_cast<uint32_t>(channels.size()));

        animation_span_gpu_record span_record;
        span_record.span_id = 1u;
        span_record.animation_input_version_low = 0x13579bdfu;
        span_record.animation_input_version_high = 0x2468ace0u;
        span_record.first_channel_record = 0u;
        span_record.channel_record_count =
            static_cast<uint32_t>(channels.size());
        span_record.span_duration_seconds = 0.5f;
        span_record.inverse_span_duration = 2.0f;
        const std::vector<animation_span_gpu_record> span_values = {
            span_record
        };
        const ComPtr<ID3D11Buffer> span_buffer =
            create_structured_buffer(
                device.Get(), span_values, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> span_srv =
            create_structured_srv<animation_span_gpu_record>(
                device.Get(), span_buffer.Get(), 1u);
        animation_span_cursor_gpu_record cursor_record;
        cursor_record.span_record_index = 0u;
        cursor_record.animation_input_version_low =
            span_record.animation_input_version_low;
        cursor_record.animation_input_version_high =
            span_record.animation_input_version_high;
        cursor_record.u_segment_start = 0.5f;
        const std::vector<animation_span_cursor_gpu_record> cursor_values = {
            cursor_record
        };
        const ComPtr<ID3D11Buffer> cursor_buffer =
            create_structured_buffer(
                device.Get(), cursor_values, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> cursor_srv =
            create_structured_srv<animation_span_cursor_gpu_record>(
                device.Get(), cursor_buffer.Get(), 1u);

        const std::vector<gpu_animation_node_local_transform_record> empty_outputs(
            channels.size());
        const std::vector<uint32_t> clear_status(1u, 0u);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(
                device.Get(), empty_outputs, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> status_buffer =
            create_structured_buffer(
                device.Get(), clear_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> output_uav =
            create_structured_uav<gpu_animation_node_local_transform_record>(
                device.Get(), output_buffer.Get(),
                static_cast<uint32_t>(empty_outputs.size()));
        const ComPtr<ID3D11UnorderedAccessView> status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), status_buffer.Get(), 1u);

        gpu_animation_channel_evaluation_parameters parameters;
        parameters.channel_count = static_cast<uint32_t>(channels.size());
        parameters.first_channel_record = 0u;
        parameters.expected_span_record_index = 0u;
        const ComPtr<ID3D11Buffer> parameter_buffer =
            create_constant_buffer(
                device.Get(), &parameters, sizeof(parameters));

        dispatch_compute(
            context.Get(), shader.Get(),
            { channel_srv.Get(), cursor_srv.Get(), span_srv.Get() },
            { output_uav.Get(), status_uav.Get() },
            parameter_buffer.Get(),
            1u);

        const std::vector<gpu_animation_node_local_transform_record> outputs =
            readback_structured_buffer<gpu_animation_node_local_transform_record>(
                device.Get(), context.Get(), output_buffer.Get(),
                static_cast<uint32_t>(channels.size()));
        const std::vector<uint32_t> status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);

        require(status[0u] == 0u &&
            outputs[0u].status == 0u &&
            outputs[1u].status == 0u,
            "Valid immutable Channel records must evaluate without setting the GPU Animation status failure bit.");
        require(outputs[0u].node_index == 0u &&
            outputs[0u].parent_index == invalid_index &&
            std::abs(outputs[0u].translation.x - 1.0f) <= 1.0e-6f &&
            std::abs(outputs[0u].translation.y - 1.0f) <= 1.0e-6f &&
            std::abs(outputs[0u].translation_velocity.x - 4.0f) <= 1.0e-6f &&
            std::abs(outputs[0u].scale.x - 1.5f) <= 1.0e-6f &&
            std::abs(outputs[0u].scale_velocity.x - 4.0f) <= 1.0e-6f,
            "GPU LINEAR Translation and CUBICSPLINE Scale must preserve Span-local values and analytic World-Time derivatives.");

        const float expected_rotation_z =
            std::sin(3.14159265358979323846f / 8.0f);
        const float expected_rotation_w =
            std::cos(3.14159265358979323846f / 8.0f);
        const float rotation_norm = std::sqrt(
            outputs[0u].rotation.x * outputs[0u].rotation.x +
            outputs[0u].rotation.y * outputs[0u].rotation.y +
            outputs[0u].rotation.z * outputs[0u].rotation.z +
            outputs[0u].rotation.w * outputs[0u].rotation.w);
        const float rotation_tangent_dot =
            outputs[0u].rotation.x * outputs[0u].rotation_velocity.x +
            outputs[0u].rotation.y * outputs[0u].rotation_velocity.y +
            outputs[0u].rotation.z * outputs[0u].rotation_velocity.z +
            outputs[0u].rotation.w * outputs[0u].rotation_velocity.w;
        require(std::abs(outputs[0u].rotation.z - expected_rotation_z) <= 1.0e-5f &&
            std::abs(outputs[0u].rotation.w - expected_rotation_w) <= 1.0e-5f &&
            std::abs(rotation_norm - 1.0f) <= 1.0e-5f &&
            std::abs(rotation_tangent_dot) <= 1.0e-5f,
            "GPU LINEAR Quaternion evaluation must use shortest-arc Slerp and return a normalized quaternion with tangent-space derivative.");

        require(outputs[1u].node_index == 1u &&
            outputs[1u].parent_index == 0u &&
            std::abs(outputs[1u].translation.x - 5.0f) <= 1.0e-6f &&
            std::abs(outputs[1u].translation.y - 6.0f) <= 1.0e-6f &&
            std::abs(outputs[1u].translation_velocity.x) <= 1.0e-6f &&
            std::abs(outputs[1u].rotation_velocity.w) <= 1.0e-6f &&
            std::abs(outputs[1u].scale.x - 2.0f) <= 1.0e-6f &&
            std::abs(outputs[1u].scale_velocity.x) <= 1.0e-6f,
            "GPU STEP / Rest TRS channels must remain constant with zero analytic velocity.");

        const float cubic_rotation_norm = std::sqrt(
            outputs[2u].rotation.x * outputs[2u].rotation.x +
            outputs[2u].rotation.y * outputs[2u].rotation.y +
            outputs[2u].rotation.z * outputs[2u].rotation.z +
            outputs[2u].rotation.w * outputs[2u].rotation.w);
        const float cubic_rotation_tangent_dot =
            outputs[2u].rotation.x * outputs[2u].rotation_velocity.x +
            outputs[2u].rotation.y * outputs[2u].rotation_velocity.y +
            outputs[2u].rotation.z * outputs[2u].rotation_velocity.z +
            outputs[2u].rotation.w * outputs[2u].rotation_velocity.w;
        require(outputs[2u].node_index == 2u &&
            std::abs(outputs[2u].rotation.z - 0.4472136f) <= 1.0e-5f &&
            std::abs(outputs[2u].rotation.w - 0.8944272f) <= 1.0e-5f &&
            std::abs(cubic_rotation_norm - 1.0f) <= 1.0e-5f &&
            std::abs(cubic_rotation_tangent_dot) <= 1.0e-5f,
            "GPU CUBICSPLINE Quaternion evaluation must normalize the raw polynomial and project its analytic derivative into the tangent space.");

        std::vector<animation_channel_gpu_record> invalid_channels = channels;
        invalid_channels[0u].rotation_parameter0 = {};
        const ComPtr<ID3D11Buffer> invalid_channel_buffer =
            create_structured_buffer(
                device.Get(), invalid_channels, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> invalid_channel_srv =
            create_structured_srv<animation_channel_gpu_record>(
                device.Get(), invalid_channel_buffer.Get(),
                static_cast<uint32_t>(invalid_channels.size()));
        const ComPtr<ID3D11Buffer> invalid_status_buffer =
            create_structured_buffer(
                device.Get(), clear_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> invalid_status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), invalid_status_buffer.Get(), 1u);
        dispatch_compute(
            context.Get(), shader.Get(),
            { invalid_channel_srv.Get(), cursor_srv.Get(), span_srv.Get() },
            { output_uav.Get(), invalid_status_uav.Get() },
            parameter_buffer.Get(),
            1u);
        const std::vector<uint32_t> invalid_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), invalid_status_buffer.Get(), 1u);
        require(invalid_status[0u] == 1u,
            "A zero-length LINEAR Quaternion input must fail closed and set the GPU Animation status bit.");

        std::cout << "[PASS] P09_WARP_ANIMATION_CHANNEL_EVALUATION\n";
    }

    void test_phase09_warp_animation_blend_local_transforms()
    {
        using skin_physics::physics::animation_blend_flag_enabled;
        using skin_physics::physics::animation_blend_gpu_record;
        using skin_physics::physics::animation_span_gpu_record;
        using skin_physics::physics::animation_span_cursor_gpu_record;

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Animation Blend GPU Evaluator requires a D3D11 WARP Feature Level 11_0 device.");

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_directory().parent_path().parent_path() /
                "shaders" / "runtime" / "animation_blend_local_transform.cs.hlsl",
            "main");

        gpu_animation_node_local_transform_record source;
        source.translation = { 0.0f, 0.0f, 0.0f, 0.0f };
        source.translation_velocity = { 1.0f, 0.0f, 0.0f, 0.0f };
        source.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
        source.scale = { 1.0f, 1.0f, 1.0f, 0.0f };
        source.scale_velocity = { 0.5f, 0.5f, 0.5f, 0.0f };
        source.node_index = 7u;
        source.parent_index = 3u;
        source.node_role_flags = 1u;

        gpu_animation_node_local_transform_record target = source;
        target.translation = { 10.0f, 0.0f, 0.0f, 0.0f };
        target.translation_velocity = { 3.0f, 0.0f, 0.0f, 0.0f };
        target.rotation = {
            0.0f, 0.0f,
            std::sin(3.14159265358979323846f / 4.0f),
            std::cos(3.14159265358979323846f / 4.0f)
        };
        target.scale = { 3.0f, 3.0f, 3.0f, 0.0f };
        target.scale_velocity = { 1.5f, 1.5f, 1.5f, 0.0f };

        const std::vector<gpu_animation_node_local_transform_record> source_values = {
            source
        };
        const std::vector<gpu_animation_node_local_transform_record> target_values = {
            target
        };
        const ComPtr<ID3D11Buffer> source_buffer =
            create_structured_buffer(
                device.Get(), source_values, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> target_buffer =
            create_structured_buffer(
                device.Get(), target_values, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> source_srv =
            create_structured_srv<gpu_animation_node_local_transform_record>(
                device.Get(), source_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> target_srv =
            create_structured_srv<gpu_animation_node_local_transform_record>(
                device.Get(), target_buffer.Get(), 1u);

        animation_blend_gpu_record blend;
        blend.source_track_index = 0u;
        blend.target_track_index = 1u;
        blend.weight_curve_flags = animation_blend_flag_enabled;
        blend.weight_coefficients = { 0.0f, 1.0f, 0.0f, 0.0f };
        blend.duration_seconds = 1.0f;
        blend.inverse_duration = 1.0f;
        blend.blend_start_offset_seconds = 0.0f;
        const std::vector<animation_blend_gpu_record> blend_values = { blend };
        const ComPtr<ID3D11Buffer> blend_buffer =
            create_structured_buffer(
                device.Get(), blend_values, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> blend_srv =
            create_structured_srv<animation_blend_gpu_record>(
                device.Get(), blend_buffer.Get(), 1u);

        animation_span_gpu_record span_record;
        span_record.span_id = 2u;
        span_record.animation_input_version_low = 0x89abcdefu;
        span_record.animation_input_version_high = 0x01234567u;
        span_record.span_duration_seconds = 1.0f;
        span_record.inverse_span_duration = 1.0f;
        span_record.first_blend_record = 0u;
        span_record.blend_record_count = 1u;
        const std::vector<animation_span_gpu_record> span_values = {
            span_record
        };
        const ComPtr<ID3D11Buffer> span_buffer =
            create_structured_buffer(
                device.Get(), span_values, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> span_srv =
            create_structured_srv<animation_span_gpu_record>(
                device.Get(), span_buffer.Get(), 1u);
        animation_span_cursor_gpu_record cursor_record;
        cursor_record.span_record_index = 0u;
        cursor_record.animation_input_version_low =
            span_record.animation_input_version_low;
        cursor_record.animation_input_version_high =
            span_record.animation_input_version_high;
        cursor_record.u_segment_start = 0.5f;
        const std::vector<animation_span_cursor_gpu_record> cursor_values = {
            cursor_record
        };
        const ComPtr<ID3D11Buffer> cursor_buffer =
            create_structured_buffer(
                device.Get(), cursor_values, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> cursor_srv =
            create_structured_srv<animation_span_cursor_gpu_record>(
                device.Get(), cursor_buffer.Get(), 1u);

        const std::vector<gpu_animation_node_local_transform_record> empty_output(1u);
        const std::vector<uint32_t> clear_status(1u, 0u);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(
                device.Get(), empty_output, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> status_buffer =
            create_structured_buffer(
                device.Get(), clear_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> output_uav =
            create_structured_uav<gpu_animation_node_local_transform_record>(
                device.Get(), output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), status_buffer.Get(), 1u);

        struct alignas(16) blend_parameters
        {
            uint32_t node_count = 1u;
            uint32_t expected_span_record_index = 0u;
            uint32_t reserved[2] = {};
        };
        const blend_parameters parameters;
        const ComPtr<ID3D11Buffer> parameter_buffer =
            create_constant_buffer(
                device.Get(), &parameters, sizeof(parameters));

        dispatch_compute(
            context.Get(), shader.Get(),
            { source_srv.Get(), target_srv.Get(), blend_srv.Get(),
                cursor_srv.Get(), span_srv.Get() },
            { output_uav.Get(), status_uav.Get() },
            parameter_buffer.Get(),
            1u);

        const std::vector<gpu_animation_node_local_transform_record> output =
            readback_structured_buffer<gpu_animation_node_local_transform_record>(
                device.Get(), context.Get(), output_buffer.Get(), 1u);
        const std::vector<uint32_t> status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(status[0u] == 0u && output[0u].status == 0u,
            "Valid Source / Target local transform records and Hermite Blend input must not set the GPU Animation failure bit.");
        require(output[0u].node_index == 7u &&
            output[0u].parent_index == 3u &&
            std::abs(output[0u].translation.x - 5.0f) <= 1.0e-6f &&
            std::abs(output[0u].translation_velocity.x - 12.0f) <= 1.0e-5f &&
            std::abs(output[0u].scale.x - 2.0f) <= 1.0e-6f &&
            std::abs(output[0u].scale_velocity.x - 3.0f) <= 1.0e-5f,
            "GPU Blend must include the analytic weight-derivative term in Translation / Scale velocity.");

        const float expected_rotation_z =
            std::sin(3.14159265358979323846f / 8.0f);
        const float expected_rotation_w =
            std::cos(3.14159265358979323846f / 8.0f);
        const float rotation_norm = std::sqrt(
            output[0u].rotation.x * output[0u].rotation.x +
            output[0u].rotation.y * output[0u].rotation.y +
            output[0u].rotation.z * output[0u].rotation.z +
            output[0u].rotation.w * output[0u].rotation.w);
        const float rotation_tangent_dot =
            output[0u].rotation.x * output[0u].rotation_velocity.x +
            output[0u].rotation.y * output[0u].rotation_velocity.y +
            output[0u].rotation.z * output[0u].rotation_velocity.z +
            output[0u].rotation.w * output[0u].rotation_velocity.w;
        require(std::abs(output[0u].rotation.z - expected_rotation_z) <= 1.0e-5f &&
            std::abs(output[0u].rotation.w - expected_rotation_w) <= 1.0e-5f &&
            std::abs(rotation_norm - 1.0f) <= 1.0e-5f &&
            std::abs(rotation_tangent_dot) <= 1.0e-5f,
            "GPU Quaternion Blend must hemisphere-align, normalize and return its analytic tangent-space derivative.");

        cursor_record.u_segment_start = 0.0f;
        context->UpdateSubresource(
            cursor_buffer.Get(), 0u, nullptr, &cursor_record, 0u, 0u);
        blend_parameters start_parameters = parameters;
        const ComPtr<ID3D11Buffer> start_parameter_buffer =
            create_constant_buffer(
                device.Get(), &start_parameters, sizeof(start_parameters));
        dispatch_compute(
            context.Get(), shader.Get(),
            { source_srv.Get(), target_srv.Get(), blend_srv.Get(),
                cursor_srv.Get(), span_srv.Get() },
            { output_uav.Get(), status_uav.Get() },
            start_parameter_buffer.Get(),
            1u);
        const std::vector<gpu_animation_node_local_transform_record>
            start_output =
                readback_structured_buffer<
                    gpu_animation_node_local_transform_record>(
                        device.Get(), context.Get(), output_buffer.Get(), 1u);
        const std::vector<uint32_t> start_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(start_status[0u] == 0u &&
            std::abs(start_output[0u].translation_velocity.x - 11.0f) <= 1.0e-5f,
            "Hermite Blend must preserve its right-sided analytic weight derivative at the exact Blend Start boundary.");

        std::vector<gpu_animation_node_local_transform_record> mismatched_target = {
            target
        };
        mismatched_target[0u].node_index = 8u;
        const ComPtr<ID3D11Buffer> mismatched_target_buffer =
            create_structured_buffer(
                device.Get(), mismatched_target, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> mismatched_target_srv =
            create_structured_srv<gpu_animation_node_local_transform_record>(
                device.Get(), mismatched_target_buffer.Get(), 1u);
        const ComPtr<ID3D11Buffer> mismatch_status_buffer =
            create_structured_buffer(
                device.Get(), clear_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> mismatch_status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), mismatch_status_buffer.Get(), 1u);
        dispatch_compute(
            context.Get(), shader.Get(),
            { source_srv.Get(), mismatched_target_srv.Get(), blend_srv.Get(),
                cursor_srv.Get(), span_srv.Get() },
            { output_uav.Get(), mismatch_status_uav.Get() },
            parameter_buffer.Get(),
            1u);
        const std::vector<uint32_t> mismatch_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), mismatch_status_buffer.Get(), 1u);
        require(mismatch_status[0u] == 2u,
            "Source / Target Node or Parent mismatches must set the GPU Blend-stage diagnostic bit.");

        std::cout << "[PASS] P09_WARP_ANIMATION_BLEND_LOCAL_TRANSFORMS\n";
    }

    void test_phase09_warp_animation_node_hierarchy()
    {
        using skin_physics::asset::invalid_index;
        using skin_physics::physics::gpu_matrix4x4;

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Animation Node Hierarchy GPU Evaluator requires a D3D11 WARP Feature Level 11_0 device.");

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_directory().parent_path().parent_path() /
                "shaders" / "runtime" / "animation_node_hierarchy_evaluate.cs.hlsl",
            "main");

        std::vector<gpu_animation_node_local_transform_record> local_transforms(4u);
        local_transforms[0u].translation = { 1.0f, 0.0f, 0.0f, 0.0f };
        local_transforms[0u].translation_velocity = { 1.0f, 0.0f, 0.0f, 0.0f };
        local_transforms[0u].rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
        local_transforms[0u].scale = { 1.0f, 1.0f, 1.0f, 0.0f };
        local_transforms[0u].node_index = 0u;
        local_transforms[0u].parent_index = invalid_index;

        local_transforms[1u].translation = { 0.0f, 2.0f, 0.0f, 0.0f };
        local_transforms[1u].translation_velocity = { 0.0f, 2.0f, 0.0f, 0.0f };
        local_transforms[1u].rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
        local_transforms[1u].scale = { 1.0f, 1.0f, 1.0f, 0.0f };
        local_transforms[1u].node_index = 1u;
        local_transforms[1u].parent_index = 0u;

        local_transforms[2u].translation = { 0.0f, 0.0f, 3.0f, 0.0f };
        local_transforms[2u].translation_velocity = { 0.0f, 0.0f, 3.0f, 0.0f };
        local_transforms[2u].rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
        local_transforms[2u].scale = { 1.0f, 1.0f, 1.0f, 0.0f };
        local_transforms[2u].node_index = 2u;
        local_transforms[2u].parent_index = 1u;

        local_transforms[3u].node_index = 3u;
        local_transforms[3u].parent_index = 0u;
        local_transforms[3u].node_role_flags = 1u << 4u;

        std::vector<gpu_matrix4x4> static_matrices(4u);
        for (gpu_matrix4x4& matrix : static_matrices)
        {
            matrix.column0 = { 1.0f, 0.0f, 0.0f, 0.0f };
            matrix.column1 = { 0.0f, 1.0f, 0.0f, 0.0f };
            matrix.column2 = { 0.0f, 0.0f, 1.0f, 0.0f };
            matrix.column3 = { 0.0f, 0.0f, 0.0f, 1.0f };
        }
        static_matrices[3u].column3.x = 5.0f;

        const ComPtr<ID3D11Buffer> local_buffer =
            create_structured_buffer(
                device.Get(), local_transforms, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> static_buffer =
            create_structured_buffer(
                device.Get(), static_matrices, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> local_srv =
            create_structured_srv<gpu_animation_node_local_transform_record>(
                device.Get(), local_buffer.Get(),
                static_cast<uint32_t>(local_transforms.size()));
        const ComPtr<ID3D11ShaderResourceView> static_srv =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), static_buffer.Get(),
                static_cast<uint32_t>(static_matrices.size()));

        const std::vector<gpu_matrix4x4> empty_matrices(4u);
        const std::vector<uint32_t> clear_status(1u, 0u);
        const ComPtr<ID3D11Buffer> global_buffer =
            create_structured_buffer(
                device.Get(), empty_matrices, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> derivative_buffer =
            create_structured_buffer(
                device.Get(), empty_matrices, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> status_buffer =
            create_structured_buffer(
                device.Get(), clear_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> global_uav =
            create_structured_uav<gpu_matrix4x4>(
                device.Get(), global_buffer.Get(), 4u);
        const ComPtr<ID3D11UnorderedAccessView> derivative_uav =
            create_structured_uav<gpu_matrix4x4>(
                device.Get(), derivative_buffer.Get(), 4u);
        const ComPtr<ID3D11UnorderedAccessView> status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), status_buffer.Get(), 1u);

        struct alignas(16) hierarchy_parameters
        {
            uint32_t node_count = 4u;
            uint32_t static_node_matrix_count = 4u;
            uint32_t hierarchy_level = 0u;
            uint32_t reserved = 0u;
        };

        for (uint32_t hierarchy_level = 0u;
            hierarchy_level < 3u; ++hierarchy_level)
        {
            hierarchy_parameters parameters;
            parameters.hierarchy_level = hierarchy_level;
            const ComPtr<ID3D11Buffer> parameter_buffer =
                create_constant_buffer(
                    device.Get(), &parameters, sizeof(parameters));
            dispatch_compute(
                context.Get(), shader.Get(),
                { local_srv.Get(), static_srv.Get() },
                { global_uav.Get(), derivative_uav.Get(), status_uav.Get() },
                parameter_buffer.Get(),
                1u);
        }

        const std::vector<gpu_matrix4x4> global_matrices =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), global_buffer.Get(), 4u);
        const std::vector<gpu_matrix4x4> global_derivatives =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), derivative_buffer.Get(), 4u);
        const std::vector<uint32_t> status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(status[0u] == 0u,
            "Valid parent-before-child TRS and static Matrix nodes must not set the GPU Hierarchy failure bit.");
        require(std::abs(global_matrices[0u].column3.x - 1.0f) <= 1.0e-6f &&
            std::abs(global_matrices[1u].column3.x - 1.0f) <= 1.0e-6f &&
            std::abs(global_matrices[1u].column3.y - 2.0f) <= 1.0e-6f &&
            std::abs(global_matrices[2u].column3.x - 1.0f) <= 1.0e-6f &&
            std::abs(global_matrices[2u].column3.y - 2.0f) <= 1.0e-6f &&
            std::abs(global_matrices[2u].column3.z - 3.0f) <= 1.0e-6f &&
            std::abs(global_matrices[3u].column3.x - 6.0f) <= 1.0e-6f,
            "GPU Hierarchy must compose parent-before-child Global Matrices and static affine Matrix nodes in Asset Node space.");
        require(std::abs(global_derivatives[0u].column3.x - 1.0f) <= 1.0e-6f &&
            std::abs(global_derivatives[1u].column3.x - 1.0f) <= 1.0e-6f &&
            std::abs(global_derivatives[1u].column3.y - 2.0f) <= 1.0e-6f &&
            std::abs(global_derivatives[2u].column3.x - 1.0f) <= 1.0e-6f &&
            std::abs(global_derivatives[2u].column3.y - 2.0f) <= 1.0e-6f &&
            std::abs(global_derivatives[2u].column3.z - 3.0f) <= 1.0e-6f &&
            std::abs(global_derivatives[3u].column3.x - 1.0f) <= 1.0e-6f,
            "GPU Hierarchy must propagate Global Matrix Derivatives with the matrix product rule.");

        std::vector<gpu_animation_node_local_transform_record> invalid_locals =
            local_transforms;
        invalid_locals[1u].parent_index = 1u;
        const ComPtr<ID3D11Buffer> invalid_local_buffer =
            create_structured_buffer(
                device.Get(), invalid_locals, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> invalid_local_srv =
            create_structured_srv<gpu_animation_node_local_transform_record>(
                device.Get(), invalid_local_buffer.Get(), 4u);
        const ComPtr<ID3D11Buffer> invalid_status_buffer =
            create_structured_buffer(
                device.Get(), clear_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> invalid_status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), invalid_status_buffer.Get(), 1u);
        hierarchy_parameters invalid_parameters;
        invalid_parameters.hierarchy_level = 0u;
        const ComPtr<ID3D11Buffer> invalid_parameter_buffer =
            create_constant_buffer(
                device.Get(), &invalid_parameters, sizeof(invalid_parameters));
        dispatch_compute(
            context.Get(), shader.Get(),
            { invalid_local_srv.Get(), static_srv.Get() },
            { global_uav.Get(), derivative_uav.Get(), invalid_status_uav.Get() },
            invalid_parameter_buffer.Get(),
            1u);
        const std::vector<uint32_t> invalid_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), invalid_status_buffer.Get(), 1u);
        require(invalid_status[0u] == 4u,
            "Self-parent / non-topological Node references must set the GPU Hierarchy-stage diagnostic bit.");

        std::cout << "[PASS] P09_WARP_ANIMATION_NODE_HIERARCHY\n";
    }

    void test_phase09_warp_animation_skin_matrices()
    {
        using skin_physics::asset::invalid_index;
        using skin_physics::physics::animation_skin_gpu_record;
        using skin_physics::physics::animation_skin_joint_gpu_record;
        using skin_physics::physics::animation_span_gpu_record;
        using skin_physics::physics::animation_span_cursor_gpu_record;
        using skin_physics::physics::gpu_matrix4x4;
        using skin_physics::physics::root_transform_gpu_record;
        using skin_physics::physics::root_transform_flag_valid_mask;

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result),
            "Animation Skin Matrix GPU Evaluator requires a D3D11 WARP Feature Level 11_0 device.");

        const ComPtr<ID3D11ComputeShader> shader = compile_compute_shader(
            device.Get(),
            test_support::fixture_directory().parent_path().parent_path() /
                "shaders" / "runtime" / "animation_skin_matrix_evaluate.cs.hlsl",
            "main");

        std::vector<gpu_animation_node_local_transform_record> local_transforms(2u);
        local_transforms[0u].node_index = 0u;
        local_transforms[0u].parent_index = invalid_index;
        local_transforms[1u].node_index = 1u;
        local_transforms[1u].parent_index = 0u;

        const ComPtr<ID3D11Buffer> local_buffer =
            create_structured_buffer(
                device.Get(), local_transforms, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> local_srv =
            create_structured_srv<gpu_animation_node_local_transform_record>(
                device.Get(), local_buffer.Get(), 2u);

        std::vector<gpu_matrix4x4> global_matrices(2u);
        std::vector<gpu_matrix4x4> global_derivatives(2u);
        for (gpu_matrix4x4& matrix : global_matrices)
        {
            matrix.column0 = { 1.0f, 0.0f, 0.0f, 0.0f };
            matrix.column1 = { 0.0f, 1.0f, 0.0f, 0.0f };
            matrix.column2 = { 0.0f, 0.0f, 1.0f, 0.0f };
            matrix.column3 = { 0.0f, 0.0f, 0.0f, 1.0f };
        }
        global_matrices[0u].column3.x = 2.0f;
        global_matrices[1u].column3.x = 5.0f;
        for (gpu_matrix4x4& matrix : global_derivatives)
        {
            matrix.column0 = {};
            matrix.column1 = {};
            matrix.column2 = {};
            matrix.column3 = {};
        }
        global_derivatives[0u].column3.x = 1.0f;
        global_derivatives[1u].column3.x = 3.0f;

        const ComPtr<ID3D11Buffer> global_buffer =
            create_structured_buffer(
                device.Get(), global_matrices, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> derivative_buffer =
            create_structured_buffer(
                device.Get(), global_derivatives, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> global_srv =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), global_buffer.Get(), 2u);
        const ComPtr<ID3D11ShaderResourceView> derivative_srv =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), derivative_buffer.Get(), 2u);

        const std::vector<animation_skin_gpu_record> skins = {
            { 0u, 0u, 1u, 1u }
        };
        const std::vector<animation_skin_joint_gpu_record> joints = {
            { invalid_index, invalid_index, invalid_index, 0u },
            { 1u, 0u, 1u, 0u }
        };
        const ComPtr<ID3D11Buffer> skin_buffer =
            create_structured_buffer(
                device.Get(), skins, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11Buffer> joint_buffer =
            create_structured_buffer(
                device.Get(), joints, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> skin_srv =
            create_structured_srv<animation_skin_gpu_record>(
                device.Get(), skin_buffer.Get(), 1u);
        const ComPtr<ID3D11ShaderResourceView> joint_srv =
            create_structured_srv<animation_skin_joint_gpu_record>(
                device.Get(), joint_buffer.Get(), 2u);

        std::vector<gpu_matrix4x4> inverse_bind_matrices(1u);
        inverse_bind_matrices[0u].column0 = { 1.0f, 0.0f, 0.0f, 0.0f };
        inverse_bind_matrices[0u].column1 = { 0.0f, 1.0f, 0.0f, 0.0f };
        inverse_bind_matrices[0u].column2 = { 0.0f, 0.0f, 1.0f, 0.0f };
        inverse_bind_matrices[0u].column3 = { -1.0f, 0.0f, 0.0f, 1.0f };
        const ComPtr<ID3D11Buffer> inverse_bind_buffer =
            create_structured_buffer(
                device.Get(), inverse_bind_matrices, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> inverse_bind_srv =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), inverse_bind_buffer.Get(), 1u);

        animation_span_gpu_record span_record;
        span_record.animation_input_version_low = 0x55667788u;
        span_record.animation_input_version_high = 0x11223344u;
        span_record.span_duration_seconds = 1.0f;
        span_record.inverse_span_duration = 1.0f;
        const std::vector<animation_span_gpu_record> span_records = {
            span_record
        };
        const ComPtr<ID3D11Buffer> span_buffer =
            create_structured_buffer(
                device.Get(), span_records, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> span_srv =
            create_structured_srv<animation_span_gpu_record>(
                device.Get(), span_buffer.Get(), 1u);

        animation_span_cursor_gpu_record cursor_record;
        cursor_record.span_record_index = 0u;
        cursor_record.animation_input_version_low =
            span_record.animation_input_version_low;
        cursor_record.animation_input_version_high =
            span_record.animation_input_version_high;
        cursor_record.u_segment_start = 0.25f;
        const std::vector<animation_span_cursor_gpu_record> cursor_records = {
            cursor_record
        };
        const ComPtr<ID3D11Buffer> cursor_buffer =
            create_structured_buffer(
                device.Get(), cursor_records, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> cursor_srv =
            create_structured_srv<animation_span_cursor_gpu_record>(
                device.Get(), cursor_buffer.Get(), 1u);

        root_transform_gpu_record root_record;
        root_record.translation_start = { 10.0f, 0.0f, 0.0f, 1.0f };
        root_record.translation_end = { 10.0f, 0.0f, 0.0f, 1.0f };
        root_record.rotation_start = { 0.0f, 0.0f, 0.0f, 1.0f };
        root_record.rotation_end = { 0.0f, 0.0f, 0.0f, 1.0f };
        root_record.duration_seconds = 1.0f;
        root_record.inverse_duration = 1.0f;
        root_record.flags = root_transform_flag_valid_mask;
        const std::vector<root_transform_gpu_record> root_records = {
            root_record
        };
        const ComPtr<ID3D11Buffer> root_buffer =
            create_structured_buffer(
                device.Get(), root_records, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> root_srv =
            create_structured_srv<root_transform_gpu_record>(
                device.Get(), root_buffer.Get(), 1u);

        const std::vector<gpu_matrix4x4> empty_output(1u);
        const std::vector<uint32_t> clear_status(1u, 0u);
        const ComPtr<ID3D11Buffer> output_buffer =
            create_structured_buffer(
                device.Get(), empty_output, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> output_derivative_buffer =
            create_structured_buffer(
                device.Get(), empty_output, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11Buffer> status_buffer =
            create_structured_buffer(
                device.Get(), clear_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> output_uav =
            create_structured_uav<gpu_matrix4x4>(
                device.Get(), output_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> output_derivative_uav =
            create_structured_uav<gpu_matrix4x4>(
                device.Get(), output_derivative_buffer.Get(), 1u);
        const ComPtr<ID3D11UnorderedAccessView> status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), status_buffer.Get(), 1u);

        struct alignas(16) skin_matrix_parameters
        {
            uint32_t skin_record_index = 0u;
            uint32_t skin_record_count = 1u;
            uint32_t track_channel_count = 2u;
            uint32_t skin_joint_record_count = 2u;
            uint32_t inverse_bind_matrix_count = 1u;
            uint32_t expected_mesh_node_index = 0u;
            uint32_t root_transform_record_index = 0u;
            uint32_t root_transform_record_count = 1u;
            uint32_t expected_span_record_index = 0u;
            uint32_t reserved[3] = {};
        };
        static_assert(sizeof(skin_matrix_parameters) == 48u);
        skin_matrix_parameters parameters;
        const ComPtr<ID3D11Buffer> parameter_buffer =
            create_constant_buffer(
                device.Get(), &parameters, sizeof(parameters));

        dispatch_compute(
            context.Get(), shader.Get(),
            {
                global_srv.Get(), derivative_srv.Get(),
                skin_srv.Get(), joint_srv.Get(),
                inverse_bind_srv.Get(), local_srv.Get(),
                root_srv.Get(), cursor_srv.Get(), span_srv.Get()
            },
            { output_uav.Get(), output_derivative_uav.Get(), status_uav.Get() },
            parameter_buffer.Get(),
            1u);

        const std::vector<gpu_matrix4x4> skin_matrices =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), output_buffer.Get(), 1u);
        const std::vector<gpu_matrix4x4> skin_matrix_derivatives =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), output_derivative_buffer.Get(), 1u);
        const std::vector<uint32_t> status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(status[0u] == 0u,
            "Valid Mesh / Joint Global Matrices and Skin-specific IBM must not set the GPU Skin Matrix failure bit.");
        require(std::abs(skin_matrices[0u].column3.x - 12.0f) <= 1.0e-6f &&
            std::abs(skin_matrices[0u].column3.w - 1.0f) <= 1.0e-6f &&
            std::abs(skin_matrix_derivatives[0u].column3.x - 2.0f) <= 1.0e-6f &&
            std::abs(skin_matrix_derivatives[0u].column3.w) <= 1.0e-6f,
            "GPU Skin Matrix must compute inverse(Mesh Global) * Joint Global * Skin-specific IBM and its analytic product-rule derivative.");

        root_transform_gpu_record root_motion_record = root_record;
        root_motion_record.translation_end.x = 20.0f;
        const std::vector<root_transform_gpu_record> root_motion_records = {
            root_motion_record
        };
        const ComPtr<ID3D11Buffer> root_motion_buffer =
            create_structured_buffer(
                device.Get(), root_motion_records,
                D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> root_motion_srv =
            create_structured_srv<root_transform_gpu_record>(
                device.Get(), root_motion_buffer.Get(), 1u);
        const skin_matrix_parameters root_motion_parameters = parameters;
        const ComPtr<ID3D11Buffer> root_motion_parameter_buffer =
            create_constant_buffer(
                device.Get(), &root_motion_parameters,
                sizeof(root_motion_parameters));
        dispatch_compute(
            context.Get(), shader.Get(),
            {
                global_srv.Get(), derivative_srv.Get(),
                skin_srv.Get(), joint_srv.Get(),
                inverse_bind_srv.Get(), local_srv.Get(),
                root_motion_srv.Get(), cursor_srv.Get(), span_srv.Get()
            },
            { output_uav.Get(), output_derivative_uav.Get(), status_uav.Get() },
            root_motion_parameter_buffer.Get(),
            1u);
        const std::vector<gpu_matrix4x4> root_motion_derivatives =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), output_derivative_buffer.Get(), 1u);
        const std::vector<uint32_t> root_motion_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), status_buffer.Get(), 1u);
        require(root_motion_status[0u] == 0u &&
            std::abs(root_motion_derivatives[0u].column3.x -
                skin_matrix_derivatives[0u].column3.x - 10.0f) <= 1.0e-6f,
            "GPU Skin Matrix derivative must include the Model Root Transform product-rule term root_dot * inverse_mesh_joint.");

        root_transform_gpu_record root_rotation_record = root_record;
        root_rotation_record.translation_start =
            { 0.0f, 0.0f, 0.0f, 1.0f };
        root_rotation_record.translation_end =
            { 0.0f, 0.0f, 0.0f, 1.0f };
        root_rotation_record.rotation_end = {
            0.0f, 0.0f,
            std::sin(3.14159265358979323846f / 4.0f),
            std::cos(3.14159265358979323846f / 4.0f)
        };
        root_rotation_record.angular_axis_and_angle =
            { 0.0f, 0.0f, 1.0f, 3.14159265358979323846f / 2.0f };
        const std::vector<root_transform_gpu_record> root_rotation_records = {
            root_rotation_record
        };
        const ComPtr<ID3D11Buffer> root_rotation_buffer =
            create_structured_buffer(
                device.Get(), root_rotation_records,
                D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> root_rotation_srv =
            create_structured_srv<root_transform_gpu_record>(
                device.Get(), root_rotation_buffer.Get(), 1u);
        animation_span_cursor_gpu_record half_cursor = cursor_record;
        half_cursor.u_segment_start = 0.5f;
        const std::vector<animation_span_cursor_gpu_record> half_cursors = {
            half_cursor
        };
        const ComPtr<ID3D11Buffer> half_cursor_buffer =
            create_structured_buffer(
                device.Get(), half_cursors, D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> half_cursor_srv =
            create_structured_srv<animation_span_cursor_gpu_record>(
                device.Get(), half_cursor_buffer.Get(), 1u);
        dispatch_compute(
            context.Get(), shader.Get(),
            {
                global_srv.Get(), derivative_srv.Get(),
                skin_srv.Get(), joint_srv.Get(),
                inverse_bind_srv.Get(), local_srv.Get(),
                root_rotation_srv.Get(), half_cursor_srv.Get(), span_srv.Get()
            },
            { output_uav.Get(), output_derivative_uav.Get(), status_uav.Get() },
            parameter_buffer.Get(),
            1u);
        const std::vector<gpu_matrix4x4> root_rotation_matrices =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), output_buffer.Get(), 1u);
        const std::vector<gpu_matrix4x4> root_rotation_derivatives =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), output_derivative_buffer.Get(), 1u);
        const float half_sqrt = std::sqrt(0.5f);
        const float expected_angular_speed =
            3.14159265358979323846f / 2.0f;
        require(std::abs(root_rotation_matrices[0u].column0.x -
                half_sqrt) <= 1.0e-5f &&
            std::abs(root_rotation_matrices[0u].column0.y -
                half_sqrt) <= 1.0e-5f &&
            std::abs(root_rotation_derivatives[0u].column0.x +
                expected_angular_speed * half_sqrt) <= 1.0e-4f &&
            std::abs(root_rotation_derivatives[0u].column0.y -
                expected_angular_speed * half_sqrt) <= 1.0e-4f,
            "GPU Root Transform Packet must evaluate shortest-arc Quaternion Slerp and its analytic World Angular Velocity from the shared Segment Cursor.");

        std::vector<gpu_matrix4x4> singular_global_matrices = global_matrices;
        singular_global_matrices[0u].column0 = {};
        const ComPtr<ID3D11Buffer> singular_global_buffer =
            create_structured_buffer(
                device.Get(), singular_global_matrices,
                D3D11_BIND_SHADER_RESOURCE);
        const ComPtr<ID3D11ShaderResourceView> singular_global_srv =
            create_structured_srv<gpu_matrix4x4>(
                device.Get(), singular_global_buffer.Get(), 2u);
        const ComPtr<ID3D11Buffer> invalid_status_buffer =
            create_structured_buffer(
                device.Get(), clear_status, D3D11_BIND_UNORDERED_ACCESS);
        const ComPtr<ID3D11UnorderedAccessView> invalid_status_uav =
            create_structured_uav<uint32_t>(
                device.Get(), invalid_status_buffer.Get(), 1u);
        dispatch_compute(
            context.Get(), shader.Get(),
            {
                singular_global_srv.Get(), derivative_srv.Get(),
                skin_srv.Get(), joint_srv.Get(),
                inverse_bind_srv.Get(), local_srv.Get(),
                root_srv.Get(), cursor_srv.Get(), span_srv.Get()
            },
            { output_uav.Get(), output_derivative_uav.Get(), invalid_status_uav.Get() },
            parameter_buffer.Get(),
            1u);
        const std::vector<uint32_t> invalid_status =
            readback_structured_buffer<uint32_t>(
                device.Get(), context.Get(), invalid_status_buffer.Get(), 1u);
        const std::vector<gpu_matrix4x4> invalid_skin_matrices =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), output_buffer.Get(), 1u);
        const std::vector<gpu_matrix4x4> invalid_skin_matrix_derivatives =
            readback_structured_buffer<gpu_matrix4x4>(
                device.Get(), context.Get(), output_derivative_buffer.Get(), 1u);
        require(invalid_status[0u] == 8u &&
            std::memcmp(
                &invalid_skin_matrices[0u], &empty_output[0u],
                sizeof(gpu_matrix4x4)) == 0 &&
            std::memcmp(
                &invalid_skin_matrix_derivatives[0u], &empty_output[0u],
                sizeof(gpu_matrix4x4)) == 0,
            "A singular Mesh Node Global Matrix must set the GPU Skin Matrix-stage diagnostic bit and zero the local Joint output slot.");

        std::cout << "[PASS] P09_WARP_ANIMATION_SKIN_MATRICES\n";
    }

    void test_phase12_warp_tick_transaction_commit_gate(
        ID3D11Device* device,
        ID3D11DeviceContext* context,
        ID3D11ComputeShader* begin_shader,
        ID3D11ComputeShader* validate_shader,
        ID3D11ComputeShader* incomplete_shader,
        ID3D11ComputeShader* blocked_shader,
        ID3D11ComputeShader* rejected_shader,
        ID3D11ComputeShader* commit_shader)
    {
        using namespace skin_physics::simulation;

        struct alignas(16) gpu_tick_dispatch_plan
        {
            uint32_t expected_predecessor_sequence_low = 0;
            uint32_t expected_predecessor_sequence_high = 0;
            uint32_t expected_source_generation_low = 0;
            uint32_t expected_source_generation_high = 0;
            uint32_t expected_source_state_set = 0;
            uint32_t destination_state_set = 0;
            uint32_t tick_sequence_low = 0;
            uint32_t tick_sequence_high = 0;
            uint32_t dispatch_id_low = 0;
            uint32_t dispatch_id_high = 0;
            uint32_t reservation_token_low = 0;
            uint32_t reservation_token_high = 0;
            uint32_t reserved[4] = {};
        };

        struct alignas(16) gpu_tick_commit_control
        {
            uint32_t committed_sequence_low = 0;
            uint32_t committed_sequence_high = 0;
            uint32_t state_generation_low = 0;
            uint32_t state_generation_high = 0;
            uint32_t active_state_set = 0;
            uint32_t terminal_state = 0;
            uint32_t terminal_failure = 0;
            uint32_t validation_mask = 0;
            uint32_t required_validation_mask = 0;
            uint32_t dispatch_id_low = 0;
            uint32_t dispatch_id_high = 0;
            uint32_t reservation_token_low = 0;
            uint32_t reservation_token_high = 0;
            uint32_t last_gate_state = 0;
            uint32_t diagnostic = 0;
            uint32_t reserved = 0;
        };

        static_assert(sizeof(gpu_tick_dispatch_plan) == 64,
            "Immutable GPU Tick Dispatch Plan ABI must be 64 bytes.");
        static_assert(sizeof(gpu_tick_commit_control) == 64,
            "GPU Tick Commit Control ABI must be 64 bytes.");

        const auto make_plan = [](
            uint32_t expected_sequence,
            uint32_t expected_generation,
            uint32_t source_state_set,
            uint32_t destination_state_set,
            uint32_t tick_sequence,
            uint32_t dispatch_id,
            uint32_t reservation_token)
        {
            gpu_tick_dispatch_plan plan;
            plan.expected_predecessor_sequence_low = expected_sequence;
            plan.expected_source_generation_low = expected_generation;
            plan.expected_source_state_set = source_state_set;
            plan.destination_state_set = destination_state_set;
            plan.tick_sequence_low = tick_sequence;
            plan.dispatch_id_low = dispatch_id;
            plan.reservation_token_low = reservation_token;
            return plan;
        };

        const auto make_control = []()
        {
            gpu_tick_commit_control control;
            control.committed_sequence_low = 4u;
            control.state_generation_low = 9u;
            control.active_state_set = 0u;
            control.required_validation_mask = 0x1fu;
            return control;
        };

        const auto run_case = [&](
            const gpu_tick_dispatch_plan& plan,
            ID3D11ComputeShader* terminal_shader)
        {
            const gpu_tick_commit_control initial_control = make_control();
            const ComPtr<ID3D11Buffer> plan_buffer =
                create_structured_buffer(
                    device, std::vector<gpu_tick_dispatch_plan>{ plan },
                    D3D11_BIND_SHADER_RESOURCE);
            const ComPtr<ID3D11Buffer> control_buffer =
                create_structured_buffer(
                    device, std::vector<gpu_tick_commit_control>{ initial_control },
                    D3D11_BIND_UNORDERED_ACCESS);
            const ComPtr<ID3D11ShaderResourceView> plan_srv =
                create_structured_srv<gpu_tick_dispatch_plan>(
                    device, plan_buffer.Get(), 1);
            const ComPtr<ID3D11UnorderedAccessView> control_uav =
                create_structured_uav<gpu_tick_commit_control>(
                    device, control_buffer.Get(), 1);

            d3d11_compute_graph graph;
            std::string diagnostic;
            compute_pass_desc begin_pass;
            begin_pass.name = "TickBeginControl";
            begin_pass.shader = begin_shader;
            begin_pass.shader_resources = {
                { 0, plan_srv.Get() }
            };
            begin_pass.unordered_access_views = {
                { 1, control_uav.Get(), 0xffffffffu }
            };
            require(graph.add_pass(begin_pass, diagnostic),
                "Tick Begin Control pass must register: " + diagnostic);

            if (terminal_shader != nullptr)
            {
                compute_pass_desc terminal_pass;
                terminal_pass.name = "TickTerminalValidation";
                terminal_pass.shader = terminal_shader;
                terminal_pass.unordered_access_views = {
                    { 1, control_uav.Get(), 0xffffffffu }
                };
                require(graph.add_pass(terminal_pass, diagnostic),
                    "Tick terminal validation pass must register: " + diagnostic);
            }

            compute_pass_desc commit_pass;
            commit_pass.name = "TickCommitControl";
            commit_pass.shader = commit_shader;
            commit_pass.shader_resources = {
                { 0, plan_srv.Get() }
            };
            commit_pass.unordered_access_views = {
                { 1, control_uav.Get(), 0xffffffffu }
            };
            require(graph.add_pass(commit_pass, diagnostic),
                "Tick Commit Control pass must register: " + diagnostic);

            const compute_graph_result execution = graph.execute(context);
            require(execution.success &&
                execution.executed_pass_count == graph.pass_count(),
                "Tick Begin / Terminal / Commit GPU control passes must execute in sequence.");

            const std::vector<gpu_tick_dispatch_plan> final_plan =
                readback_structured_buffer<gpu_tick_dispatch_plan>(
                    device, context, plan_buffer.Get(), 1);
            const std::vector<gpu_tick_commit_control> final_control =
                readback_structured_buffer<gpu_tick_commit_control>(
                    device, context, control_buffer.Get(), 1);
            return std::make_pair(final_plan[0], final_control[0]);
        };

        const gpu_tick_dispatch_plan successful_plan =
            make_plan(4u, 9u, 0u, 1u, 5u, 1u, 101u);
        const auto successful = run_case(successful_plan, validate_shader);
        require(std::memcmp(&successful.first, &successful_plan,
                sizeof(gpu_tick_dispatch_plan)) == 0 &&
            successful.second.last_gate_state == 1u &&
            successful.second.terminal_state == 6u &&
            successful.second.committed_sequence_low == 5u &&
            successful.second.state_generation_low == 10u &&
            successful.second.active_state_set == 1u,
            "A fully validated Tick must atomically publish the destination State Bank, Sequence and Generation without mutating the immutable Dispatch Plan.");

        const auto stale_predecessor = run_case(
            make_plan(3u, 9u, 0u, 1u, 4u, 2u, 102u),
            validate_shader);
        require(stale_predecessor.second.last_gate_state == 2u &&
            stale_predecessor.second.terminal_state == 3u &&
            stale_predecessor.second.committed_sequence_low == 4u &&
            stale_predecessor.second.state_generation_low == 9u &&
            stale_predecessor.second.active_state_set == 0u,
            "A stale predecessor plan must be BLOCKED before Commit and preserve the active State Bank.");

        const auto incomplete_validation = run_case(
            make_plan(4u, 9u, 0u, 1u, 5u, 3u, 103u),
            incomplete_shader);
        require(incomplete_validation.second.last_gate_state == 1u &&
            incomplete_validation.second.terminal_state == 4u &&
            incomplete_validation.second.committed_sequence_low == 4u &&
            incomplete_validation.second.state_generation_low == 9u &&
            incomplete_validation.second.active_state_set == 0u,
            "Missing GPU validation bits must reject Commit without publishing a partial State Bank.");

        const auto blocked = run_case(
            make_plan(4u, 9u, 0u, 1u, 5u, 4u, 104u),
            blocked_shader);
        require(blocked.second.terminal_state == 3u &&
            blocked.second.committed_sequence_low == 4u &&
            blocked.second.state_generation_low == 9u &&
            blocked.second.active_state_set == 0u,
            "A Capacity / Budget BLOCKED result must retain the old committed State Bank and generation.");

        const auto rejected = run_case(
            make_plan(4u, 9u, 0u, 1u, 5u, 5u, 105u),
            rejected_shader);
        require(rejected.second.terminal_state == 4u &&
            rejected.second.committed_sequence_low == 4u &&
            rejected.second.state_generation_low == 9u &&
            rejected.second.active_state_set == 0u,
            "A REJECTED Tick must preserve the last committed State Bank, Sequence and Generation.");

        std::cout << "[PASS] P12_WARP_TICK_TRANSACTION_COMMIT_GATE\\n";
    }



}

int run_gpu_solver_tests()
{
    test_phase05_warp_colored_cloth_stretch();
    test_phase05_warp_cloth_segment_state_passes();
    test_phase05_warp_cloth_all_constraint_types();
    test_phase05_warp_cloth_jacobi_contribution_gather();
    test_phase06_warp_corotational_fem();
    test_r1_warp_runtime_soft_body_fem_svd_parity();
    test_phase09_warp_animation_trajectory_packet_abi();
    test_phase09_warp_animation_skin_abi();
    test_phase09_warp_animation_trajectory_packet_upload();
    test_phase09_warp_physics_mesh_skinning_targets();
    test_phase09_warp_animation_channel_evaluation();
    test_phase09_warp_animation_blend_local_transforms();
    test_phase09_warp_animation_node_hierarchy();
    test_phase09_warp_animation_skin_matrices();
    {
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        const HRESULT device_result =
            create_warp_device(device.GetAddressOf(), context.GetAddressOf());
        require(SUCCEEDED(device_result), "D3D11 WARP device creation must succeed.");
        const ComPtr<ID3D11ComputeShader> vt_shader = compile_compute_shader(
            device.Get(), test_support::fixture_path("phase07_collision/collision_gpu.cs.hlsl"),
            "vertex_triangle_contact");
        const ComPtr<ID3D11ComputeShader> ee_shader = compile_compute_shader(
            device.Get(), test_support::fixture_path("phase07_collision/collision_gpu.cs.hlsl"),
            "edge_edge_contact");
        const ComPtr<ID3D11ComputeShader> broadphase_shader = compile_compute_shader(
            device.Get(), test_support::fixture_path("phase07_collision/collision_gpu.cs.hlsl"),
            "bvh_broadphase");
        const ComPtr<ID3D11ComputeShader> correction_swept_aabb_shader =
            compile_compute_shader(
                device.Get(),
                test_support::fixture_path("phase07_collision/collision_gpu.cs.hlsl"),
                "correction_swept_aabb");
        const std::filesystem::path correction_candidate_shader_path =
            test_support::fixture_path(
                "phase12_correction/correction_candidate_pipeline.cs.hlsl");
        const ComPtr<ID3D11ComputeShader> correction_candidate_validate_shader =
            compile_compute_shader(device.Get(), correction_candidate_shader_path,
                "correction_candidate_validate_primitives");
        const ComPtr<ID3D11ComputeShader> correction_candidate_count_shader =
            compile_compute_shader(device.Get(), correction_candidate_shader_path,
                "correction_candidate_count");
        const ComPtr<ID3D11ComputeShader> correction_candidate_scan_shader =
            compile_compute_shader(device.Get(), correction_candidate_shader_path,
                "correction_candidate_exclusive_scan");
        const ComPtr<ID3D11ComputeShader> correction_candidate_initialize_shader =
            compile_compute_shader(device.Get(), correction_candidate_shader_path,
                "correction_candidate_initialize");
        const ComPtr<ID3D11ComputeShader> correction_candidate_fill_shader =
            compile_compute_shader(device.Get(), correction_candidate_shader_path,
                "correction_candidate_fill");
        const ComPtr<ID3D11ComputeShader> correction_candidate_sort_shader =
            compile_compute_shader(device.Get(), correction_candidate_shader_path,
                "correction_candidate_bitonic_sort");
        const ComPtr<ID3D11ComputeShader> correction_candidate_deduplicate_shader =
            compile_compute_shader(device.Get(), correction_candidate_shader_path,
                "correction_candidate_deduplicate");
        const std::filesystem::path tick_transaction_shader_path =
            test_support::fixture_path(
                "phase12_correction/tick_transaction_control.cs.hlsl");
        const ComPtr<ID3D11ComputeShader> tick_begin_shader =
            compile_compute_shader(device.Get(), tick_transaction_shader_path,
                "tick_begin_control");
        const ComPtr<ID3D11ComputeShader> tick_validate_shader =
            compile_compute_shader(device.Get(), tick_transaction_shader_path,
                "tick_mark_validated");
        const ComPtr<ID3D11ComputeShader> tick_incomplete_shader =
            compile_compute_shader(device.Get(), tick_transaction_shader_path,
                "tick_mark_incomplete");
        const ComPtr<ID3D11ComputeShader> tick_blocked_shader =
            compile_compute_shader(device.Get(), tick_transaction_shader_path,
                "tick_mark_blocked");
        const ComPtr<ID3D11ComputeShader> tick_rejected_shader =
            compile_compute_shader(device.Get(), tick_transaction_shader_path,
                "tick_mark_rejected");
        const ComPtr<ID3D11ComputeShader> tick_commit_shader =
            compile_compute_shader(device.Get(), tick_transaction_shader_path,
                "tick_commit_control");
        test_phase12_warp_tick_transaction_commit_gate(
            device.Get(), context.Get(),
            tick_begin_shader.Get(),
            tick_validate_shader.Get(),
            tick_incomplete_shader.Get(),
            tick_blocked_shader.Get(),
            tick_rejected_shader.Get(),
            tick_commit_shader.Get());
        const std::filesystem::path indirect_dispatch_shader_path =
            test_support::fixture_path(
                "phase12_correction/indirect_dispatch.cs.hlsl");
        const ComPtr<ID3D11ComputeShader> produce_candidate_count_shader =
            compile_compute_shader(device.Get(), indirect_dispatch_shader_path,
                "produce_candidate_count");
        const ComPtr<ID3D11ComputeShader> build_dispatch_arguments_shader =
            compile_compute_shader(device.Get(), indirect_dispatch_shader_path,
                "build_dispatch_arguments");
        const ComPtr<ID3D11ComputeShader> consume_dispatch_arguments_shader =
            compile_compute_shader(device.Get(), indirect_dispatch_shader_path,
                "consume_dispatch_arguments");
        test_phase12_warp_gpu_generated_indirect_dispatch(
            device.Get(), context.Get(),
            produce_candidate_count_shader.Get(),
            build_dispatch_arguments_shader.Get(),
            consume_dispatch_arguments_shader.Get());
        test_phase07_warp_vertex_triangle_contact(device.Get(), context.Get(), vt_shader.Get());
        test_phase07_warp_edge_edge_contact(device.Get(), context.Get(), ee_shader.Get());
        test_phase07_warp_hierarchical_broadphase(device.Get(), context.Get(), broadphase_shader.Get());
        test_phase12_warp_correction_swept_aabb(
            device.Get(), context.Get(), correction_swept_aabb_shader.Get());
        test_phase12_warp_correction_candidate_pipeline(
            device.Get(), context.Get(),
            correction_candidate_validate_shader.Get(),
            correction_candidate_count_shader.Get(),
            correction_candidate_scan_shader.Get(),
            correction_candidate_initialize_shader.Get(),
            correction_candidate_fill_shader.Get(),
            correction_candidate_sort_shader.Get(),
            correction_candidate_deduplicate_shader.Get());
    }
    test_phase08_warp_point_triangle_ccd();
    test_phase09_warp_animation_target();
    test_phase10_warp_attribute_paint();
    test_phase11_warp_timestamp_profiler_and_dxgi_memory();
    test_phase11_warp_compute_pass_graph();
    std::cout << "GpuSolverTests passed.\n";
    return 0;
}
