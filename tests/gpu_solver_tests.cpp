#include "skin_physics/physics/cloth_solver.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"
#include "skin_physics/physics/soft_body_fem.hpp"
#include "skin_physics/physics/collision.hpp"
#include "skin_physics/physics/animation_track.hpp"
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

    static_assert(sizeof(gpu_tet_input) == 160, "Tet input layout must match HLSL.");
    static_assert(sizeof(gpu_tet_output) == 224, "Tet output layout must match HLSL.");

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
            const ComPtr<ID3D11Buffer> flags_buffer =
                create_structured_buffer(
                    device, initial_flags, D3D11_BIND_UNORDERED_ACCESS);
            const ComPtr<ID3D11Buffer> prefix_buffer =
                create_structured_buffer(
                    device, initial_prefix, D3D11_BIND_UNORDERED_ACCESS);
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
        ID3D11ComputeShader* build_arguments_shader,
        ID3D11ComputeShader* consume_arguments_shader)
    {
        using namespace skin_physics::simulation;

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
        compute_pass_desc build_pass;
        build_pass.name = "BuildDispatchArguments";
        build_pass.shader = build_arguments_shader;
        build_pass.unordered_access_views = {
            { 0, argument_uav.Get(), 0xffffffffu }
        };

        compute_pass_desc consume_pass;
        consume_pass.name = "ConsumeDispatchArguments";
        consume_pass.shader = consume_arguments_shader;
        consume_pass.unordered_access_views = {
            { 0, output_uav.Get(), 0xffffffffu }
        };
        consume_pass.indirect_arguments = argument_buffer.Get();

        compute_pass_desc invalid_alignment = consume_pass;
        invalid_alignment.name = "InvalidIndirectAlignment";
        invalid_alignment.indirect_argument_offset = 2u;

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
        require(!graph.add_pass(invalid_argument_flags, diagnostic) &&
            diagnostic.find("DRAWINDIRECT_ARGS") != std::string::npos,
            "Compute graph must reject a buffer without the indirect-argument misc flag.");
        require(!graph.add_pass(invalid_uav_alias, diagnostic) &&
            diagnostic.find("unbound from UAV") != std::string::npos,
            "Compute graph must reject an indirect-argument buffer also bound as UAV in the consumer pass.");

        require(graph.add_pass(build_pass, diagnostic),
            "Compute graph must accept a GPU argument producer pass: " + diagnostic);
        require(graph.add_pass(consume_pass, diagnostic),
            "Compute graph must accept an indirect consumer pass: " + diagnostic);
        const compute_graph_result execution = graph.execute(context);
        require(execution.success && execution.executed_pass_count == 2,
            "Compute graph must execute the indirect consumer after its producer pass.");

        const std::vector<uint32_t> actual_output =
            readback_structured_buffer<uint32_t>(
                device, context, output_buffer.Get(), initial_output.size());
        require(actual_output == std::vector<uint32_t>{ 0x5a00u, 0x5a01u, 0x5a02u },
            "GPU-generated group counts must drive DispatchIndirect without CPU readback.");

        std::cout << "[PASS] P12_WARP_GPU_GENERATED_INDIRECT_DISPATCH\n";
    }


}

int run_gpu_solver_tests()
{
    test_phase05_warp_cloth_all_constraint_types();
    test_phase06_warp_corotational_fem();
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
        const std::filesystem::path indirect_dispatch_shader_path =
            test_support::fixture_path(
                "phase12_correction/indirect_dispatch.cs.hlsl");
        const ComPtr<ID3D11ComputeShader> build_dispatch_arguments_shader =
            compile_compute_shader(device.Get(), indirect_dispatch_shader_path,
                "build_dispatch_arguments");
        const ComPtr<ID3D11ComputeShader> consume_dispatch_arguments_shader =
            compile_compute_shader(device.Get(), indirect_dispatch_shader_path,
                "consume_dispatch_arguments");
        test_phase12_warp_gpu_generated_indirect_dispatch(
            device.Get(), context.Get(),
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
