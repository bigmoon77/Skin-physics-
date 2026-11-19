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
        const float contact_thickness = 0.02f;
        const std::vector<vec3> positions_before = {
            { 0.25f, 0.25f, 1.0f },
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f }
        };
        const std::vector<vec3> positions_after = {
            { 0.25f, 0.25f, -1.0f },
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f }
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

        ID3D11ShaderResourceView* resources[] = {
            before_view.Get(), after_view.Get(), primitive_view.Get()
        };
        ID3D11UnorderedAccessView* unordered_views[] = {
            bounds_view.Get(), status_view.Get()
        };
        context->CSSetShader(shader, nullptr, 0);
        context->CSSetShaderResources(6, 3, resources);
        context->CSSetUnorderedAccessViews(4, 2, unordered_views, nullptr);
        ID3D11Buffer* constant_buffer = constants.Get();
        context->CSSetConstantBuffers(1, 1, &constant_buffer);
        context->Dispatch(1, 1, 1);

        ID3D11ShaderResourceView* null_resources[] = {
            nullptr, nullptr, nullptr
        };
        ID3D11UnorderedAccessView* null_unordered[] = { nullptr, nullptr };
        ID3D11Buffer* null_constants[] = { nullptr };
        context->CSSetShaderResources(6, 3, null_resources);
        context->CSSetUnorderedAccessViews(4, 2, null_unordered, nullptr);
        context->CSSetConstantBuffers(1, 1, null_constants);
        context->CSSetShader(nullptr, nullptr, 0);

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
                    actual_maximum[axis] < expected_maximum[axis])
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
        ID3D11ShaderResourceView* invalid_resources[] = {
            before_view.Get(), after_view.Get(), invalid_primitive_view.Get()
        };
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
        ID3D11UnorderedAccessView* invalid_unordered[] = {
            invalid_bounds_view.Get(), invalid_status_view.Get()
        };
        context->CSSetShader(shader, nullptr, 0);
        context->CSSetShaderResources(6, 3, invalid_resources);
        context->CSSetUnorderedAccessViews(4, 2, invalid_unordered, nullptr);
        context->CSSetConstantBuffers(1, 1, &constant_buffer);
        context->Dispatch(1, 1, 1);
        context->CSSetShaderResources(6, 3, null_resources);
        context->CSSetUnorderedAccessViews(4, 2, null_unordered, nullptr);
        context->CSSetConstantBuffers(1, 1, null_constants);
        context->CSSetShader(nullptr, nullptr, 0);

        const std::vector<uint32_t> invalid_status =
            readback_structured_buffer<uint32_t>(
                device, context, invalid_status_buffer.Get(), 2);
        require(invalid_status[0] == 0u && invalid_status[1] == 2u,
            "WARP Correction Swept AABB must fail closed on an out-of-range primitive vertex index.");

        std::cout << "[PASS] P12_WARP_CORRECTION_SWEPT_AABB\n";
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
        test_phase07_warp_vertex_triangle_contact(device.Get(), context.Get(), vt_shader.Get());
        test_phase07_warp_edge_edge_contact(device.Get(), context.Get(), ee_shader.Get());
        test_phase07_warp_hierarchical_broadphase(device.Get(), context.Get(), broadphase_shader.Get());
    }
    test_phase08_warp_point_triangle_ccd();
    test_phase09_warp_animation_target();
    test_phase10_warp_attribute_paint();
    test_phase11_warp_timestamp_profiler_and_dxgi_memory();
    test_phase11_warp_compute_pass_graph();
    std::cout << "GpuSolverTests passed.\n";
    return 0;
}
