#include "skin_physics/physics/cloth_solver.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"
#include "skin_physics/physics/soft_body_fem.hpp"
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

}

int run_gpu_solver_tests()
{
    test_phase05_warp_cloth_all_constraint_types();
    test_phase06_warp_corotational_fem();
    std::cout << "GpuSolverTests passed.\n";
    return 0;
}
