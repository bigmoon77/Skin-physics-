#include "skin_physics/runtime/physics_runtime.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"

#include <cmath>
#include <cstring>
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
        require(runtime.initialize(device.Get(), context.Get(), runtime_desc).success(),
            "Runtime must accept a valid Host-owned WARP Device and Immediate Context.");
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
            std::abs(before_retry.reserved_tick_time - 0.02) <= 1.0e-12,
            "CPU Preflight must report required work while preserving the exact head Reservation.");

        require(runtime.set_gravity(
                world, { 0.0f, -1.0f, 0.0f }).code == status_code::busy,
            "World input mutation must be rejected while a Tick Reservation holds its immutable input snapshot.");

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
        skin_physics::runtime::model_mesh_physics_asset mesh_asset;
        mesh_asset.mesh_index = 0u;
        mesh_asset.physics_mesh = built_mesh.value;
        model_desc.mesh_physics_assets.push_back(mesh_asset);
        mesh_asset.mesh_index = 1u;
        model_desc.mesh_physics_assets.push_back(std::move(mesh_asset));

        skin_physics::runtime::world_desc world_desc;
        world_desc.fixed_delta_time = 0.01;
        world_desc.contact_max_substep_delta_time = 0.004;
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

        skin_physics::runtime::attribute_edit_desc paint_edit;
        paint_edit.channel =
            skin_physics::asset::physics_attribute_channel::shape_recovery;
        paint_edit.vertex_indices = { 0u };
        paint_edit.target = 0.25f;
        paint_edit.strength = 1.0f;
        require(runtime.apply_attribute_edit(world, model, 0u, paint_edit).success(),
            "An idle Soft Body Paint edit must update Render Mesh values and refresh the Physics Mesh harmonic transfer.");

        require(runtime.set_playback_state(
                world, skin_physics::runtime::playback_state::playing).success(),
            "Soft Body Runtime World must enter Playing.");

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
                "Render Feedback must invert the non-zero Model Root Translation and preserve the expected small gravity displacement in model-local space.");

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
        require(result.gpu_committed_sequence == 0u &&
            result.cpu_observed_sequence == 0u &&
            result.in_flight_tick_count == 0u &&
            result.head_reserved_tick_sequence == 1u &&
            result.pending_tick_count == 2u &&
            std::abs(result.reserved_tick_time - 0.02) <= 1.0e-12,
            "Both non-committed Reservations must be retained in ascending Sequence order with full reserved time and no published State Bank.");

        skin_physics::runtime::world_status repeated;
        require(runtime.poll(world, &repeated).success() &&
            repeated.state == skin_physics::runtime::tick_state::failed &&
            repeated.last_tick_status == status_code::gpu_execution_failed &&
            repeated.head_reserved_tick_sequence == 1u &&
            repeated.pending_tick_count == 2u,
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
        require(runtime.set_mesh_physics_parameters(
                world, model, 0u, parameters).success(),
            "High-stiffness CFL material must be accepted before Tick Reservation.");

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
            "Reset must release the pending CFL-blocked Reservation.");
        require(runtime.destroy_model(world, model).success(),
            "Reset CFL Model must release its GPU resources.");
        require(runtime.destroy_world(world).success(),
            "Reset CFL World must be destroyable.");
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the CFL preflight regression.");
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
    std::cout << "[PASS] R1_WARP_SOFT_BODY_RENDER_CANDIDATE_LEASE\n";
    test_runtime_multiple_in_flight_non_commit_preserves_head_sequence();
    std::cout << "[PASS] R0_WARP_PREDECESSOR_FAILURE_ORDER\n";
    test_runtime_soft_body_cfl_preflight_and_budget_recovery();
    std::cout << "[PASS] R1_WARP_SOFT_BODY_CFL_PREFLIGHT\n";
    std::cout << "RuntimeTests passed.\n";
    return 0;
}
