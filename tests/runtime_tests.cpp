#include "skin_physics/runtime/physics_runtime.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
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

        skin_physics::runtime::world_status before_retry;
        require(runtime.poll(world, &before_retry).success() &&
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
        require(runtime.shutdown() == status_code::ok,
            "Runtime must shut down after the CPU Preflight recovery regression.");
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
    std::cout << "RuntimeTests passed.\n";
    return 0;
}
