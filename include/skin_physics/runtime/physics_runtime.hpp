#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <d3d11.h>

#include "skin_physics/asset/types.hpp"
#include "skin_physics/physics/physics_mesh.hpp"
#include "skin_physics/physics/render_binding.hpp"

namespace skin_physics::runtime
{
    enum class status_code : uint32_t
    {
        ok,
        not_ready,
        invalid_argument,
        invalid_state,
        busy,
        submission_thread_violation,
        blocked_by_input,
        blocked_by_capacity,
        blocked_by_ccd,
        blocked_by_predecessor,
        render_binding_busy,
        stale_handle,
        unsupported_feature,
        asset_invalid,
        capacity_exceeded,
        out_of_memory,
        device_removed,
        gpu_execution_failed,
        step_rejected,
        work_budget_exceeded,
        work_budget_unsatisfiable,
        contact_geometry_unresolved,
        contact_manifold_reduction_failure,
        contact_impulse_unresolved,
        animation_time_precision_unresolved,
        no_force_target
    };

    struct status
    {
        status_code code = status_code::ok;
        int32_t native_code = 0;
        std::string diagnostic;

        bool success() const;
    };

    enum class playback_state : uint8_t
    {
        stopped,
        playing,
        paused
    };

    enum class animation_discontinuity_policy : uint8_t
    {
        preserve_velocity,
        reset_velocity,
        blend_velocity
    };

    enum class work_budget_block_origin : uint8_t
    {
        none,
        cpu_preflight,
        gpu_dynamic_toi
    };

    enum class work_budget_resolution : uint8_t
    {
        none,
        retryable,
        hard_limit_reached
    };

    enum class tick_state : uint8_t
    {
        idle,
        pending,
        in_flight,
        blocked,
        rejected,
        failed,
        committed
    };

    struct world_handle
    {
        uint32_t index = asset::invalid_index;
        uint32_t generation = 0;
    };

    struct model_handle
    {
        uint32_t index = asset::invalid_index;
        uint32_t generation = 0;
    };

    struct collider_handle
    {
        uint32_t index = asset::invalid_index;
        uint32_t generation = 0;
    };

    struct transform
    {
        asset::vec3 translation;
        asset::vec4 rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
    };

    struct runtime_desc
    {
        std::string shader_package_root;
        uint64_t maximum_gpu_resource_bytes = 0;
        uint32_t maximum_world_substeps_hard_limit = 1024;
        uint32_t maximum_integration_segments_per_tick_hard_limit = 4096;
        uint64_t host_reserved_gpu_bytes = 0;
        uint32_t query_ring_capacity = 64;
        bool enable_debug_layer_validation = false;
    };

    struct world_execution_budget
    {
        uint32_t maximum_substeps = 128;
        uint32_t maximum_integration_segments_per_tick = 512;
    };

    struct world_desc
    {
        double fixed_delta_time = 1.0 / 60.0;
        double time_scale = 1.0;
        asset::vec3 gravity = { 0.0f, -9.81f, 0.0f };
        uint32_t maximum_in_flight_ticks = 2;
        uint32_t minimum_substeps = 1;
        uint32_t maximum_substeps = 128;
        uint32_t maximum_integration_segments_per_tick = 512;
        double world_max_substep_delta_time = 0.0;
        double contact_max_substep_delta_time = 1.0 / 120.0;
        uint32_t contact_impulse_iterations = 8;
        float contact_impulse_relaxation = 0.5f;
        float contact_impulse_residual_tolerance = 1.0e-4f;
        bool enable_cloth_temporal_quality_limit = true;
        float cloth_temporal_quality_factor = 0.2f;
        uint32_t maximum_steps_per_advance = 8;
        uint64_t memory_budget_bytes = 0;
    };

    struct model_mesh_physics_asset
    {
        uint32_t mesh_index = asset::invalid_index;
        // Resolved by Runtime from the unique Asset Node that instances this
        // Soft Body Mesh with a Skin. One Physics State Bank currently maps to
        // one Skinned Mesh Node; multiple instances require separate assets.
        uint32_t skin_node_index = asset::invalid_index;
        uint32_t skin_index = asset::invalid_index;
        physics::physics_mesh physics_mesh;
        std::vector<physics::render_vertex_binding> render_bindings;
        uint64_t cache_identity = 0;
    };

    struct model_desc
    {
        asset::model source_model;
        std::vector<model_mesh_physics_asset> mesh_physics_assets;
        transform world_transform;
    };

    struct collision_filter
    {
        uint32_t layer = 1u;
        uint32_t mask = 0xffffffffu;
    };

    struct collider_desc
    {
        asset::mesh collision_mesh;
        transform initial_transform;
        float thickness = 0.0f;
        float friction = 0.5f;
        float restitution = 0.0f;
        collision_filter filter;
    };

    struct target_filter
    {
        // Invalid Model + invalid Mesh selects every supported Soft Body Mesh
        // in the World. A valid Model + invalid Mesh selects every supported
        // Soft Body Mesh in that Model. A valid Model + Mesh selects one Mesh.
        model_handle model;
        uint32_t mesh_index = asset::invalid_index;
    };

    struct force_desc
    {
        uint64_t event_id = 0;
        target_filter target;
        asset::vec3 position;
        asset::vec3 direction = { 0.0f, 1.0f, 0.0f }; // Normalized by enqueue_force.
        float magnitude = 0.0f;
        float radius = 1.0f;
        double duration = 0.0;
        double start_time = 0.0;
    };

    struct impulse_desc
    {
        uint64_t event_id = 0;
        target_filter target;
        asset::vec3 position;
        asset::vec3 direction = { 0.0f, 1.0f, 0.0f }; // Normalized by enqueue_impulse.
        float impulse_magnitude = 0.0f;
        float radius = 1.0f;
        double event_time = 0.0;
    };

    struct animation_track_desc
    {
        uint32_t clip_index = 0;
        double time = 0.0;
        double speed = 1.0;
        bool loop = false;
    };

    struct animation_playback_desc
    {
        animation_track_desc track;
        animation_discontinuity_policy discontinuity_policy =
            animation_discontinuity_policy::preserve_velocity;
        float w_event = 0.0f;
    };

    struct animation_blend_desc
    {
        animation_track_desc source_track;
        animation_track_desc target_track;
        double blend_start_time = 0.0;
        double blend_end_time = 1.0;
        float weight_start = 0.0f;
        float weight_end = 1.0f;
        float tangent_start = 0.0f;
        float tangent_end = 0.0f;
        animation_discontinuity_policy discontinuity_policy =
            animation_discontinuity_policy::preserve_velocity;
        float w_event = 0.0f;
    };

    struct mesh_physics_parameters
    {
        float density = 1000.0f;
        float young_modulus = 100000.0f;
        float poisson_ratio = 0.3f;
        float damping = 0.01f;
        // Kelvin-Voigt Animation Tracking coefficients. Zero stiffness disables
        // the spring / frequency-limit term; both zero disable the full force.
        float tracking_stiffness = 0.0f;
        float tracking_damping = 0.0f;
        std::array<float,
            static_cast<std::size_t>(asset::physics_attribute_channel::count)>
            attribute_multipliers = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
    };

    enum class attribute_edit_operation : uint8_t
    {
        paint,
        smooth,
        average,
        erase
    };

    struct attribute_edit_desc
    {
        attribute_edit_operation operation = attribute_edit_operation::paint;
        asset::physics_attribute_channel channel = asset::physics_attribute_channel::stretch;
        std::vector<uint32_t> vertex_indices;
        float target = 1.0f;
        float strength = 1.0f;
    };

    struct advance_report
    {
        uint32_t submitted_tick_count = 0;
        uint32_t pending_tick_count = 0;
        uint32_t in_flight_tick_count = 0;
        uint32_t cpu_preflight_blocked_tick_count = 0;
        uint64_t first_cpu_preflight_blocked_tick_sequence = 0;
        bool render_binding_backpressure = false;
        bool capacity_blocked = false;
        double accumulator_seconds = 0.0;
    };

    struct world_status
    {
        playback_state playback = playback_state::stopped;
        tick_state state = tick_state::idle;
        status_code last_tick_status = status_code::ok;
        work_budget_block_origin budget_block_origin = work_budget_block_origin::none;
        work_budget_resolution budget_resolution = work_budget_resolution::none;
        uint8_t work_budget_block_flags = 0;
        uint64_t execution_budget_revision = 1;
        uint64_t submitted_sequence = 0;
        uint64_t gpu_committed_sequence = 0;
        uint64_t cpu_observed_sequence = 0;
        uint64_t head_reserved_tick_sequence = 0;
        uint64_t head_input_snapshot_version = 0;
        uint32_t planned_substeps = 0;
        uint32_t planned_integration_segments = 0;
        uint32_t executed_integration_segments = 0;
        uint32_t minimum_required_integration_segments = 0;
        double requested_simulation_time = 0.0;
        double cancelled_requested_time = 0.0;
        double effective_requested_time = 0.0;
        double reserved_tick_time = 0.0;
        double gpu_committed_simulation_time = 0.0;
        double cpu_observed_completed_time = 0.0;
        double simulation_backlog = 0.0;
        uint32_t pending_tick_count = 0;
        uint32_t in_flight_tick_count = 0;
        std::string diagnostic;
    };

    struct render_binding
    {
        ID3D11ShaderResourceView* position_srv = nullptr;
        ID3D11ShaderResourceView* normal_srv = nullptr;
        ID3D11ShaderResourceView* tangent_srv = nullptr;
        DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
        uint32_t stride = 0;
        uint32_t vertex_count = 0;
        uint32_t index_count = 0;
        uint64_t resource_generation = 0;
        uint64_t device_generation = 0;
        uint64_t committed_tick_sequence = 0;
        uint32_t render_slot_id = asset::invalid_index;
        uint32_t render_slot_generation = 0;
        uint64_t lease_generation = 0;
    };

    class physics_runtime
    {
    public:
        physics_runtime();
        ~physics_runtime();

        physics_runtime(const physics_runtime&) = delete;
        physics_runtime& operator=(const physics_runtime&) = delete;
        physics_runtime(physics_runtime&&) = delete;
        physics_runtime& operator=(physics_runtime&&) = delete;

        status initialize(
            ID3D11Device* device,
            ID3D11DeviceContext* immediate_context,
            const runtime_desc& desc);

        status create_world(const world_desc& desc, world_handle* out_world);
        status add_model(world_handle world, const model_desc& desc, model_handle* out_model);
        status create_static_collider(
            world_handle world, const collider_desc& desc, collider_handle* out_collider);
        status create_kinematic_collider(
            world_handle world, const collider_desc& desc, collider_handle* out_collider);
        status destroy_model(world_handle world, model_handle model);
        status destroy_collider(world_handle world, collider_handle collider);

        status set_playback_state(world_handle world, playback_state state);
        status set_fixed_delta_time(world_handle world, double fixed_delta_time);
        status set_time_scale(world_handle world, double time_scale);
        status set_world_execution_budget(
            world_handle world, const world_execution_budget& budget);
        status set_gravity(world_handle world, const asset::vec3& gravity);

        status enqueue_force(world_handle world, const force_desc& force);
        status enqueue_impulse(world_handle world, const impulse_desc& impulse);
        status set_animation_playback(
            world_handle world, model_handle model, const animation_playback_desc& desc);
        status set_animation_blend(
            world_handle world, model_handle model, const animation_blend_desc& desc);
        status set_model_transform_sample(
            world_handle world,
            model_handle model,
            double sample_time,
            const transform& world_transform,
            bool is_teleport);
        status set_mesh_physics_parameters(
            world_handle world,
            model_handle model,
            uint32_t mesh_index,
            const mesh_physics_parameters& parameters);
        status apply_attribute_edit(
            world_handle world,
            model_handle model,
            uint32_t mesh_index,
            const attribute_edit_desc& edit);
        status set_kinematic_pose_sample(
            world_handle world,
            collider_handle collider,
            double sample_time,
            const transform& pose);

        status advance(
            world_handle world, double frame_delta_time, advance_report* out_report);
        status poll(world_handle world, world_status* out_status);
        status single_step(world_handle world, advance_report* out_report);

        // Exposes the latest CPU-observed committed Candidate and opens a World Render Lease.
        // Repeated Mesh requests while leased return the same pinned Candidate Slot.
        status get_render_binding(
            world_handle world,
            model_handle model,
            uint32_t mesh_index,
            render_binding* out_binding);

        // Release only after all returned SRVs are unbound from every shader stage and
        // every command list that references them has executed on the Immediate Context.
        status release_render_bindings(world_handle world, uint64_t lease_generation);

        status reset_world(world_handle world);
        status destroy_world(world_handle world);
        status drain();
        status_code shutdown() noexcept;

        bool initialized() const;
        const runtime_desc* description() const;

    private:
        struct implementation;
        std::unique_ptr<implementation> _implementation;
    };
}
