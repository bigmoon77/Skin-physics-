#include "skin_physics/simulation/step_pipeline.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <utility>

namespace skin_physics::simulation
{
    namespace
    {
        bool valid_stage(simulation_stage stage)
        {
            return static_cast<std::size_t>(stage) <
                static_cast<std::size_t>(simulation_stage::count);
        }

        bool finite_vector(const asset::vec3& value)
        {
            return std::isfinite(value.x) &&
                std::isfinite(value.y) &&
                std::isfinite(value.z);
        }

        bool valid_state(const simulation_state& state)
        {
            if (state.positions.size() != state.velocities.size())
            {
                return false;
            }
            for (const asset::vec3& value : state.positions)
            {
                if (!finite_vector(value)) return false;
            }
            for (const asset::vec3& value : state.velocities)
            {
                if (!finite_vector(value)) return false;
            }
            return true;
        }

        pipeline_result make_failure(
            const simulation_state& original,
            pipeline_error error,
            simulation_stage stage,
            uint32_t completed_stages,
            const std::string& diagnostic)
        {
            pipeline_result result;
            result.error = error;
            result.failed_stage = stage;
            result.completed_stages = completed_stages;
            result.committed_step_index = original.step_index;
            result.diagnostic = diagnostic;
            result.committed = false;
            result.success = false;
            return result;
        }
    }

    bool simulation_step_pipeline::set_stage_callback(
        simulation_stage stage,
        simulation_stage_callback callback)
    {
        if (!valid_stage(stage))
        {
            return false;
        }
        _callbacks[static_cast<std::size_t>(stage)] = std::move(callback);
        return true;
    }

    void simulation_step_pipeline::clear_stage_callback(simulation_stage stage)
    {
        if (!valid_stage(stage)) return;
        _callbacks[static_cast<std::size_t>(stage)] = {};
    }

    void simulation_step_pipeline::clear_all_callbacks()
    {
        for (simulation_stage_callback& callback : _callbacks)
        {
            callback = {};
        }
    }

    pipeline_result simulation_step_pipeline::execute(
        simulation_state& state,
        double delta_time) const
    {
        if (!std::isfinite(delta_time) || delta_time <= 0.0)
        {
            return make_failure(state, pipeline_error::invalid_delta_time,
                simulation_stage::count, 0,
                "Simulation pipeline delta time must be finite and positive.");
        }
        if (!valid_state(state))
        {
            return make_failure(state, pipeline_error::invalid_initial_state,
                simulation_stage::count, 0,
                "Simulation state positions and velocities must have matching counts and finite values.");
        }
        if (state.step_index == std::numeric_limits<uint64_t>::max())
        {
            return make_failure(state, pipeline_error::step_index_overflow,
                simulation_stage::count, 0,
                "Simulation step index cannot be incremented without overflow.");
        }

        simulation_state working = state;
        uint32_t completed_stages = 0;
        for (std::size_t stage_index = 0; stage_index < stage_count; ++stage_index)
        {
            const simulation_stage stage = static_cast<simulation_stage>(stage_index);
            const simulation_stage_callback callback = _callbacks[stage_index];
            if (!callback)
            {
                continue;
            }

            const std::size_t expected_vertex_count = working.positions.size();
            std::string stage_error;
            bool accepted = false;
            try
            {
                accepted = callback(working, delta_time, stage_error);
            }
            catch (const std::exception& exception)
            {
                stage_error = exception.what();
            }
            catch (...)
            {
                stage_error = "Stage callback threw an unknown exception.";
            }

            if (!accepted)
            {
                if (stage_error.empty())
                {
                    stage_error = "Stage callback rejected the simulation step.";
                }
                return make_failure(state, pipeline_error::stage_failed,
                    stage, completed_stages,
                    std::string(stage_name(stage)) + ": " + stage_error);
            }

            if (working.positions.size() != expected_vertex_count ||
                working.velocities.size() != expected_vertex_count ||
                !valid_state(working))
            {
                return make_failure(state, pipeline_error::invalid_stage_output,
                    stage, completed_stages,
                    std::string(stage_name(stage)) +
                    ": stage output changed the vertex count or produced a non-finite state.");
            }
            ++completed_stages;
        }

        ++working.step_index;
        state = std::move(working);

        pipeline_result result;
        result.completed_stages = completed_stages;
        result.committed_step_index = state.step_index;
        result.committed = true;
        result.success = true;
        return result;
    }

    const char* simulation_step_pipeline::stage_name(simulation_stage stage)
    {
        switch (stage)
        {
        case simulation_stage::animation_targets: return "AnimationTargets";
        case simulation_stage::external_forces: return "ExternalForces";
        case simulation_stage::soft_body: return "SoftBody";
        case simulation_stage::cloth: return "Cloth";
        case simulation_stage::broadphase: return "Broadphase";
        case simulation_stage::narrowphase: return "Narrowphase";
        case simulation_stage::ccd: return "CCD";
        case simulation_stage::position_solve: return "PositionSolve";
        case simulation_stage::velocity_solve: return "VelocitySolve";
        case simulation_stage::render_feedback: return "RenderFeedback";
        default: return "Unknown";
        }
    }
}
