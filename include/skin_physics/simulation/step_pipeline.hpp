#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::simulation
{
    enum class simulation_stage : uint8_t
    {
        animation_targets,
        external_forces,
        soft_body,
        cloth,
        broadphase,
        narrowphase,
        ccd,
        position_solve,
        velocity_solve,
        render_feedback,
        count
    };

    struct simulation_state
    {
        std::vector<asset::vec3> positions;
        std::vector<asset::vec3> velocities;
        uint64_t step_index = 0;
    };

    using simulation_stage_callback =
        std::function<bool(simulation_state&, double, std::string&)>;

    enum class pipeline_error : uint8_t
    {
        none,
        invalid_delta_time,
        invalid_initial_state,
        stage_failed,
        invalid_stage_output,
        step_index_overflow
    };

    struct pipeline_result
    {
        pipeline_error error = pipeline_error::none;
        simulation_stage failed_stage = simulation_stage::count;
        uint32_t completed_stages = 0;
        uint64_t committed_step_index = 0;
        std::string diagnostic;
        bool committed = false;
        bool success = false;
    };

    class simulation_step_pipeline
    {
    public:
        bool set_stage_callback(
            simulation_stage stage,
            simulation_stage_callback callback);
        void clear_stage_callback(simulation_stage stage);
        void clear_all_callbacks();

        pipeline_result execute(simulation_state& state, double delta_time) const;

        static const char* stage_name(simulation_stage stage);

    private:
        static constexpr std::size_t stage_count =
            static_cast<std::size_t>(simulation_stage::count);
        std::array<simulation_stage_callback, stage_count> _callbacks;
    };
}
