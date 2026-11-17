#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace skin_physics::simulation
{
    enum class playback_state : uint8_t
    {
        stopped,
        playing,
        paused
    };

    enum class fixed_step_error : uint8_t
    {
        none,
        invalid_configuration,
        invalid_frame_delta,
        callback_failed,
        step_not_allowed
    };

    struct fixed_step_settings
    {
        double fixed_delta_time = 1.0 / 60.0;
        double time_scale = 1.0;
        uint32_t maximum_steps_per_advance = 8;
    };

    struct fixed_step_result
    {
        uint32_t completed_steps = 0;
        double simulation_time = 0.0;
        double accumulator = 0.0;
        bool has_backlog = false;
        fixed_step_error error = fixed_step_error::none;
        std::string message;
        bool success = true;
    };

    using fixed_step_callback = std::function<bool(double)>;

    class fixed_step_controller
    {
    public:
        explicit fixed_step_controller(const fixed_step_settings& settings = {});

        bool configure(const fixed_step_settings& settings);
        void play();
        void pause();
        void stop();
        void reset();

        bool set_time_scale(double value);
        bool set_fixed_delta_time(double value);

        fixed_step_result advance(double frame_delta_time, const fixed_step_callback& callback);
        fixed_step_result single_step(const fixed_step_callback& callback);

        playback_state state() const;
        double simulation_time() const;
        double accumulator() const;
        const fixed_step_settings& settings() const;

    private:
        fixed_step_settings _settings;
        playback_state _state = playback_state::stopped;
        double _simulation_time = 0.0;
        double _accumulator = 0.0;

        bool valid_settings(const fixed_step_settings& settings) const;
        fixed_step_result make_result(uint32_t completed_steps, fixed_step_error error,
            const std::string& message) const;
        bool execute_one_step(const fixed_step_callback& callback, std::string& error_message);
    };
}
