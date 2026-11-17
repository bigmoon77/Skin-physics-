#include "skin_physics/simulation/fixed_step.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace skin_physics::simulation
{
    fixed_step_controller::fixed_step_controller(const fixed_step_settings& settings)
    {
        if (!configure(settings))
        {
            _settings = fixed_step_settings{};
        }
    }

    bool fixed_step_controller::valid_settings(const fixed_step_settings& settings) const
    {
        return std::isfinite(settings.fixed_delta_time) &&
            settings.fixed_delta_time > 0.0 &&
            std::isfinite(settings.time_scale) &&
            settings.time_scale >= 0.0 &&
            settings.maximum_steps_per_advance > 0;
    }

    bool fixed_step_controller::configure(const fixed_step_settings& settings)
    {
        if (!valid_settings(settings))
        {
            return false;
        }
        _settings = settings;
        _accumulator = 0.0;
        return true;
    }

    void fixed_step_controller::play()
    {
        _state = playback_state::playing;
    }

    void fixed_step_controller::pause()
    {
        if (_state == playback_state::playing)
        {
            _state = playback_state::paused;
        }
    }

    void fixed_step_controller::stop()
    {
        _state = playback_state::stopped;
        _accumulator = 0.0;
    }

    void fixed_step_controller::reset()
    {
        _state = playback_state::stopped;
        _simulation_time = 0.0;
        _accumulator = 0.0;
    }

    bool fixed_step_controller::set_time_scale(double value)
    {
        if (!std::isfinite(value) || value < 0.0)
        {
            return false;
        }
        _settings.time_scale = value;
        return true;
    }

    bool fixed_step_controller::set_fixed_delta_time(double value)
    {
        if (!std::isfinite(value) || value <= 0.0)
        {
            return false;
        }
        _settings.fixed_delta_time = value;
        _accumulator = 0.0;
        return true;
    }

    bool fixed_step_controller::execute_one_step(
        const fixed_step_callback& callback,
        std::string& error_message)
    {
        if (!callback)
        {
            error_message = "Fixed-step callback is empty.";
            return false;
        }
        if (!callback(_settings.fixed_delta_time))
        {
            error_message = "Fixed-step callback rejected the simulation step; time was not committed.";
            return false;
        }
        _simulation_time += _settings.fixed_delta_time;
        return true;
    }

    fixed_step_result fixed_step_controller::make_result(
        uint32_t completed_steps,
        fixed_step_error error,
        const std::string& message) const
    {
        fixed_step_result result;
        result.completed_steps = completed_steps;
        result.simulation_time = _simulation_time;
        result.accumulator = _accumulator;
        result.has_backlog = _accumulator + 1.0e-12 >= _settings.fixed_delta_time;
        result.error = error;
        result.message = message;
        result.success = error == fixed_step_error::none;
        return result;
    }

    fixed_step_result fixed_step_controller::advance(
        double frame_delta_time,
        const fixed_step_callback& callback)
    {
        if (!std::isfinite(frame_delta_time) || frame_delta_time < 0.0)
        {
            return make_result(0, fixed_step_error::invalid_frame_delta,
                "Frame delta time must be finite and non-negative.");
        }
        if (!valid_settings(_settings))
        {
            return make_result(0, fixed_step_error::invalid_configuration,
                "Fixed-step configuration is invalid.");
        }

        if (_state != playback_state::playing)
        {
            return make_result(0, fixed_step_error::none, {});
        }

        _accumulator += frame_delta_time * _settings.time_scale;
        uint32_t completed_steps = 0;
        while (_accumulator + 1.0e-12 >= _settings.fixed_delta_time &&
            completed_steps < _settings.maximum_steps_per_advance)
        {
            std::string error_message;
            if (!execute_one_step(callback, error_message))
            {
                return make_result(completed_steps, fixed_step_error::callback_failed, error_message);
            }
            _accumulator = std::max(0.0, _accumulator - _settings.fixed_delta_time);
            ++completed_steps;
        }
        return make_result(completed_steps, fixed_step_error::none, {});
    }

    fixed_step_result fixed_step_controller::single_step(const fixed_step_callback& callback)
    {
        if (_state == playback_state::playing)
        {
            return make_result(0, fixed_step_error::step_not_allowed,
                "Single-step execution requires Pause or Stop.");
        }
        std::string error_message;
        if (!execute_one_step(callback, error_message))
        {
            return make_result(0, fixed_step_error::callback_failed, error_message);
        }
        return make_result(1, fixed_step_error::none, {});
    }

    playback_state fixed_step_controller::state() const
    {
        return _state;
    }

    double fixed_step_controller::simulation_time() const
    {
        return _simulation_time;
    }

    double fixed_step_controller::accumulator() const
    {
        return _accumulator;
    }

    const fixed_step_settings& fixed_step_controller::settings() const
    {
        return _settings;
    }
}
