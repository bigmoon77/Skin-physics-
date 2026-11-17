#include "skin_physics/simulation/scenario.hpp"
#include "skin_physics/simulation/step_pipeline.hpp"
#include "skin_physics/simulation/fixed_step.hpp"
#include "test_support.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using skin_physics::asset::vec3;
    using skin_physics::serialization::json_value;
    using skin_physics::simulation::simulation_stage;
    using skin_physics::simulation::simulation_state;
    using skin_physics::simulation::simulation_step_pipeline;

    void require(bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    bool equal_vec3(const vec3& left, const vec3& right)
    {
        return left.x == right.x && left.y == right.y && left.z == right.z;
    }

    void test_phase04_scenario_json_round_trip()
    {
        const std::filesystem::path input_path =
            test_support::fixture_path("phase04_test_environment/scenario.input.json");
        const std::filesystem::path output_path =
            std::filesystem::temp_directory_path() / "skin_physics_scenario_round_trip.json";

        const auto loaded = skin_physics::simulation::load_scenario(input_path);
        require(loaded.success, "Phase 4 Scenario fixture must pass schema validation.");
        require(loaded.applied_defaults.empty(),
            "The reviewed Scenario fixture already supplies every required field.");
        const json_value* extension = loaded.document.find("extensions");
        require(extension != nullptr && extension->is_object(),
            "Unknown top-level extension object must be retained.");
        const json_value* note = extension->find("test_note");
        require(note != nullptr && note->is_string() &&
            note->as_string() == "unknown optional fields must survive a load-save cycle",
            "Unknown extension field must retain its exact value.");

        std::string diagnostic;
        require(skin_physics::simulation::save_scenario(
            output_path, loaded.document, diagnostic),
            "Validated Scenario must serialize: " + diagnostic);
        const auto reloaded = skin_physics::simulation::load_scenario(output_path);
        require(reloaded.success, "Saved Scenario must reload successfully.");
        require(reloaded.document == loaded.document,
            "Scenario save-load-save must preserve semantic JSON equality.");
        require(reloaded.document.find("extensions") != nullptr,
            "Scenario serialization must preserve unknown extension fields.");

        json_value unsupported = loaded.document;
        unsupported.as_object()["schema_version"].data = 99.0;
        const auto unsupported_result =
            skin_physics::simulation::validate_scenario(unsupported);
        require(!unsupported_result.success &&
            unsupported_result.diagnostic.find("schema_version") != std::string::npos,
            "Unknown required schema version must be rejected.");

        json_value non_finite = loaded.document;
        non_finite.as_object()["solver_settings"].as_object()["fixed_delta_time"].data =
            std::numeric_limits<double>::quiet_NaN();
        const auto non_finite_result =
            skin_physics::simulation::validate_scenario(non_finite);
        require(!non_finite_result.success,
            "Non-finite fixed delta time must be rejected.");

        json_value invalid_generation = loaded.document;
        invalid_generation.as_object()["asset_generation_settings"].as_object()
            ["target_tet_count"].data = 9.0;
        const auto invalid_count_result =
            skin_physics::simulation::validate_scenario(invalid_generation);
        require(!invalid_count_result.success,
            "Target tetrahedron count must not exceed max_tet_count.");

        std::error_code remove_error;
        std::filesystem::remove(output_path, remove_error);
    }

    void test_phase04_run_result_json_round_trip_and_comparison()
    {
        const std::filesystem::path input_path =
            test_support::fixture_path("phase04_test_environment/run_result.input.json");
        const std::filesystem::path output_path =
            std::filesystem::temp_directory_path() / "skin_physics_run_result_round_trip.json";

        const auto loaded = skin_physics::simulation::load_run_result(input_path);
        require(loaded.success, "Phase 4 Run Result fixture must pass schema validation.");
        const json_value* gpu = loaded.document.find("gpu");
        require(gpu != nullptr && gpu->find("available") != nullptr &&
            !gpu->find("available")->as_bool(),
            "CPU-only Run Result must mark GPU measurements unavailable.");
        const json_value* passes = loaded.document.find("passes");
        require(passes != nullptr && passes->is_array(),
            "Run Result pass list must be retained.");
        for (const json_value& pass : passes->as_array())
        {
            const json_value* gpu_time = pass.find("gpu_time_ms");
            require(gpu_time != nullptr && gpu_time->is_null(),
                "Unavailable GPU pass timing must be represented by JSON null.");
        }

        std::string diagnostic;
        require(skin_physics::simulation::save_run_result(
            output_path, loaded.document, diagnostic),
            "Validated Run Result must serialize: " + diagnostic);
        const auto reloaded = skin_physics::simulation::load_run_result(output_path);
        require(reloaded.success && reloaded.document == loaded.document,
            "Run Result save-load cycle must preserve semantic JSON equality.");
        require(skin_physics::simulation::run_results_comparable(
            loaded.document, reloaded.document),
            "Runs with matching scenario hash, asset identity and cache version must be comparable.");

        json_value different_scenario = loaded.document;
        different_scenario.as_object()["scenario_hash"].data = std::string("sha256:different");
        require(!skin_physics::simulation::run_results_comparable(
            loaded.document, different_scenario),
            "Runs from different scenario hashes must not be compared.");

        json_value invalid_gpu = loaded.document;
        invalid_gpu.as_object()["passes"].as_array()[0].as_object()["gpu_time_ms"].data = 0.01;
        const auto invalid_gpu_result =
            skin_physics::simulation::validate_run_result(invalid_gpu);
        require(!invalid_gpu_result.success,
            "A CPU-only run must not contain a fabricated GPU timing.");

        std::error_code remove_error;
        std::filesystem::remove(output_path, remove_error);
    }

    void test_simulation_step_pipeline_order_and_commit()
    {
        simulation_step_pipeline pipeline;
        std::vector<simulation_stage> order;

        require(pipeline.set_stage_callback(simulation_stage::animation_targets,
            [&order](simulation_state& state, double, std::string&)
            {
                order.push_back(simulation_stage::animation_targets);
                state.positions[0].x += 1.0f;
                return true;
            }), "Animation stage callback registration must succeed.");

        require(pipeline.set_stage_callback(simulation_stage::external_forces,
            [&order](simulation_state& state, double delta_time, std::string&)
            {
                order.push_back(simulation_stage::external_forces);
                state.velocities[0].y -= static_cast<float>(9.81 * delta_time);
                return true;
            }), "External force stage callback registration must succeed.");

        require(pipeline.set_stage_callback(simulation_stage::soft_body,
            [&order](simulation_state& state, double delta_time, std::string&)
            {
                order.push_back(simulation_stage::soft_body);
                state.positions[0].y += state.velocities[0].y * static_cast<float>(delta_time);
                return true;
            }), "Soft Body stage callback registration must succeed.");

        require(pipeline.set_stage_callback(simulation_stage::broadphase,
            [&order](simulation_state&, double, std::string&)
            {
                order.push_back(simulation_stage::broadphase);
                return true;
            }), "Broadphase stage callback registration must succeed.");

        require(pipeline.set_stage_callback(simulation_stage::narrowphase,
            [&order](simulation_state&, double, std::string&)
            {
                order.push_back(simulation_stage::narrowphase);
                return true;
            }), "Narrowphase stage callback registration must succeed.");

        require(pipeline.set_stage_callback(simulation_stage::render_feedback,
            [&order](simulation_state&, double, std::string&)
            {
                order.push_back(simulation_stage::render_feedback);
                return true;
            }), "Render Feedback stage callback registration must succeed.");

        simulation_state state;
        state.positions = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } };
        state.velocities = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };

        const auto result = pipeline.execute(state, 0.02);
        require(result.success && result.committed && result.committed_step_index == 1,
            "A completed simulation pipeline must atomically commit exactly one step.");
        require(result.completed_stages == 6 && order.size() == 6,
            "Only registered stage callbacks must execute.");
        require(order.front() == simulation_stage::animation_targets &&
            order.back() == simulation_stage::render_feedback,
            "Simulation stages must execute in design order.");
        require(std::abs(state.positions[0].x - 1.0f) <= 1.0e-6f &&
            std::abs(state.positions[0].y + 0.003924f) <= 1.0e-6f,
            "Committed state must contain the ordered stage outputs.");
    }

    void test_simulation_step_pipeline_failure_is_atomic()
    {
        simulation_step_pipeline pipeline;
        pipeline.set_stage_callback(simulation_stage::soft_body,
            [](simulation_state& state, double, std::string&)
            {
                state.positions[0].x = 100.0f;
                return true;
            });
        pipeline.set_stage_callback(simulation_stage::narrowphase,
            [](simulation_state& state, double, std::string& diagnostic)
            {
                state.positions[0].x = 200.0f;
                diagnostic = "fixture contact capacity exceeded";
                return false;
            });

        simulation_state state;
        state.positions = { { 1.0f, 2.0f, 3.0f } };
        state.velocities = { { 4.0f, 5.0f, 6.0f } };
        const simulation_state original = state;

        const auto result = pipeline.execute(state, 0.01);
        require(!result.success && !result.committed &&
            result.error == skin_physics::simulation::pipeline_error::stage_failed &&
            result.failed_stage == simulation_stage::narrowphase,
            "A stage failure must identify the exact failed stage.");
        require(result.diagnostic.find("capacity exceeded") != std::string::npos,
            "Stage failure diagnostics must preserve the cause.");
        require(state.step_index == original.step_index &&
            equal_vec3(state.positions[0], original.positions[0]) &&
            equal_vec3(state.velocities[0], original.velocities[0]),
            "A failed pipeline must roll back all earlier stage changes.");

        pipeline.clear_all_callbacks();
        pipeline.set_stage_callback(simulation_stage::cloth,
            [](simulation_state& working, double, std::string&)
            {
                working.positions[0].z =
                    std::numeric_limits<float>::quiet_NaN();
                return true;
            });
        const auto invalid_output = pipeline.execute(state, 0.01);
        require(!invalid_output.success &&
            invalid_output.error == skin_physics::simulation::pipeline_error::invalid_stage_output,
            "Non-finite solver output must block the step.");
        require(equal_vec3(state.positions[0], original.positions[0]),
            "Non-finite output must not modify the committed state.");

        skin_physics::simulation::fixed_step_settings settings;
        settings.fixed_delta_time = 0.01;
        settings.maximum_steps_per_advance = 4;
        skin_physics::simulation::fixed_step_controller clock(settings);
        clock.play();
        const auto blocked = clock.advance(0.02,
            [&pipeline, &state](double delta_time)
            {
                return pipeline.execute(state, delta_time).success;
            });
        require(!blocked.success &&
            blocked.error == skin_physics::simulation::fixed_step_error::callback_failed &&
            blocked.completed_steps == 0 &&
            clock.simulation_time() == 0.0 &&
            std::abs(clock.accumulator() - 0.02) <= 1.0e-9,
            "Fixed-step time must not commit or discard accumulated time when the pipeline blocks.");
    }
}

int run_environment_tests()
{
    test_phase04_scenario_json_round_trip();
    std::cout << "[PASS] P04_SCENARIO_JSON_ROUND_TRIP\n";
    test_phase04_run_result_json_round_trip_and_comparison();
    std::cout << "[PASS] P04_RUN_RESULT_JSON\n";
    test_simulation_step_pipeline_order_and_commit();
    std::cout << "[PASS] SIM_PIPELINE_ORDER_AND_COMMIT\n";
    test_simulation_step_pipeline_failure_is_atomic();
    std::cout << "[PASS] SIM_PIPELINE_ATOMIC_ROLLBACK\n";
    std::cout << "EnvironmentTests passed.\n";
    return 0;
}
