#include "skin_physics/physics/msh_reader.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"
#include "skin_physics/physics/render_binding.hpp"
#include "skin_physics/simulation/fixed_step.hpp"
#include "json_test_support.hpp"
#include "test_support.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using skin_physics::asset::mesh;
    using skin_physics::asset::vertex;
    using skin_physics::physics::physics_mesh;
    using skin_physics::physics::render_binding_settings;
    using skin_physics::physics::render_binding_kind;
    using skin_physics::simulation::fixed_step_controller;

    void require(bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    physics_mesh load_single_tetrahedron()
    {
        const auto read = skin_physics::physics::read_gmsh_2_2_ascii(
            test_support::fixture_path("phase02_physics_mesh/single_tet.msh"));
        require(read.success, "Phase 2 single-tetrahedron fixture should load.");
        const auto built = skin_physics::physics::build_physics_mesh(read.mesh);
        require(built.success, "Phase 2 single-tetrahedron Physics Mesh should build.");
        return built.value;
    }

    skin_physics::asset::vec3 read_vec3(const test_json::value& value)
    {
        return {
            static_cast<float>(value.at(0).as_number()),
            static_cast<float>(value.at(1).as_number()),
            static_cast<float>(value.at(2).as_number())
        };
    }

    void test_phase03_render_binding_golden()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase03_render_binding/tet_binding.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase03_render_binding/tet_binding.expected.json"));
        const physics_mesh physics_value = load_single_tetrahedron();

        mesh render_mesh;
        const auto& render_vertices = input.at("render_vertices").as_array();
        render_mesh.vertices.resize(render_vertices.size());
        for (std::size_t index = 0; index < render_vertices.size(); ++index)
        {
            render_mesh.vertices[index].position = read_vec3(render_vertices[index].at("position"));
        }

        render_binding_settings settings;
        settings.maximum_surface_distance = 0.25f;
        settings.maximum_rest_error_relative_to_bbox_diagonal = static_cast<float>(
            input.at("thresholds").at("maximum_rest_error_relative_to_bbox_diagonal").as_number());

        const auto built = skin_physics::physics::build_render_vertex_bindings(
            render_mesh, physics_value, settings);
        require(built.success, "Phase 3 golden render bindings should be generated.");
        require(built.bindings.size() == 2, "Phase 3 golden fixture should create two bindings.");

        const auto& expected_bindings = expected.at("bindings").as_array();
        const auto& expected_interior = expected_bindings[0];
        const auto& interior = built.bindings[0];
        require(interior.kind == render_binding_kind::tetrahedron,
            "Interior Render Vertex must bind to a tetrahedron.");
        for (std::size_t index = 0; index < 4; ++index)
        {
            require(interior.physics_vertices[index] ==
                static_cast<uint32_t>(expected_interior.at("physics_vertex_ids").at(index).as_number()),
                "Interior tetrahedron vertex ID must match the golden.");
            require(std::abs(interior.barycentric[index] -
                expected_interior.at("barycentric").at(index).as_number()) <= 1.0e-6f,
                "Interior tetrahedron barycentric coordinate must match the golden.");
        }

        const auto& expected_boundary = expected_bindings[1];
        const auto& boundary = built.bindings[1];
        require(boundary.kind == render_binding_kind::boundary_triangle,
            "Exterior Render Vertex must bind to a boundary triangle.");
        for (std::size_t index = 0; index < 3; ++index)
        {
            require(boundary.physics_vertices[index] ==
                static_cast<uint32_t>(expected_boundary.at("triangle_physics_vertex_ids").at(index).as_number()),
                "Boundary triangle vertex ID must match the golden.");
            require(std::abs(boundary.barycentric[index] -
                expected_boundary.at("barycentric").at(index).as_number()) <= 1.0e-6f,
                "Boundary barycentric coordinate must match the golden.");
        }
        require(built.maximum_relative_rest_error <=
            settings.maximum_rest_error_relative_to_bbox_diagonal,
            "Phase 3 Rest Pose embedding error must satisfy its relative threshold.");

        // Rest-pose feedback must reproduce the original positions.
        physics_mesh moved_physics = physics_value;
        mesh attributed_mesh = render_mesh;
        attributed_mesh.indices = { 0, 1, 0 };
        attributed_mesh.submeshes.push_back({ "fixture_material", 0, 3, 7 });
        attributed_mesh.vertices[0].attributes =
            skin_physics::asset::attribute_texcoord_0 | skin_physics::asset::attribute_color_0;
        attributed_mesh.vertices[0].texcoord_0 = { 0.25f, 0.75f };
        attributed_mesh.vertices[0].color = { 0.1f, 0.2f, 0.3f, 0.4f };
        attributed_mesh.vertices[1].attributes =
            skin_physics::asset::attribute_texcoord_0 | skin_physics::asset::attribute_color_0;
        attributed_mesh.vertices[1].texcoord_0 = { 0.9f, 0.1f };
        attributed_mesh.vertices[1].color = { 0.8f, 0.7f, 0.6f, 0.5f };

        const auto feedback = skin_physics::physics::apply_render_feedback(
            attributed_mesh, moved_physics, built.bindings);
        require(feedback.success, "Rest-pose Render Feedback should succeed.");
        for (std::size_t index = 0; index < attributed_mesh.vertices.size(); ++index)
        {
            require(std::abs(feedback.value.vertices[index].position.x -
                attributed_mesh.vertices[index].position.x) <= 1.0e-6f &&
                std::abs(feedback.value.vertices[index].position.y -
                attributed_mesh.vertices[index].position.y) <= 1.0e-6f &&
                std::abs(feedback.value.vertices[index].position.z -
                attributed_mesh.vertices[index].position.z) <= 1.0e-6f,
                "Rest-pose feedback must reconstruct the input Render Vertex.");
            require(feedback.value.vertices[index].texcoord_0.x ==
                attributed_mesh.vertices[index].texcoord_0.x &&
                feedback.value.vertices[index].texcoord_0.y ==
                attributed_mesh.vertices[index].texcoord_0.y,
                "Render Feedback must preserve UV coordinates.");
            require(feedback.value.vertices[index].color.x ==
                attributed_mesh.vertices[index].color.x &&
                feedback.value.vertices[index].color.y ==
                attributed_mesh.vertices[index].color.y &&
                feedback.value.vertices[index].color.z ==
                attributed_mesh.vertices[index].color.z &&
                feedback.value.vertices[index].color.w ==
                attributed_mesh.vertices[index].color.w,
                "Render Feedback must preserve vertex colors.");
        }
        require(feedback.value.indices == attributed_mesh.indices,
            "Render Feedback must preserve the index buffer.");
        require(feedback.value.submeshes[0].material_index == 7,
            "Render Feedback must preserve material assignment.");

        for (skin_physics::asset::vec3& position : moved_physics.rest_positions)
        {
            position.x += 0.25f;
        }
        const auto moved_feedback = skin_physics::physics::apply_render_feedback(
            attributed_mesh, moved_physics, built.bindings);
        require(moved_feedback.success, "Translated Physics Mesh feedback should succeed.");
        require(std::abs(moved_feedback.value.vertices[0].position.x -
            (attributed_mesh.vertices[0].position.x + 0.25f)) <= 1.0e-6f,
            "Interior feedback should follow translated Physics Mesh positions.");
        require(std::abs(moved_feedback.value.vertices[1].position.x -
            (attributed_mesh.vertices[1].position.x + 0.25f)) <= 1.0e-6f,
            "Boundary offset feedback should follow translated Physics Mesh positions.");
    }

    void test_phase03_invalid_binding_is_atomic()
    {
        const test_json::value invalid_input = test_json::load(
            test_support::fixture_path("phase03_render_binding/tet_binding.invalid.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase03_render_binding/tet_binding.invalid.expected.json"));
        mesh render_mesh;
        render_mesh.vertices.resize(1);
        render_mesh.vertices[0].position = read_vec3(
            invalid_input.at("render_vertices").at(0).at("position"));

        render_binding_settings settings;
        settings.maximum_surface_distance = static_cast<float>(
            invalid_input.at("maximum_surface_distance").as_number());
        const auto result = skin_physics::physics::build_render_vertex_bindings(
            render_mesh, load_single_tetrahedron(), settings);
        require(!result.success, "Distant render vertex must fail binding.");
        require(result.bindings.empty(), "Failed binding must not partially commit bindings.");
        require(!result.diagnostics.empty(), "Failed binding must provide a diagnostic.");
        require(expected.at("success").as_bool() == result.success,
            "Invalid binding success state must match the golden.");
        require(result.diagnostics[0].code ==
            skin_physics::physics::render_binding_error::no_surface_embedding,
            "Distant render vertex must report no_surface_embedding.");
    }

    void test_phase04_fixed_step_golden()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase04_test_environment/fixed_step.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase04_test_environment/fixed_step.expected.json"));

        skin_physics::simulation::fixed_step_settings settings;
        settings.fixed_delta_time = input.at("fixed_delta_time").as_number();
        settings.time_scale = input.at("time_scale").as_number();
        settings.maximum_steps_per_advance = 8;
        fixed_step_controller controller(settings);

        uint32_t callback_count = 0;
        const skin_physics::simulation::fixed_step_callback callback =
            [&callback_count](double delta_time)
            {
                if (std::abs(delta_time - 0.02) > 1.0e-12)
                {
                    return false;
                }
                ++callback_count;
                return true;
            };

        controller.play();
        const auto frame0 = controller.advance(
            input.at("frame_delta_times").at(0).as_number(), callback);
        require(frame0.completed_steps == expected.at("after_frame_0").at("completed_steps").as_number(),
            "Phase 4 frame 0 step count must match the golden.");
        require(std::abs(frame0.accumulator - expected.at("after_frame_0").at("accumulator").as_number()) <= 1.0e-9,
            "Phase 4 frame 0 accumulator must match the golden.");

        const auto frame1 = controller.advance(
            input.at("frame_delta_times").at(1).as_number(), callback);
        require(frame1.completed_steps == 1 &&
            std::abs(frame1.simulation_time - expected.at("after_frame_1").at("simulation_time").as_number()) <= 1.0e-9 &&
            std::abs(frame1.accumulator - expected.at("after_frame_1").at("accumulator").as_number()) <= 1.0e-9,
            "Phase 4 frame 1 must produce exactly one fixed step.");

        const auto frame2 = controller.advance(
            input.at("frame_delta_times").at(2).as_number(), callback);
        controller.pause();
        require(frame2.completed_steps == 1 &&
            std::abs(frame2.accumulator - expected.at("after_frame_2_pause").at("accumulator").as_number()) <= 1.0e-9 &&
            std::abs(frame2.simulation_time - expected.at("after_frame_2_pause").at("simulation_time").as_number()) <= 1.0e-9,
            "Phase 4 frame 2 must preserve the unconsumed 0.015 seconds.");

        const auto stepped = controller.single_step(callback);
        require(stepped.completed_steps == 1 &&
            std::abs(stepped.simulation_time - expected.at("after_single_step").at("simulation_time").as_number()) <= 1.0e-9 &&
            std::abs(stepped.accumulator - expected.at("after_single_step").at("accumulator").as_number()) <= 1.0e-9,
            "Paused single-step must advance one fixed interval and preserve the accumulator.");

        controller.reset();
        require(controller.state() == skin_physics::simulation::playback_state::stopped &&
            controller.simulation_time() == expected.at("after_reset").at("simulation_time").as_number() &&
            controller.accumulator() == expected.at("after_reset").at("accumulator").as_number(),
            "Reset must restore stopped state, simulation time and accumulator.");
        require(callback_count == 3, "Phase 4 expected exactly three successful solver callbacks.");
    }

    void test_phase04_overload_and_callback_failure()
    {
        skin_physics::simulation::fixed_step_settings settings;
        settings.fixed_delta_time = 0.01;
        settings.time_scale = 1.0;
        settings.maximum_steps_per_advance = 2;
        fixed_step_controller controller(settings);
        controller.play();

        const auto limited = controller.advance(0.055, [](double) { return true; });
        require(limited.completed_steps == 2 && limited.has_backlog,
            "Step budget must preserve the unprocessed accumulator as backlog.");
        require(std::abs(limited.accumulator - 0.035) <= 1.0e-9,
            "Overload must not drop simulation time.");

        const double time_before_failure = controller.simulation_time();
        const double accumulator_before_failure = controller.accumulator();
        const auto failed = controller.advance(0.0, [](double) { return false; });
        require(!failed.success &&
            failed.error == skin_physics::simulation::fixed_step_error::callback_failed,
            "A rejected physics step must be diagnosed.");
        require(controller.simulation_time() == time_before_failure &&
            controller.accumulator() == accumulator_before_failure,
            "A rejected physics step must not commit time or consume the accumulator.");
    }
}

int run_simulation_tests()
{
    test_phase03_render_binding_golden();
    std::cout << "[PASS] P03_RENDER_BINDING_GOLDEN\n";
    test_phase03_invalid_binding_is_atomic();
    std::cout << "[PASS] P03_INVALID_BINDING\n";
    test_phase04_fixed_step_golden();
    std::cout << "[PASS] P04_FIXED_STEP_GOLDEN\n";
    test_phase04_overload_and_callback_failure();
    std::cout << "[PASS] P04_FIXED_STEP_OVERLOAD\n";
    std::cout << "SimulationCoreTests passed.\n";
    return 0;
}
