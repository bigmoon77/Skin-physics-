#include "skin_physics/physics/msh_reader.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"
#include "skin_physics/physics/render_binding.hpp"
#include "skin_physics/physics/cloth_solver.hpp"
#include "skin_physics/physics/soft_body_fem.hpp"
#include "skin_physics/physics/collision.hpp"
#include "skin_physics/physics/ccd.hpp"
#include "skin_physics/physics/correction_ccd.hpp"
#include "skin_physics/physics/animation_track.hpp"
#include "skin_physics/physics/attribute_paint.hpp"
#include "skin_physics/runtime/detail/force_distribution.hpp"
#include "skin_physics/physics/performance_metrics.hpp"
#include "skin_physics/simulation/fixed_step.hpp"
#include "json_test_support.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>

int run_environment_tests();
int run_gpu_solver_tests();

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

    bool initialize_test_correction_segment_geometry(
        skin_physics::physics::correction_segment_candidate_state& state,
        uint64_t base_integration_segment_id,
        const std::vector<skin_physics::physics::correction_primitive_epoch_input>& primitives,
        float contact_thickness)
    {
        std::map<uint64_t, skin_physics::asset::vec3> unique_start_positions;
        std::vector<skin_physics::physics::correction_vertex_motion>
            envelope_vertex_motions;

        for (const auto& primitive : primitives)
        {
            if (primitive.vertex_stable_ids.size() !=
                    primitive.segment_start_positions.size() ||
                primitive.vertex_stable_ids.size() !=
                    primitive.vertex_motions.size())
            {
                return false;
            }

            for (std::size_t index = 0;
                index < primitive.vertex_stable_ids.size(); ++index)
            {
                const uint64_t stable_id =
                    primitive.vertex_stable_ids[index];
                const skin_physics::asset::vec3& start_position =
                    primitive.segment_start_positions[index];
                const auto inserted = unique_start_positions.emplace(
                    stable_id, start_position);
                if (!inserted.second &&
                    (inserted.first->second.x != start_position.x ||
                    inserted.first->second.y != start_position.y ||
                    inserted.first->second.z != start_position.z))
                {
                    return false;
                }

                envelope_vertex_motions.push_back({
                    start_position, primitive.vertex_motions[index].before
                });
                envelope_vertex_motions.push_back(
                    primitive.vertex_motions[index]);
            }
        }

        if (envelope_vertex_motions.empty())
        {
            envelope_vertex_motions.push_back({
                { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }
            });
        }

        skin_physics::physics::collision_aabb physical_swept_bounds;
        if (!skin_physics::physics::build_correction_swept_aabb(
            envelope_vertex_motions, contact_thickness,
            physical_swept_bounds))
        {
            return false;
        }

        std::vector<skin_physics::physics::correction_segment_vertex_snapshot>
            segment_start_snapshot;
        segment_start_snapshot.reserve(unique_start_positions.size());
        for (const auto& entry : unique_start_positions)
        {
            segment_start_snapshot.push_back({ entry.first, entry.second });
        }

        return skin_physics::physics::initialize_correction_segment_geometry(
            state, base_integration_segment_id,
            segment_start_snapshot, physical_swept_bounds);
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

    skin_physics::physics::cloth_build_result load_cloth_fixture()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase05_cloth/cloth_patch.input.json"));
        skin_physics::physics::raw_cloth_mesh raw;
        for (const test_json::value& position : input.at("positions").as_array())
        {
            raw.rest_positions.push_back(read_vec3(position));
        }
        for (const test_json::value& triangle_value : input.at("triangles").as_array())
        {
            raw.triangles.push_back({
                static_cast<uint32_t>(triangle_value.at(0).as_number()),
                static_cast<uint32_t>(triangle_value.at(1).as_number()),
                static_cast<uint32_t>(triangle_value.at(2).as_number())
            });
        }
        return skin_physics::physics::build_cloth_mesh(raw);
    }

    void test_phase05_cloth_fixture_and_bend_gradient()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase05_cloth/cloth_patch.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase05_cloth/cloth_patch.expected.json"));
        const auto built = load_cloth_fixture();

        require(built.success, "Phase 5 cloth fixture should build.");
        require(built.value.stretch_constraints.size() == 5,
            "Cloth patch should contain five unique stretch edges.");
        require(built.value.shear_constraints.size() == 2,
            "Cloth patch should contain one shear constraint per triangle.");
        require(built.value.bend_constraints.size() == 1,
            "Cloth patch should contain one bend hinge.");

        const auto& input_edges = input.at("stretch_edges").as_array();
        const auto& expected_lengths = expected.at("rest_edge_lengths").as_array();
        for (std::size_t edge_index = 0; edge_index < input_edges.size(); ++edge_index)
        {
            const uint32_t a = static_cast<uint32_t>(input_edges[edge_index].at(0).as_number());
            const uint32_t b = static_cast<uint32_t>(input_edges[edge_index].at(1).as_number());
            bool found = false;
            for (const auto& constraint : built.value.stretch_constraints)
            {
                if (constraint.vertices[0] == std::min(a, b) &&
                    constraint.vertices[1] == std::max(a, b))
                {
                    found = true;
                    require(std::abs(constraint.rest_length -
                        expected_lengths[edge_index].as_number()) <= 1.0e-6f,
                        "Cloth Rest edge length must match the golden.");
                    break;
                }
            }
            require(found, "Each fixture stretch edge must exist in the built topology.");
        }

        const auto& expected_areas = expected.at("rest_triangle_areas").as_array();
        for (std::size_t triangle_index = 0;
            triangle_index < built.value.triangles.size(); ++triangle_index)
        {
            const auto& triangle = built.value.triangles[triangle_index];
            const auto& p0 = built.value.rest_positions[triangle[0]];
            const auto& p1 = built.value.rest_positions[triangle[1]];
            const auto& p2 = built.value.rest_positions[triangle[2]];
            const skin_physics::asset::vec3 edge1 = {
                p1.x - p0.x, p1.y - p0.y, p1.z - p0.z };
            const skin_physics::asset::vec3 edge2 = {
                p2.x - p0.x, p2.y - p0.y, p2.z - p0.z };
            const skin_physics::asset::vec3 normal = {
                edge1.y * edge2.z - edge1.z * edge2.y,
                edge1.z * edge2.x - edge1.x * edge2.z,
                edge1.x * edge2.y - edge1.y * edge2.x
            };
            const float area = 0.5f * std::sqrt(
                normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            require(std::abs(area - expected_areas[triangle_index].as_number()) <= 1.0e-6f,
                "Cloth Rest triangle area must match the golden.");
        }

        const auto& bend = built.value.bend_constraints[0];
        std::array<skin_physics::asset::vec3, 4> rest_hinge = {};
        for (std::size_t index = 0; index < 4; ++index)
        {
            rest_hinge[index] = built.value.rest_positions[bend.vertices[index]];
        }
        const auto rest_evaluation =
            skin_physics::physics::evaluate_cloth_bend(rest_hinge, bend.rest_angle);
        require(rest_evaluation.valid &&
            std::abs(rest_evaluation.constraint -
                expected.at("rest_dihedral_radians").as_number()) <= 1.0e-6,
            "Cloth Rest dihedral angle must match the golden.");

        std::array<skin_physics::asset::vec3, 4> distorted = rest_hinge;
        distorted[3].z += 0.23f;
        const auto analytic =
            skin_physics::physics::evaluate_cloth_bend(distorted, bend.rest_angle);
        require(analytic.valid, "Analytic Bend gradient should be valid.");

        const float epsilon = 1.0e-3f;
        double squared_error = 0.0;
        double squared_reference = 0.0;
        for (std::size_t vertex_index = 0; vertex_index < 4; ++vertex_index)
        {
            for (uint32_t axis = 0; axis < 3; ++axis)
            {
                std::array<skin_physics::asset::vec3, 4> plus = distorted;
                std::array<skin_physics::asset::vec3, 4> minus = distorted;
                if (axis == 0)
                {
                    plus[vertex_index].x += epsilon;
                    minus[vertex_index].x -= epsilon;
                }
                else if (axis == 1)
                {
                    plus[vertex_index].y += epsilon;
                    minus[vertex_index].y -= epsilon;
                }
                else
                {
                    plus[vertex_index].z += epsilon;
                    minus[vertex_index].z -= epsilon;
                }

                const auto plus_value =
                    skin_physics::physics::evaluate_cloth_bend(plus, bend.rest_angle);
                const auto minus_value =
                    skin_physics::physics::evaluate_cloth_bend(minus, bend.rest_angle);
                require(plus_value.valid && minus_value.valid,
                    "Finite-difference Bend samples must remain valid.");
                const double numeric_gradient =
                    (plus_value.constraint - minus_value.constraint) / (2.0 * epsilon);
                double analytic_gradient = 0.0;
                if (axis == 0) analytic_gradient = analytic.gradients[vertex_index].x;
                if (axis == 1) analytic_gradient = analytic.gradients[vertex_index].y;
                if (axis == 2) analytic_gradient = analytic.gradients[vertex_index].z;

                const double difference = analytic_gradient - numeric_gradient;
                squared_error += difference * difference;
                squared_reference += numeric_gradient * numeric_gradient;
            }
        }
        const double relative_error =
            std::sqrt(squared_error) / std::max(1.0e-8, std::sqrt(squared_reference));
        require(relative_error <=
            expected.at("gradient_test").at("relative_error_max").as_number(),
            "Analytic Bend gradient must agree with central finite differences.");
    }

    float cloth_stretch_error(
        const skin_physics::physics::cloth_mesh& mesh,
        const std::vector<skin_physics::asset::vec3>& positions)
    {
        float error = 0.0f;
        for (const auto& constraint : mesh.stretch_constraints)
        {
            const auto& a = positions[constraint.vertices[0]];
            const auto& b = positions[constraint.vertices[1]];
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            error += std::abs(std::sqrt(dx * dx + dy * dy + dz * dz) -
                constraint.rest_length);
        }
        return error;
    }

    float cloth_shear_error(
        const skin_physics::physics::cloth_mesh& mesh,
        const std::vector<skin_physics::asset::vec3>& positions)
    {
        float error = 0.0f;
        for (const auto& constraint : mesh.shear_constraints)
        {
            const auto& p0 = positions[constraint.vertices[0]];
            const auto& p1 = positions[constraint.vertices[1]];
            const auto& p2 = positions[constraint.vertices[2]];
            const float ux = p1.x - p0.x;
            const float uy = p1.y - p0.y;
            const float uz = p1.z - p0.z;
            const float vx = p2.x - p0.x;
            const float vy = p2.y - p0.y;
            const float vz = p2.z - p0.z;
            const float current_dot = ux * vx + uy * vy + uz * vz;
            error += std::abs((current_dot - constraint.rest_dot) /
                constraint.normalization_length);
        }
        return error;
    }

    float cloth_bend_error(
        const skin_physics::physics::cloth_mesh& mesh,
        const std::vector<skin_physics::asset::vec3>& positions)
    {
        float error = 0.0f;
        for (const auto& constraint : mesh.bend_constraints)
        {
            std::array<skin_physics::asset::vec3, 4> points = {};
            for (std::size_t index = 0; index < 4; ++index)
            {
                points[index] = positions[constraint.vertices[index]];
            }
            const auto evaluation =
                skin_physics::physics::evaluate_cloth_bend(points, constraint.rest_angle);
            if (!evaluation.valid) return std::numeric_limits<float>::max();
            error += std::abs(evaluation.constraint);
        }
        return error;
    }

    void test_phase05_cloth_compliance_monotonicity()
    {
        const auto built = load_cloth_fixture();
        require(built.success, "Cloth fixture should build for compliance tests.");
        const std::vector<float> inverse_mass(built.value.rest_positions.size(), 1.0f);

        for (uint32_t family = 0; family < 3; ++family)
        {
            std::vector<skin_physics::asset::vec3> deformed = built.value.rest_positions;
            if (family < 2) deformed[3].x += 0.25f;
            else deformed[3].z += 0.35f;

            skin_physics::physics::cloth_solver_settings soft_settings;
            soft_settings.fixed_delta_time = 0.02f;
            soft_settings.iterations = 24;
            soft_settings.relaxation = 0.7f;
            soft_settings.stretch_compliance = 0.1f;
            soft_settings.shear_compliance = 0.1f;
            soft_settings.bend_compliance = 0.1f;
            soft_settings.solve_stretch = family == 0;
            soft_settings.solve_shear = family == 1;
            soft_settings.solve_bend = family == 2;

            const auto soft_result = skin_physics::physics::solve_cloth(
                built.value, deformed, inverse_mass, soft_settings);
            require(soft_result.success, "High-compliance Cloth solve should succeed.");

            skin_physics::physics::cloth_solver_settings stiff_settings = soft_settings;
            stiff_settings.stretch_compliance = 1.0e-5f;
            stiff_settings.shear_compliance = 1.0e-5f;
            stiff_settings.bend_compliance = 1.0e-5f;
            const auto stiff_result = skin_physics::physics::solve_cloth(
                built.value, deformed, inverse_mass, stiff_settings);
            require(stiff_result.success, "Low-compliance Cloth solve should succeed.");

            float soft_error = 0.0f;
            float stiff_error = 0.0f;
            if (family == 0)
            {
                soft_error = cloth_stretch_error(built.value, soft_result.positions);
                stiff_error = cloth_stretch_error(built.value, stiff_result.positions);
            }
            else if (family == 1)
            {
                soft_error = cloth_shear_error(built.value, soft_result.positions);
                stiff_error = cloth_shear_error(built.value, stiff_result.positions);
            }
            else
            {
                soft_error = cloth_bend_error(built.value, soft_result.positions);
                stiff_error = cloth_bend_error(built.value, stiff_result.positions);
            }
            require(stiff_error <= soft_error + 1.0e-5f,
                "Reducing Cloth compliance must not increase the corresponding constraint residual.");
        }
    }

    void test_phase06_corotational_fem_golden()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase06_soft_body/tet_deformation.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase06_soft_body/tet_deformation.expected.json"));
        const physics_mesh mesh_value = load_single_tetrahedron();

        skin_physics::physics::soft_body_material material;
        material.shear_modulus = static_cast<float>(input.at("mu").as_number());
        material.bulk_modulus = static_cast<float>(input.at("K").as_number());
        material.density = static_cast<float>(input.at("density").as_number());
        material.inversion_stiffness = 10000.0f;
        material.minimum_jacobian = 0.1f;

        const auto& cases = input.at("deformation_cases").as_array();
        std::vector<skin_physics::asset::vec3> rotated_positions;
        std::vector<skin_physics::asset::vec3> compressed_positions;
        for (const test_json::value& current_case : cases)
        {
            std::vector<skin_physics::asset::vec3> positions;
            for (const test_json::value& position : current_case.at("current_positions").as_array())
            {
                positions.push_back(read_vec3(position));
            }
            const auto evaluated =
                skin_physics::physics::evaluate_corotational_tetrahedron(
                    mesh_value, 0, positions, material);
            require(evaluated.success, "Phase 6 Corotational FEM fixture evaluation should succeed.");

            const std::string& case_name = current_case.at("name").as_string();
            if (case_name == "rigid_rotation_z_90")
            {
                rotated_positions = positions;
                const auto& expected_f = expected.at("rigid_rotation_z_90").at("F");
                for (std::size_t row = 0; row < 3; ++row)
                {
                    for (std::size_t column = 0; column < 3; ++column)
                    {
                        require(std::abs(evaluated.deformation_gradient.value[row * 3 + column] -
                            expected_f.at(row).at(column).as_number()) <= 1.0e-6,
                            "Rigid rotation deformation gradient must match the golden.");
                        require(std::abs(evaluated.rotation.value[row * 3 + column] -
                            expected_f.at(row).at(column).as_number()) <= 1.0e-5,
                            "Corotational rotation must preserve a rigid 90-degree rotation.");
                    }
                }
                require(std::abs(evaluated.jacobian -
                    expected.at("rigid_rotation_z_90").at("determinant").as_number()) <= 1.0e-6,
                    "Rigid rotation Jacobian must equal one.");
                require(evaluated.strain_energy_density <=
                    expected.at("rigid_rotation_z_90").at("elastic_energy").as_number() + 1.0e-5,
                    "Rigid rotation must have near-zero Corotational strain energy.");
                require(evaluated.state_committable,
                    "A valid rigid rotation must be committable.");
            }
            else if (case_name == "compress_x_10_percent")
            {
                compressed_positions = positions;
                const auto& compression = expected.at("compress_x_10_percent");
                require(std::abs(evaluated.jacobian -
                    compression.at("determinant").as_number()) <= 1.0e-6,
                    "Ten percent X compression must produce J=0.9.");
                require(std::abs(evaluated.current_volume -
                    compression.at("current_volume").as_number()) <= 1.0e-6,
                    "Compressed tetrahedron volume must match the golden.");
                require(evaluated.state_committable,
                    "A positive-Jacobian compression should remain committable.");

                float force_sum_x = 0.0f;
                float force_sum_y = 0.0f;
                float force_sum_z = 0.0f;
                for (const skin_physics::asset::vec3& force : evaluated.internal_forces)
                {
                    force_sum_x += force.x;
                    force_sum_y += force.y;
                    force_sum_z += force.z;
                }
                require(std::abs(force_sum_x) <= 1.0e-4f &&
                    std::abs(force_sum_y) <= 1.0e-4f &&
                    std::abs(force_sum_z) <= 1.0e-4f,
                    "Internal FEM forces must conserve linear momentum within one element.");

                skin_physics::physics::soft_body_material no_shape_recovery = material;
                no_shape_recovery.shape_recovery_scale = 0.0f;
                const auto volume_only =
                    skin_physics::physics::evaluate_corotational_tetrahedron(
                        mesh_value, 0, positions, no_shape_recovery);
                skin_physics::physics::soft_body_material no_volume_retention = material;
                no_volume_retention.volume_retention_scale = 0.0f;
                const auto shape_only =
                    skin_physics::physics::evaluate_corotational_tetrahedron(
                        mesh_value, 0, positions, no_volume_retention);
                require(volume_only.success && shape_only.success,
                    "Independent FEM material scales must evaluate successfully.");
                require(volume_only.elastic_energy < evaluated.elastic_energy &&
                    shape_only.elastic_energy < evaluated.elastic_energy,
                    "Shape recovery and volume retention scales must control independent energy terms.");
            }
        }
        require(!rotated_positions.empty() && !compressed_positions.empty(),
            "Both reviewed Phase 6 deformation scenarios must be tested.");
    }

    void test_phase06_inversion_penalty_is_finite_and_not_committed()
    {
        const physics_mesh mesh_value = load_single_tetrahedron();
        std::vector<skin_physics::asset::vec3> inverted_positions = mesh_value.rest_positions;
        inverted_positions[1].x = -0.1f;

        skin_physics::physics::soft_body_material material;
        material.shear_modulus = 1000.0f;
        material.bulk_modulus = 5000.0f;
        material.density = 1000.0f;
        material.inversion_stiffness = 10000.0f;
        material.minimum_jacobian = 0.1f;

        const auto evaluated =
            skin_physics::physics::evaluate_corotational_tetrahedron(
                mesh_value, 0, inverted_positions, material);
        require(evaluated.success, "Inverted tetrahedron should produce a finite diagnostic evaluation.");
        require(evaluated.jacobian < 0.0f && evaluated.inverted,
            "Negative deformation determinant must be diagnosed as an inverted tetrahedron.");
        require(!evaluated.state_committable,
            "Inverted tetrahedron state must not be committed.");
        require(evaluated.used_svd_fallback,
            "Inverted deformation must use the proper-rotation SVD fallback.");
        require(skin_physics::physics::determinant_matrix3(evaluated.rotation) > 0.999f,
            "SVD fallback must return a proper rotation with positive determinant.");
        require(std::isfinite(evaluated.inversion_energy_density) &&
            evaluated.inversion_energy_density > 0.0f &&
            std::isfinite(evaluated.elastic_energy),
            "Inversion penalty and total energy must remain finite.");
        require(!evaluated.diagnostics.empty() &&
            evaluated.diagnostics[0].code ==
                skin_physics::physics::soft_body_error::inverted_tetrahedron,
            "Inverted element must report InvertedTetrahedron.");
        for (const skin_physics::asset::vec3& force : evaluated.internal_forces)
        {
            require(std::isfinite(force.x) && std::isfinite(force.y) && std::isfinite(force.z),
                "Inversion penalty forces must remain finite.");
        }
    }

    void test_phase07_vertex_triangle_and_edge_edge_contacts()
    {
        const test_json::value vt_input = test_json::load(
            test_support::fixture_path("phase07_collision/vt_contact.input.json"));
        const test_json::value vt_expected = test_json::load(
            test_support::fixture_path("phase07_collision/vt_contact.expected.json"));

        std::array<skin_physics::asset::vec3, 3> triangle = {};
        for (std::size_t index = 0; index < 3; ++index)
        {
            triangle[index] = read_vec3(vt_input.at("triangle").at(index));
        }
        const skin_physics::asset::vec3 point = read_vec3(vt_input.at("vertex_position"));
        const auto vt_contact = skin_physics::physics::find_vertex_triangle_contact(
            point, triangle, static_cast<float>(vt_input.at("vertex_radius").as_number()));
        require(vt_contact.valid && vt_contact.hit,
            "Phase 7 VT fixture should produce a valid penetrating contact.");

        const double position_tolerance =
            vt_expected.at("tolerances").at("position_absolute").as_number();
        const double penetration_tolerance =
            vt_expected.at("tolerances").at("penetration_absolute").as_number();
        const skin_physics::asset::vec3 expected_closest =
            read_vec3(vt_expected.at("closest_point"));
        const skin_physics::asset::vec3 expected_normal =
            read_vec3(vt_expected.at("normal"));
        require(std::abs(vt_contact.closest_point.x - expected_closest.x) <= position_tolerance &&
            std::abs(vt_contact.closest_point.y - expected_closest.y) <= position_tolerance &&
            std::abs(vt_contact.closest_point.z - expected_closest.z) <= position_tolerance,
            "VT closest point must match the analytical fixture.");
        for (std::size_t index = 0; index < 3; ++index)
        {
            require(std::abs(vt_contact.barycentric[index] -
                vt_expected.at("barycentric").at(index).as_number()) <= position_tolerance,
                "VT barycentric coordinates must match the analytical fixture.");
        }
        require(std::abs(vt_contact.normal.x - expected_normal.x) <= position_tolerance &&
            std::abs(vt_contact.normal.y - expected_normal.y) <= position_tolerance &&
            std::abs(vt_contact.normal.z - expected_normal.z) <= position_tolerance,
            "VT contact normal must match the analytical fixture.");
        require(std::abs(vt_contact.penetration_depth -
            vt_expected.at("penetration_depth").as_number()) <= penetration_tolerance,
            "VT penetration depth must match the analytical fixture.");

        std::array<skin_physics::asset::vec3, 4> positions = {
            point, triangle[0], triangle[1], triangle[2]
        };
        std::array<float, 4> inverse_masses = {
            static_cast<float>(vt_input.at("inverse_masses").at("vertex").as_number()),
            static_cast<float>(vt_input.at("inverse_masses").at("triangle_vertices").at(0).as_number()),
            static_cast<float>(vt_input.at("inverse_masses").at("triangle_vertices").at(1).as_number()),
            static_cast<float>(vt_input.at("inverse_masses").at("triangle_vertices").at(2).as_number())
        };
        const auto solved = skin_physics::physics::solve_vertex_triangle_contact(
            vt_contact, positions, inverse_masses,
            static_cast<float>(vt_input.at("vertex_radius").as_number()));
        require(solved.success && solved.momentum_conserved,
            "VT correction should solve without violating weighted momentum conservation.");
        require(std::abs(solved.post_solve_penetration -
            vt_expected.at("post_solve_penetration_max").as_number()) <= penetration_tolerance,
            "VT Jacobi relaxation should leave the expected half-step penetration.");

        const test_json::value ee_input = test_json::load(
            test_support::fixture_path("phase07_collision/ee_contact.input.json"));
        const test_json::value ee_expected = test_json::load(
            test_support::fixture_path("phase07_collision/ee_contact.expected.json"));
        std::array<skin_physics::asset::vec3, 2> edge_a = {
            read_vec3(ee_input.at("edge_a").at(0)),
            read_vec3(ee_input.at("edge_a").at(1))
        };
        std::array<skin_physics::asset::vec3, 2> edge_b = {
            read_vec3(ee_input.at("edge_b").at(0)),
            read_vec3(ee_input.at("edge_b").at(1))
        };
        const auto ee_contact = skin_physics::physics::find_edge_edge_contact(
            edge_a, edge_b, static_cast<float>(ee_input.at("thickness").as_number()));
        require(ee_contact.valid && ee_contact.hit,
            "Phase 7 EE fixture should produce a valid contact.");
        const double ee_tolerance =
            ee_expected.at("tolerances").at("distance_absolute").as_number();
        require(std::abs(ee_contact.parameter_a -
            ee_expected.at("closest_parameters").at(0).as_number()) <=
            ee_expected.at("tolerances").at("parameter_absolute").as_number(),
            "EE first closest-segment parameter must match the fixture.");
        require(std::abs(ee_contact.parameter_b -
            ee_expected.at("closest_parameters").at(1).as_number()) <=
            ee_expected.at("tolerances").at("parameter_absolute").as_number(),
            "EE second closest-segment parameter must match the fixture.");
        require(std::abs(ee_contact.distance - ee_expected.at("distance").as_number()) <= ee_tolerance &&
            std::abs(ee_contact.penetration_depth -
                ee_expected.at("penetration_depth").as_number()) <= ee_tolerance,
            "EE distance and penetration must match the fixture.");
    }

    void test_phase07_broadphase_filters_and_capacity_overflow()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase07_collision/broadphase.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase07_collision/broadphase.expected.json"));

        std::vector<skin_physics::physics::broadphase_primitive> primitives;
        for (const test_json::value& primitive_value : input.at("primitives").as_array())
        {
            skin_physics::physics::broadphase_primitive primitive;
            primitive.mesh_id = static_cast<uint32_t>(primitive_value.at("mesh").as_number());
            primitive.bounds.minimum = read_vec3(primitive_value.at("bounds_min"));
            primitive.bounds.maximum = read_vec3(primitive_value.at("bounds_max"));
            primitives.push_back(primitive);
        }

        std::vector<skin_physics::physics::broadphase_mesh_pair> mesh_pairs;
        for (const test_json::value& pair_value : input.at("mesh_pairs").as_array())
        {
            skin_physics::physics::broadphase_mesh_pair pair;
            pair.mesh_a = static_cast<uint32_t>(pair_value.at("a").as_number());
            pair.mesh_b = static_cast<uint32_t>(pair_value.at("b").as_number());
            pair.enabled = pair_value.at("enabled").as_bool();
            mesh_pairs.push_back(pair);
        }

        skin_physics::physics::broadphase_settings settings;
        settings.cell_size = static_cast<float>(input.at("cell_size").as_number());
        settings.self_collision_enabled = input.at("self_collision_enabled").as_bool();
        const auto result = skin_physics::physics::generate_broadphase_candidates(
            primitives, mesh_pairs, settings);
        require(result.success && !result.overflow,
            "Phase 7 broadphase fixture should generate candidates.");
        const auto& reference = expected.at("exact_reference_candidate_set").as_array();
        for (const test_json::value& pair_value : reference)
        {
            const skin_physics::physics::collision_candidate target = {
                static_cast<uint32_t>(pair_value.at(0).as_number()),
                static_cast<uint32_t>(pair_value.at(1).as_number())
            };
            require(std::find(result.candidates.begin(), result.candidates.end(), target) !=
                result.candidates.end(),
                "Broadphase candidate set must contain the exhaustive reference pair.");
        }
        require(result.candidates.size() == 1 &&
            result.candidates[0].primitive_a == 0 &&
            result.candidates[0].primitive_b == 1,
            "Disabled mesh pairs must be excluded and candidate IDs must not be duplicated.");

        const test_json::value overflow_input = test_json::load(
            test_support::fixture_path("phase07_collision/overflow.input.json"));
        const test_json::value overflow_expected = test_json::load(
            test_support::fixture_path("phase07_collision/overflow.expected.json"));
        const auto blocked = skin_physics::physics::check_candidate_capacity(
            static_cast<std::size_t>(overflow_input.at("generated_candidate_count").as_number()),
            static_cast<std::size_t>(overflow_input.at("candidate_capacity").as_number()));
        require(blocked.overflow && !blocked.state_committable &&
            blocked.required_count == 2,
            "Broadphase candidate overflow must block the simulation step without dropping candidates.");
        require(overflow_expected.at("first_attempt").at("dropped_candidates").as_number() == 0 &&
            !overflow_expected.at("first_attempt").at("committed").as_bool(),
            "Overflow golden must specify zero dropped candidates and no commit.");

        const auto retry = skin_physics::physics::check_candidate_capacity(
            static_cast<std::size_t>(overflow_input.at("generated_candidate_count").as_number()),
            static_cast<std::size_t>(overflow_input.at("retry_capacity").as_number()));
        require(!retry.overflow && retry.state_committable &&
            overflow_expected.at("retry").at("candidate_count").as_number() == retry.required_count,
            "Retry with sufficient candidate capacity must be allowed to commit.");
    }

    void test_phase08_ccd_and_initial_penetration_repair()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase08_ccd/point_triangle.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase08_ccd/point_triangle.expected.json"));
        const skin_physics::asset::vec3 point_start = read_vec3(input.at("point_start"));
        const skin_physics::asset::vec3 point_end = read_vec3(input.at("point_end"));
        std::array<skin_physics::asset::vec3, 3> triangle = {};
        for (std::size_t index = 0; index < 3; ++index)
        {
            triangle[index] = read_vec3(input.at("triangle").at(index));
        }

        const float point_radius = static_cast<float>(input.at("point_radius").as_number());
        const float triangle_thickness =
            static_cast<float>(input.at("triangle_thickness").as_number());
        const float interval = static_cast<float>(input.at("interval_seconds").as_number());
        const auto ccd = skin_physics::physics::conservative_advance_point_triangle(
            point_start, point_end, triangle,
            point_radius + triangle_thickness, interval);
        require(ccd.status == skin_physics::physics::ccd_status::hit && ccd.hit,
            "Phase 8 swept point-triangle fixture should report a time of impact.");
        require(std::abs(ccd.toi_seconds -
            expected.at("first_contact_toi_seconds").as_number()) <=
            expected.at("tolerances").at("toi_seconds").as_number(),
            "Conservative Advancement TOI must match the analytical fixture.");
        const skin_physics::asset::vec3 expected_center =
            read_vec3(expected.at("first_contact_center"));
        const skin_physics::asset::vec3 expected_normal =
            read_vec3(expected.at("contact_normal"));
        require(std::abs(ccd.contact_center.z - expected_center.z) <=
            expected.at("tolerances").at("position_absolute").as_number() &&
            std::abs(ccd.contact_normal.z - expected_normal.z) <= 1.0e-5,
            "CCD first contact center and normal must match the fixture.");
        require(std::abs(ccd.remaining_interval_seconds -
            expected.at("remaining_interval_seconds").as_number()) <= 1.0e-5,
            "CCD must preserve the remaining simulation interval.");

        const float signed_endpoint_distance =
            point_end.z - triangle[0].z;
        require(signed_endpoint_distance < point_radius + triangle_thickness,
            "Discrete endpoint should be on the penetrating side of the one-sided surface.");

        const float normal_velocity =
            static_cast<float>(input.at("normal_velocity").as_number());
        const skin_physics::asset::vec3 point_velocity = {
            0.0f, 0.0f, normal_velocity
        };
        const auto impulse = skin_physics::physics::resolve_toi_normal_impulse(
            point_velocity, {}, expected_normal,
            1.0f, 0.0f, 0.0f, false);
        require(impulse.success && impulse.application_count ==
            expected.at("impact_impulse_application_count").as_number() &&
            std::abs(impulse.velocity_point_after.z) <= 1.0e-6f,
            "TOI impact impulse must be applied exactly once.");
        const auto duplicate_impulse = skin_physics::physics::resolve_toi_normal_impulse(
            impulse.velocity_point_after, impulse.velocity_triangle_after,
            expected_normal, 1.0f, 0.0f, 0.0f, true);
        require(duplicate_impulse.success && duplicate_impulse.application_count == 0,
            "An already-resolved TOI must not apply the impact impulse twice.");

        const test_json::value repair_input = test_json::load(
            test_support::fixture_path("phase08_ccd/initial_repair.input.json"));
        const test_json::value repair_expected = test_json::load(
            test_support::fixture_path("phase08_ccd/initial_repair.expected.json"));
        std::vector<skin_physics::asset::vec3> initial_positions(3);
        initial_positions[1] = { 1.0f, 0.0f, 0.0f };
        initial_positions[2] = { 0.0f, 1.0f, 0.0f };
        std::vector<uint32_t> participants;
        for (const test_json::value& participant : repair_input.at("participant_vertex_ids").as_array())
        {
            participants.push_back(static_cast<uint32_t>(participant.as_number()));
        }
        const auto repaired = skin_physics::physics::repair_initial_penetration(
            initial_positions, participants, { 0.0f, 0.0f, 1.0f },
            static_cast<float>(repair_input.at("initial_penetration_depth").as_number()),
            static_cast<float>(repair_input.at("max_initial_repair_distance_per_vertex").as_number()));
        require(repaired.status == skin_physics::physics::initial_repair_status::over_budget &&
            !repaired.state_committed &&
            !repair_expected.at("state_committed").as_bool() &&
            !repair_expected.at("partial_correction_committed").as_bool(),
            "Initial penetration repair over budget must be rejected atomically.");
        require(repaired.positions.size() == initial_positions.size() &&
            repaired.positions[1].x == initial_positions[1].x &&
            repaired.positions[1].y == initial_positions[1].y &&
            repaired.positions[1].z == initial_positions[1].z,
            "Rejected initial repair must preserve all input positions.");
    }

    void test_phase09_animation_target_sampling()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase09_animation/target_motion.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase09_animation/target_motion.expected.json"));

        skin_physics::physics::animation_track track;
        const auto& times = input.at("translation_track").at("times").as_array();
        const auto& translations = input.at("translation_track").at("values").as_array();
        const auto& rotations = input.at("rotation_track").at("values").as_array();
        for (std::size_t index = 0; index < times.size(); ++index)
        {
            skin_physics::physics::animation_keyframe keyframe;
            keyframe.time = times[index].as_number();
            keyframe.translation = read_vec3(translations[index]);
            keyframe.rotation = {
                static_cast<float>(rotations[index].at(0).as_number()),
                static_cast<float>(rotations[index].at(1).as_number()),
                static_cast<float>(rotations[index].at(2).as_number()),
                static_cast<float>(rotations[index].at(3).as_number())
            };
            track.keyframes.push_back(keyframe);
        }

        const double sample_time = input.at("sample_time").as_number();
        const auto sample =
            skin_physics::physics::sample_animation_track(track, sample_time);
        require(sample.success, "Phase 9 animation track sample should succeed.");
        const skin_physics::asset::vec3 expected_position =
            read_vec3(expected.at("translation_at_sample"));
        const skin_physics::asset::vec3 expected_velocity =
            read_vec3(expected.at("target_velocity"));
        const double position_tolerance =
            expected.at("tolerances").at("position_absolute").as_number();
        const double velocity_tolerance =
            expected.at("tolerances").at("velocity_absolute").as_number();
        require(std::abs(sample.value.translation.x - expected_position.x) <= position_tolerance &&
            std::abs(sample.value.translation.y - expected_position.y) <= position_tolerance &&
            std::abs(sample.value.translation.z - expected_position.z) <= position_tolerance,
            "Animation translation sample must match the golden.");
        require(std::abs(sample.value.translation_velocity.x - expected_velocity.x) <= velocity_tolerance &&
            std::abs(sample.value.translation_velocity.y - expected_velocity.y) <= velocity_tolerance &&
            std::abs(sample.value.translation_velocity.z - expected_velocity.z) <= velocity_tolerance,
            "Animation target velocity must match the keyframe segment derivative.");

        const float expected_sine = std::sin(11.25f * 3.14159265358979323846f / 180.0f);
        const float expected_cosine = std::cos(11.25f * 3.14159265358979323846f / 180.0f);
        require(std::abs(sample.value.rotation.y - expected_sine) <=
            expected.at("tolerances").at("quaternion_absolute").as_number() &&
            std::abs(sample.value.rotation.w - expected_cosine) <=
            expected.at("tolerances").at("quaternion_absolute").as_number(),
            "Animation rotation must use shortest-path quaternion Slerp.");

        skin_physics::physics::animation_sample_options teleport;
        teleport.teleport = true;
        teleport.zero_velocity_on_teleport = true;
        teleport.teleport_translation = read_vec3(input.at("teleport").at("to"));
        const auto teleported =
            skin_physics::physics::sample_animation_track(track, sample_time, teleport);
        const skin_physics::asset::vec3 expected_teleport =
            read_vec3(expected.at("teleport_target"));
        const skin_physics::asset::vec3 expected_zero_velocity =
            read_vec3(expected.at("teleport_velocity"));
        require(teleported.success &&
            std::abs(teleported.value.translation.x - expected_teleport.x) <= position_tolerance &&
            std::abs(teleported.value.translation_velocity.x - expected_zero_velocity.x) <= velocity_tolerance &&
            std::abs(teleported.value.translation_velocity.y - expected_zero_velocity.y) <= velocity_tolerance &&
            std::abs(teleported.value.translation_velocity.z - expected_zero_velocity.z) <= velocity_tolerance,
            "Teleport zero-velocity policy must not generate an artificial velocity spike.");
    }

    void test_phase09_animation_channel_span_coefficients()
    {
        using skin_physics::asset::animation_key;
        using skin_physics::asset::animation_path;
        using skin_physics::asset::animation_track;
        using skin_physics::asset::interpolation;
        using skin_physics::physics::animation_channel_span_build_result;
        using skin_physics::physics::animation_channel_span_sample_result;

        skin_physics::physics::animation_blend_weight_curve_desc blend_curve;
        blend_curve.blend_start_time = 0.5;
        blend_curve.blend_end_time = 2.5;
        blend_curve.weight_start = 0.0f;
        blend_curve.weight_end = 1.0f;
        blend_curve.tangent_start = 0.0f;
        blend_curve.tangent_end = 0.0f;
        require(skin_physics::physics::valid_animation_blend_weight_curve(
                blend_curve),
            "A monotonic cubic Hermite Blend Weight Curve must validate over its complete domain.");

        const skin_physics::physics::animation_blend_weight_spans_build_result
            blend_schedule =
                skin_physics::physics::build_animation_blend_weight_spans(
                    blend_curve, 0.0, 3.0);
        require(blend_schedule.success &&
            blend_schedule.values.size() == 3u &&
            std::abs(blend_schedule.values[0u].world_time_start - 0.0) <= 1.0e-12 &&
            std::abs(blend_schedule.values[0u].world_time_duration - 0.5) <= 1.0e-12 &&
            std::abs(blend_schedule.values[1u].world_time_start - 0.5) <= 1.0e-12 &&
            std::abs(blend_schedule.values[1u].world_time_duration - 2.0) <= 1.0e-12 &&
            std::abs(blend_schedule.values[2u].world_time_start - 2.5) <= 1.0e-12 &&
            std::abs(blend_schedule.values[2u].world_time_duration - 0.5) <= 1.0e-12,
            "Blend Weight scheduling must split a Logical Tick at both blend boundaries.");

        const skin_physics::physics::animation_blend_weight_sample_result
            blend_before =
                skin_physics::physics::evaluate_animation_blend_weight_span(
                    blend_schedule.values[0u], 0.5);
        const skin_physics::physics::animation_blend_weight_sample_result
            blend_mid =
                skin_physics::physics::evaluate_animation_blend_weight_span(
                    blend_schedule.values[1u], 0.5);
        const skin_physics::physics::animation_blend_weight_sample_result
            blend_after =
                skin_physics::physics::evaluate_animation_blend_weight_span(
                    blend_schedule.values[2u], 0.5);
        require(blend_before.success && blend_mid.success && blend_after.success &&
            std::abs(blend_before.value.weight) <= 1.0e-12 &&
            std::abs(blend_before.value.world_time_derivative) <= 1.0e-12 &&
            std::abs(blend_mid.value.weight - 0.5) <= 1.0e-12 &&
            std::abs(blend_mid.value.world_time_derivative - 0.75) <= 1.0e-12 &&
            std::abs(blend_after.value.weight - 1.0) <= 1.0e-12 &&
            std::abs(blend_after.value.world_time_derivative) <= 1.0e-12,
            "Cubic Hermite Blend Weight spans must evaluate endpoint holds, interior Weight and analytic World-Time derivative.");

        const skin_physics::physics::animation_blend_weight_span_build_result
            partial_blend_span =
                skin_physics::physics::build_animation_blend_weight_span(
                    blend_curve, 1.0, 1.0);
        const skin_physics::physics::animation_blend_weight_sample_result
            partial_blend_sample =
                skin_physics::physics::evaluate_animation_blend_weight_span(
                    partial_blend_span.value, 0.5);
        require(partial_blend_span.success && partial_blend_sample.success &&
            std::abs(partial_blend_sample.value.weight - 0.5) <= 1.0e-12 &&
            std::abs(partial_blend_sample.value.world_time_derivative - 0.75) <= 1.0e-12,
            "A partial Blend Weight Span must reparameterize the original Hermite curve without changing its value or derivative.");

        const skin_physics::physics::animation_blend_weight_span_build_result
            crossing_blend_span =
                skin_physics::physics::build_animation_blend_weight_span(
                    blend_curve, 0.25, 1.0);
        require(!crossing_blend_span.success &&
            crossing_blend_span.error ==
                skin_physics::physics::animation_blend_weight_error::span_crosses_curve_boundary,
            "A Blend Weight Span crossing blend_start_time must be split by the scheduler.");

        skin_physics::physics::animation_blend_weight_curve_desc overshooting_blend_curve =
            blend_curve;
        overshooting_blend_curve.tangent_start = 10.0f;
        overshooting_blend_curve.tangent_end = 10.0f;
        require(!skin_physics::physics::valid_animation_blend_weight_curve(
                overshooting_blend_curve) &&
            !skin_physics::physics::build_animation_blend_weight_spans(
                overshooting_blend_curve, 0.0, 3.0).success,
            "A Hermite Blend Weight Curve whose interior extrema leave [0, 1] must be rejected before Span construction.");

        animation_track linear_translation;
        linear_translation.node_index = 3u;
        linear_translation.path = animation_path::translation;
        linear_translation.mode = interpolation::linear;
        linear_translation.component_count = 3u;
        animation_key linear_start;
        linear_start.time = 0.0f;
        linear_start.value = { 0.0f, 1.0f, 2.0f };
        animation_key linear_end;
        linear_end.time = 2.0f;
        linear_end.value = { 4.0f, 5.0f, 6.0f };
        linear_translation.keys = { linear_start, linear_end };

        const animation_channel_span_build_result linear_span =
            skin_physics::physics::build_animation_channel_span(
                linear_translation, 0.5, 1.0);
        require(linear_span.success &&
            linear_span.value.node_index == 3u &&
            linear_span.value.interpolation_mode == interpolation::linear,
            "LINEAR Animation Channel must build one immutable Key-bounded Span.");

        const animation_channel_span_sample_result linear_mid =
            skin_physics::physics::evaluate_animation_channel_span(
                linear_span.value, 0.5, 0.5);
        require(linear_mid.success &&
            std::abs(linear_mid.value.value.x - 2.0f) <= 1.0e-6f &&
            std::abs(linear_mid.value.value.y - 3.0f) <= 1.0e-6f &&
            std::abs(linear_mid.value.value.z - 4.0f) <= 1.0e-6f &&
            std::abs(linear_mid.value.world_time_derivative.x - 4.0f) <= 1.0e-6f &&
            std::abs(linear_mid.value.world_time_derivative.y - 4.0f) <= 1.0e-6f &&
            std::abs(linear_mid.value.world_time_derivative.z - 4.0f) <= 1.0e-6f,
            "LINEAR Span evaluation must return Position and analytic World-Time velocity.");

        animation_track reverse_translation = linear_translation;
        const animation_channel_span_build_result reverse_span =
            skin_physics::physics::build_animation_channel_span(
                reverse_translation, 1.5, -1.0);
        require(reverse_span.success,
            "A signed Clip-Time Delta must support reverse Playback inside one Key interval.");
        const animation_channel_span_sample_result reverse_mid =
            skin_physics::physics::evaluate_animation_channel_span(
                reverse_span.value, 0.5, 0.5);
        require(reverse_mid.success &&
            std::abs(reverse_mid.value.value.x - 2.0f) <= 1.0e-6f &&
            std::abs(reverse_mid.value.world_time_derivative.x + 4.0f) <= 1.0e-6f,
            "Reverse Span evaluation must reverse the analytic Target Velocity without reversing the world-time cursor.");

        animation_track cubic_translation;
        cubic_translation.node_index = 4u;
        cubic_translation.path = animation_path::translation;
        cubic_translation.mode = interpolation::cubic_spline;
        cubic_translation.component_count = 3u;
        animation_key cubic_start;
        cubic_start.time = 0.0f;
        cubic_start.in_tangent = { 0.0f, 0.0f, 0.0f };
        cubic_start.value = { 0.0f, 0.0f, 0.0f };
        cubic_start.out_tangent = { 0.0f, 0.0f, 0.0f };
        animation_key cubic_end;
        cubic_end.time = 2.0f;
        cubic_end.in_tangent = { 0.0f, 0.0f, 0.0f };
        cubic_end.value = { 1.0f, 2.0f, 3.0f };
        cubic_end.out_tangent = { 0.0f, 0.0f, 0.0f };
        cubic_translation.keys = { cubic_start, cubic_end };

        const animation_channel_span_build_result cubic_span =
            skin_physics::physics::build_animation_channel_span(
                cubic_translation, 0.5, 1.0);
        require(cubic_span.success &&
            cubic_span.value.interpolation_mode == interpolation::cubic_spline,
            "glTF CUBICSPLINE tangents must be converted into Span-local cubic coefficients.");
        const animation_channel_span_sample_result cubic_mid =
            skin_physics::physics::evaluate_animation_channel_span(
                cubic_span.value, 0.5, 0.5);
        require(cubic_mid.success &&
            std::abs(cubic_mid.value.value.x - 0.5f) <= 1.0e-6f &&
            std::abs(cubic_mid.value.value.y - 1.0f) <= 1.0e-6f &&
            std::abs(cubic_mid.value.value.z - 1.5f) <= 1.0e-6f &&
            std::abs(cubic_mid.value.world_time_derivative.x - 1.5f) <= 1.0e-5f &&
            std::abs(cubic_mid.value.world_time_derivative.y - 3.0f) <= 1.0e-5f &&
            std::abs(cubic_mid.value.world_time_derivative.z - 4.5f) <= 1.0e-5f,
            "CUBICSPLINE Span coefficients must preserve Hermite values and analytic derivatives after Clip-Time reparameterization.");

        animation_track step_translation;
        step_translation.node_index = 5u;
        step_translation.path = animation_path::translation;
        step_translation.mode = interpolation::step;
        step_translation.component_count = 3u;
        animation_key step_start;
        step_start.time = 0.0f;
        step_start.value = { 1.0f, 2.0f, 3.0f };
        animation_key step_end;
        step_end.time = 1.0f;
        step_end.value = { 5.0f, 6.0f, 7.0f };
        step_translation.keys = { step_start, step_end };
        const animation_channel_span_build_result step_span =
            skin_physics::physics::build_animation_channel_span(
                step_translation, 0.25, 0.5);
        require(step_span.success &&
            step_span.value.interpolation_mode == interpolation::step,
            "STEP Animation Channel must build a constant Span between Key boundaries.");
        const animation_channel_span_sample_result step_mid =
            skin_physics::physics::evaluate_animation_channel_span(
                step_span.value, 0.5, 0.5);
        require(step_mid.success &&
            step_mid.value.value.x == 1.0f &&
            step_mid.value.value.y == 2.0f &&
            step_mid.value.value.z == 3.0f &&
            step_mid.value.world_time_derivative.x == 0.0f &&
            step_mid.value.world_time_derivative.y == 0.0f &&
            step_mid.value.world_time_derivative.z == 0.0f,
            "STEP Span evaluation must produce a constant value and zero analytic velocity.");

        animation_track linear_rotation;
        linear_rotation.node_index = 6u;
        linear_rotation.path = animation_path::rotation;
        linear_rotation.mode = interpolation::linear;
        linear_rotation.component_count = 4u;
        animation_key rotation_start;
        rotation_start.time = 0.0f;
        rotation_start.value = { 0.0f, 0.0f, 0.0f, 1.0f };
        animation_key rotation_end;
        rotation_end.time = 2.0f;
        rotation_end.value = {
            0.0f, 0.0f,
            std::sin(3.14159265358979323846f / 4.0f),
            std::cos(3.14159265358979323846f / 4.0f)
        };
        linear_rotation.keys = { rotation_start, rotation_end };
        const animation_channel_span_build_result rotation_span =
            skin_physics::physics::build_animation_channel_span(
                linear_rotation, 0.5, 1.0);
        require(rotation_span.success,
            "LINEAR Quaternion Span must pre-evaluate its exact Clip-Time endpoints before Slerp.");
        const animation_channel_span_sample_result rotation_mid =
            skin_physics::physics::evaluate_animation_channel_span(
                rotation_span.value, 0.5, 0.5);
        const float quaternion_norm = std::sqrt(
            rotation_mid.value.value.x * rotation_mid.value.value.x +
            rotation_mid.value.value.y * rotation_mid.value.value.y +
            rotation_mid.value.value.z * rotation_mid.value.value.z +
            rotation_mid.value.value.w * rotation_mid.value.value.w);
        const float quaternion_derivative_dot =
            rotation_mid.value.value.x * rotation_mid.value.world_time_derivative.x +
            rotation_mid.value.value.y * rotation_mid.value.world_time_derivative.y +
            rotation_mid.value.value.z * rotation_mid.value.world_time_derivative.z +
            rotation_mid.value.value.w * rotation_mid.value.world_time_derivative.w;
        require(rotation_mid.success &&
            std::abs(rotation_mid.value.value.z -
                std::sin(3.14159265358979323846f / 8.0f)) <= 1.0e-5f &&
            std::abs(rotation_mid.value.value.w -
                std::cos(3.14159265358979323846f / 8.0f)) <= 1.0e-5f &&
            std::abs(quaternion_norm - 1.0f) <= 1.0e-5f &&
            std::abs(quaternion_derivative_dot) <= 1.0e-5f,
            "LINEAR Quaternion Span must use shortest-arc Slerp and its tangent-space analytic derivative.");

        const animation_channel_span_build_result paused_rotation_span =
            skin_physics::physics::build_animation_channel_span(
                linear_rotation, 1.0, 0.0);
        require(paused_rotation_span.success,
            "A paused LINEAR rotation Track must build a constant Span at its current Clip Time.");
        const animation_channel_span_sample_result paused_rotation =
            skin_physics::physics::evaluate_animation_channel_span(
                paused_rotation_span.value, 0.75, 0.25);
        require(paused_rotation.success &&
            std::abs(paused_rotation.value.value.z -
                std::sin(3.14159265358979323846f / 8.0f)) <= 1.0e-5f &&
            std::abs(paused_rotation.value.world_time_derivative.z) <= 1.0e-6f &&
            std::abs(paused_rotation.value.world_time_derivative.w) <= 1.0e-6f,
            "Paused LINEAR rotation must sample the current interpolated quaternion with zero velocity.");

        animation_track cubic_rotation;
        cubic_rotation.node_index = 7u;
        cubic_rotation.path = animation_path::rotation;
        cubic_rotation.mode = interpolation::cubic_spline;
        cubic_rotation.component_count = 4u;
        animation_key cubic_rotation_start;
        cubic_rotation_start.time = 0.0f;
        cubic_rotation_start.in_tangent = { 0.0f, 0.0f, 0.0f, 0.0f };
        cubic_rotation_start.value = { 0.0f, 0.0f, 0.0f, 1.0f };
        cubic_rotation_start.out_tangent = { 0.0f, 0.0f, 0.0f, 0.0f };
        animation_key cubic_rotation_end;
        cubic_rotation_end.time = 2.0f;
        cubic_rotation_end.in_tangent = { 0.0f, 0.0f, 0.0f, 0.0f };
        cubic_rotation_end.value = rotation_end.value;
        cubic_rotation_end.out_tangent = { 0.0f, 0.0f, 0.0f, 0.0f };
        cubic_rotation.keys = {
            cubic_rotation_start, cubic_rotation_end
        };
        const animation_channel_span_build_result cubic_rotation_span =
            skin_physics::physics::build_animation_channel_span(
                cubic_rotation, 0.5, 1.0);
        const animation_channel_span_sample_result cubic_rotation_mid =
            skin_physics::physics::evaluate_animation_channel_span(
                cubic_rotation_span.value, 0.5, 0.5);
        const float cubic_rotation_norm = std::sqrt(
            cubic_rotation_mid.value.value.x * cubic_rotation_mid.value.value.x +
            cubic_rotation_mid.value.value.y * cubic_rotation_mid.value.value.y +
            cubic_rotation_mid.value.value.z * cubic_rotation_mid.value.value.z +
            cubic_rotation_mid.value.value.w * cubic_rotation_mid.value.value.w);
        const float cubic_rotation_derivative_dot =
            cubic_rotation_mid.value.value.x *
                cubic_rotation_mid.value.world_time_derivative.x +
            cubic_rotation_mid.value.value.y *
                cubic_rotation_mid.value.world_time_derivative.y +
            cubic_rotation_mid.value.value.z *
                cubic_rotation_mid.value.world_time_derivative.z +
            cubic_rotation_mid.value.value.w *
                cubic_rotation_mid.value.world_time_derivative.w;
        require(cubic_rotation_span.success && cubic_rotation_mid.success &&
            std::abs(cubic_rotation_mid.value.value.z -
                std::sin(3.14159265358979323846f / 8.0f)) <= 1.0e-5f &&
            std::abs(cubic_rotation_mid.value.value.w -
                std::cos(3.14159265358979323846f / 8.0f)) <= 1.0e-5f &&
            std::abs(cubic_rotation_norm - 1.0f) <= 1.0e-5f &&
            std::abs(cubic_rotation_derivative_dot) <= 1.0e-5f,
            "CUBICSPLINE Quaternion Span must normalize its raw Hermite polynomial and project its analytic derivative onto the quaternion tangent space.");

        animation_track one_key_track;
        one_key_track.node_index = 8u;
        one_key_track.path = animation_path::translation;
        one_key_track.mode = interpolation::linear;
        one_key_track.component_count = 3u;
        animation_key only_key;
        only_key.time = 1.0f;
        only_key.value = { 7.0f, 8.0f, 9.0f };
        one_key_track.keys = { only_key };
        const animation_channel_span_build_result one_key_span =
            skin_physics::physics::build_animation_channel_span(
                one_key_track, -2.0, 5.0);
        const animation_channel_span_sample_result one_key_sample =
            skin_physics::physics::evaluate_animation_channel_span(
                one_key_span.value, 0.5, 0.5);
        require(one_key_span.success && one_key_sample.success &&
            one_key_sample.value.value.x == 7.0f &&
            one_key_sample.value.value.y == 8.0f &&
            one_key_sample.value.value.z == 9.0f &&
            one_key_sample.value.world_time_derivative.x == 0.0f &&
            one_key_sample.value.world_time_derivative.y == 0.0f &&
            one_key_sample.value.world_time_derivative.z == 0.0f,
            "A one-key Animation Track must remain a constant Span even when the requested Clip-Time range crosses its only key.");
        const skin_physics::physics::animation_channel_spans_build_result one_key_schedule =
            skin_physics::physics::build_animation_channel_spans(
                one_key_track, -2.0, 5.0);
        require(one_key_schedule.success && one_key_schedule.values.size() == 1u,
            "The Channel Span Scheduler must not manufacture a discontinuity at the only key of a constant Track.");

        animation_track key_bounded_track = linear_translation;
        animation_key middle_key;
        middle_key.time = 1.0f;
        middle_key.value = { 2.0f, 3.0f, 4.0f };
        key_bounded_track.keys.insert(
            key_bounded_track.keys.begin() + 1, middle_key);
        const animation_channel_span_build_result crossing_span =
            skin_physics::physics::build_animation_channel_span(
                key_bounded_track, 0.5, 1.0);
        require(!crossing_span.success &&
            crossing_span.error ==
                skin_physics::physics::animation_channel_span_error::span_crosses_key_boundary,
            "An Animation Span crossing an interior Key must be rejected for Scheduler subdivision.");

        const skin_physics::physics::animation_channel_spans_build_result split_forward =
            skin_physics::physics::build_animation_channel_spans(
                key_bounded_track, 0.5, 1.0);
        require(split_forward.success && split_forward.values.size() == 2u &&
            std::abs(split_forward.values[0u].clip_time_start - 0.5) <= 1.0e-12 &&
            std::abs(split_forward.values[0u].clip_time_delta - 0.5) <= 1.0e-12 &&
            std::abs(split_forward.values[1u].clip_time_start - 1.0) <= 1.0e-12 &&
            std::abs(split_forward.values[1u].clip_time_delta - 0.5) <= 1.0e-12,
            "Forward Channel Span Scheduler must split at each crossed Key and preserve ordered Clip-Time ranges.");
        const animation_channel_span_sample_result split_forward_left_end =
            skin_physics::physics::evaluate_animation_channel_span(
                split_forward.values[0u], 1.0, 0.25);
        const animation_channel_span_sample_result split_forward_right_start =
            skin_physics::physics::evaluate_animation_channel_span(
                split_forward.values[1u], 0.0, 0.25);
        require(split_forward_left_end.success && split_forward_right_start.success &&
            std::abs(split_forward_left_end.value.value.x -
                split_forward_right_start.value.value.x) <= 1.0e-6f,
            "Adjacent LINEAR Span records must evaluate to the same value at their shared Key boundary.");

        const skin_physics::physics::animation_channel_spans_build_result split_reverse =
            skin_physics::physics::build_animation_channel_spans(
                key_bounded_track, 1.5, -1.0);
        require(split_reverse.success && split_reverse.values.size() == 2u &&
            std::abs(split_reverse.values[0u].clip_time_start - 1.5) <= 1.0e-12 &&
            std::abs(split_reverse.values[0u].clip_time_delta + 0.5) <= 1.0e-12 &&
            std::abs(split_reverse.values[1u].clip_time_start - 1.0) <= 1.0e-12 &&
            std::abs(split_reverse.values[1u].clip_time_delta + 0.5) <= 1.0e-12,
            "Reverse Channel Span Scheduler must traverse Key boundaries in descending Clip-Time order.");
        const animation_channel_span_sample_result split_reverse_left_end =
            skin_physics::physics::evaluate_animation_channel_span(
                split_reverse.values[0u], 1.0, 0.25);
        const animation_channel_span_sample_result split_reverse_right_start =
            skin_physics::physics::evaluate_animation_channel_span(
                split_reverse.values[1u], 0.0, 0.25);
        require(split_reverse_left_end.success && split_reverse_right_start.success &&
            std::abs(split_reverse_left_end.value.value.x -
                split_reverse_right_start.value.value.x) <= 1.0e-6f,
            "Reverse LINEAR Span records must also meet at the same Key-boundary value.");

        const skin_physics::physics::animation_channel_spans_build_result paused_schedule =
            skin_physics::physics::build_animation_channel_spans(
                key_bounded_track, 1.0, 0.0);
        require(paused_schedule.success && paused_schedule.values.size() == 1u &&
            paused_schedule.values[0u].clip_time_delta == 0.0,
            "A paused Track must produce one immutable zero-Clip-Time Span rather than an empty schedule.");

        animation_track step_bounded_track = key_bounded_track;
        step_bounded_track.mode = interpolation::step;
        const skin_physics::physics::animation_channel_spans_build_result
            forward_step_schedule =
                skin_physics::physics::build_animation_channel_spans(
                    step_bounded_track, 0.5, 1.0);
        require(forward_step_schedule.success &&
            forward_step_schedule.values.size() == 2u,
            "Forward STEP playback must produce one Span on each side of a discontinuous Key.");
        const animation_channel_span_sample_result
            forward_step_before_boundary =
                skin_physics::physics::evaluate_animation_channel_span(
                    forward_step_schedule.values[0u], 1.0, 0.25);
        const animation_channel_span_sample_result
            forward_step_after_boundary =
                skin_physics::physics::evaluate_animation_channel_span(
                    forward_step_schedule.values[1u], 0.0, 0.25);
        require(forward_step_before_boundary.success &&
            forward_step_after_boundary.success &&
            forward_step_before_boundary.value.value.x == 0.0f &&
            forward_step_after_boundary.value.value.x == 2.0f,
            "Forward STEP spans must preserve the pre-Key and post-Key one-sided values across a discontinuity.");

        const skin_physics::physics::animation_channel_spans_build_result
            reverse_step_schedule =
                skin_physics::physics::build_animation_channel_spans(
                    step_bounded_track, 1.5, -1.0);
        require(reverse_step_schedule.success &&
            reverse_step_schedule.values.size() == 2u,
            "Reverse STEP playback must produce one Span on each side of a discontinuous Key.");
        const animation_channel_span_sample_result
            reverse_step_before_boundary =
                skin_physics::physics::evaluate_animation_channel_span(
                    reverse_step_schedule.values[0u], 1.0, 0.25);
        const animation_channel_span_sample_result
            reverse_step_after_boundary =
                skin_physics::physics::evaluate_animation_channel_span(
                    reverse_step_schedule.values[1u], 0.0, 0.25);
        require(reverse_step_before_boundary.success &&
            reverse_step_after_boundary.success &&
            reverse_step_before_boundary.value.value.x == 2.0f &&
            reverse_step_after_boundary.value.value.x == 0.0f,
            "Reverse STEP spans must preserve descending-time one-sided values across a discontinuity.");

        const skin_physics::physics::animation_channel_span_schedule_result
            forward_world_schedule =
                skin_physics::physics::schedule_animation_channel_spans(
                    key_bounded_track, 0.5, 1.0, 10.0, 0.4);
        require(forward_world_schedule.success &&
            forward_world_schedule.values.size() == 2u &&
            std::abs(forward_world_schedule.values[0u].world_time_start - 10.0) <= 1.0e-12 &&
            std::abs(forward_world_schedule.values[0u].world_time_duration - 0.2) <= 1.0e-12 &&
            std::abs(forward_world_schedule.values[1u].world_time_start - 10.2) <= 1.0e-12 &&
            std::abs(forward_world_schedule.values[1u].world_time_duration - 0.2) <= 1.0e-12,
            "Forward Clip-Time spans must map onto positive ordered World-Time partitions within the Logical Tick.");

        const skin_physics::physics::animation_channel_span_schedule_result
            reverse_world_schedule =
                skin_physics::physics::schedule_animation_channel_spans(
                    key_bounded_track, 1.5, -1.0, 3.0, 0.8);
        require(reverse_world_schedule.success &&
            reverse_world_schedule.values.size() == 2u &&
            std::abs(reverse_world_schedule.values[0u].world_time_start - 3.0) <= 1.0e-12 &&
            std::abs(reverse_world_schedule.values[0u].world_time_duration - 0.4) <= 1.0e-12 &&
            std::abs(reverse_world_schedule.values[1u].world_time_start - 3.4) <= 1.0e-12 &&
            std::abs(reverse_world_schedule.values[1u].world_time_duration - 0.4) <= 1.0e-12,
            "Reverse Clip-Time spans must retain forward-monotonic World-Time ordering and positive interval durations.");

        const skin_physics::physics::animation_channel_span_schedule_result
            paused_world_schedule =
                skin_physics::physics::schedule_animation_channel_spans(
                    key_bounded_track, 1.0, 0.0, 7.0, 0.5);
        require(paused_world_schedule.success &&
            paused_world_schedule.values.size() == 1u &&
            std::abs(paused_world_schedule.values[0u].world_time_start - 7.0) <= 1.0e-12 &&
            std::abs(paused_world_schedule.values[0u].world_time_duration - 0.5) <= 1.0e-12,
            "A paused Track must retain the full Logical Tick World-Time interval.");

        const skin_physics::physics::animation_channel_span_schedule_result
            unrepresentable_clip_schedule =
                skin_physics::physics::schedule_animation_channel_spans(
                    key_bounded_track, 1.0e16, 1.0, 0.0, 0.1);
        require(!unrepresentable_clip_schedule.success &&
            unrepresentable_clip_schedule.values.empty() &&
            unrepresentable_clip_schedule.error ==
                skin_physics::physics::animation_channel_span_error::invalid_time,
            "A non-zero Clip-Time delta smaller than the double-precision ULP at its start must fail closed.");

        const skin_physics::physics::animation_channel_span_schedule_result
            invalid_world_schedule =
                skin_physics::physics::schedule_animation_channel_spans(
                    key_bounded_track, 0.5, 1.0, 1.0e16, 0.1);
        require(!invalid_world_schedule.success &&
            invalid_world_schedule.values.empty() &&
            invalid_world_schedule.error ==
                skin_physics::physics::animation_channel_span_error::invalid_time,
            "A World-Time Tick interval that cannot advance at its absolute start must fail closed.");

        animation_track duplicate_key_track = key_bounded_track;
        duplicate_key_track.keys[1u].time =
            duplicate_key_track.keys[0u].time;
        const skin_physics::physics::animation_channel_spans_build_result invalid_schedule =
            skin_physics::physics::build_animation_channel_spans(
                duplicate_key_track, 0.25, 0.5);
        require(!invalid_schedule.success && invalid_schedule.values.empty() &&
            invalid_schedule.error ==
                skin_physics::physics::animation_channel_span_error::non_monotonic_time,
            "The Channel Span Scheduler must fail atomically on duplicate Key times.");

        skin_physics::asset::animation_clip synchronized_clip;
        synchronized_clip.name = "synchronized_span_test";
        synchronized_clip.duration = 2.0f;
        animation_track synchronized_translation = key_bounded_track;
        synchronized_translation.node_index = 3u;
        animation_track synchronized_scale;
        synchronized_scale.node_index = 1u;
        synchronized_scale.path = animation_path::scale;
        synchronized_scale.mode = interpolation::step;
        synchronized_scale.component_count = 3u;
        animation_key scale_key0;
        scale_key0.time = 0.0f;
        scale_key0.value = { 1.0f, 1.0f, 1.0f };
        animation_key scale_key1;
        scale_key1.time = 0.5f;
        scale_key1.value = { 2.0f, 2.0f, 2.0f };
        animation_key scale_key2;
        scale_key2.time = 2.0f;
        scale_key2.value = { 1.5f, 1.5f, 1.5f };
        synchronized_scale.keys = { scale_key0, scale_key1, scale_key2 };
        animation_track synchronized_rotation = linear_rotation;
        synchronized_rotation.node_index = 2u;
        animation_key rotation_middle;
        rotation_middle.time = 1.5f;
        rotation_middle.value = rotation_end.value;
        synchronized_rotation.keys = {
            rotation_start, rotation_middle, rotation_end
        };
        animation_track synchronized_constant_channel;
        synchronized_constant_channel.node_index = 4u;
        synchronized_constant_channel.path = animation_path::scale;
        synchronized_constant_channel.mode = interpolation::linear;
        synchronized_constant_channel.component_count = 3u;
        animation_key constant_channel_key;
        constant_channel_key.time = 1.25f;
        constant_channel_key.value = { 4.0f, 4.0f, 4.0f };
        synchronized_constant_channel.keys = { constant_channel_key };
        synchronized_clip.tracks = {
            synchronized_translation,
            synchronized_scale,
            synchronized_rotation,
            synchronized_constant_channel
        };

        const skin_physics::physics::animation_clip_spans_build_result clip_spans =
            skin_physics::physics::build_animation_clip_spans(
                synchronized_clip, 5u, 0.25, 1.5);
        require(clip_spans.success && clip_spans.values.size() == 4u,
            "An Animation Clip Span Scheduler must merge Key boundaries from all Tracks, not schedule each Node / Path independently.");
        const double expected_clip_boundaries[5] = {
            0.25, 0.5, 1.0, 1.5, 1.75
        };
        for (std::size_t span_index = 0u;
            span_index < clip_spans.values.size(); ++span_index)
        {
            const skin_physics::physics::animation_clip_span& clip_span =
                clip_spans.values[span_index];
            require(clip_span.span_index == span_index &&
                clip_span.channels.size() == synchronized_clip.tracks.size() &&
                std::abs(clip_span.clip_time_start -
                    expected_clip_boundaries[span_index]) <= 1.0e-12 &&
                std::abs(clip_span.clip_time_start + clip_span.clip_time_delta -
                    expected_clip_boundaries[span_index + 1u]) <= 1.0e-12,
                "Every Clip Span must retain one globally merged interval and all original Asset Tracks.");
            for (std::size_t track_index = 0u;
                track_index < clip_span.channels.size(); ++track_index)
            {
                const skin_physics::physics::animation_channel_span& channel =
                    clip_span.channels[track_index];
                require(channel.track_index == track_index &&
                    channel.node_index ==
                        synchronized_clip.tracks[track_index].node_index &&
                    channel.path == synchronized_clip.tracks[track_index].path &&
                    channel.clip_time_start == clip_span.clip_time_start &&
                    channel.clip_time_delta == clip_span.clip_time_delta,
                    "All Node / Path channel records in a Clip Span must share the same Clip-Time interval and preserve original Track ordering.");
            }
        }

        const skin_physics::physics::animation_clip_spans_build_result reverse_clip_spans =
            skin_physics::physics::build_animation_clip_spans(
                synchronized_clip, 5u, 1.75, -1.5);
        require(reverse_clip_spans.success &&
            reverse_clip_spans.values.size() == 4u &&
            std::abs(reverse_clip_spans.values[0u].clip_time_start - 1.75) <= 1.0e-12 &&
            std::abs(reverse_clip_spans.values[0u].clip_time_delta + 0.25) <= 1.0e-12 &&
            std::abs(reverse_clip_spans.values[1u].clip_time_start - 1.5) <= 1.0e-12 &&
            std::abs(reverse_clip_spans.values[2u].clip_time_start - 1.0) <= 1.0e-12 &&
            std::abs(reverse_clip_spans.values[3u].clip_time_start - 0.5) <= 1.0e-12,
            "Multi-Track Clip Span scheduling must merge all Key times in descending order for reverse Playback.");

        const skin_physics::physics::animation_clip_span_schedule_result
            forward_loop_spans =
                skin_physics::physics::schedule_animation_clip_playback_spans(
                    synchronized_clip, 5u, 1.75, 1.0, true, 30.0, 0.5);
        require(forward_loop_spans.success &&
            forward_loop_spans.values.size() == 2u &&
            forward_loop_spans.values[0u].clip.span_index == 0u &&
            forward_loop_spans.values[1u].clip.span_index == 1u &&
            std::abs(forward_loop_spans.values[0u].clip.clip_time_start - 1.75) <= 1.0e-12 &&
            std::abs(forward_loop_spans.values[0u].clip.clip_time_delta - 0.25) <= 1.0e-12 &&
            std::abs(forward_loop_spans.values[0u].world_time_start - 30.0) <= 1.0e-12 &&
            std::abs(forward_loop_spans.values[0u].world_time_duration - 0.25) <= 1.0e-12 &&
            std::abs(forward_loop_spans.values[1u].clip.clip_time_start) <= 1.0e-12 &&
            std::abs(forward_loop_spans.values[1u].clip.clip_time_delta - 0.25) <= 1.0e-12 &&
            std::abs(forward_loop_spans.values[1u].world_time_start - 30.25) <= 1.0e-12 &&
            std::abs(forward_loop_spans.values[1u].world_time_duration - 0.25) <= 1.0e-12,
            "Forward Loop Playback must split at Clip duration and resume at zero in the next World-Time Span.");

        const skin_physics::physics::animation_clip_span_schedule_result
            reverse_loop_spans =
                skin_physics::physics::schedule_animation_clip_playback_spans(
                    synchronized_clip, 5u, 0.25, -1.0, true, 40.0, 0.5);
        require(reverse_loop_spans.success &&
            reverse_loop_spans.values.size() == 2u &&
            std::abs(reverse_loop_spans.values[0u].clip.clip_time_start - 0.25) <= 1.0e-12 &&
            std::abs(reverse_loop_spans.values[0u].clip.clip_time_delta + 0.25) <= 1.0e-12 &&
            std::abs(reverse_loop_spans.values[1u].clip.clip_time_start - 2.0) <= 1.0e-12 &&
            std::abs(reverse_loop_spans.values[1u].clip.clip_time_delta + 0.25) <= 1.0e-12 &&
            std::abs(reverse_loop_spans.values[1u].world_time_start - 40.25) <= 1.0e-12,
            "Reverse Loop Playback must split at zero and resume from Clip duration without reversing World-Time order.");

        const skin_physics::physics::animation_clip_span_schedule_result
            non_loop_clamp_spans =
                skin_physics::physics::schedule_animation_clip_playback_spans(
                    synchronized_clip, 5u, 1.75, 1.0, false, 50.0, 0.5);
        require(non_loop_clamp_spans.success &&
            non_loop_clamp_spans.values.size() == 2u &&
            std::abs(non_loop_clamp_spans.values[0u].clip.clip_time_delta - 0.25) <= 1.0e-12 &&
            std::abs(non_loop_clamp_spans.values[0u].world_time_duration - 0.25) <= 1.0e-12 &&
            std::abs(non_loop_clamp_spans.values[1u].clip.clip_time_start - 2.0) <= 1.0e-12 &&
            non_loop_clamp_spans.values[1u].clip.clip_time_delta == 0.0 &&
            std::abs(non_loop_clamp_spans.values[1u].world_time_start - 50.25) <= 1.0e-12 &&
            std::abs(non_loop_clamp_spans.values[1u].world_time_duration - 0.25) <= 1.0e-12,
            "Non-looping Playback must hold the terminal Clip Pose for the unconsumed World-Time interval.");

        const skin_physics::physics::animation_clip_span_schedule_result
            reverse_non_loop_clamp_spans =
                skin_physics::physics::schedule_animation_clip_playback_spans(
                    synchronized_clip, 5u, 0.25, -1.0, false, 60.0, 0.5);
        require(reverse_non_loop_clamp_spans.success &&
            reverse_non_loop_clamp_spans.values.size() == 2u &&
            std::abs(reverse_non_loop_clamp_spans.values[0u].clip.clip_time_delta + 0.25) <= 1.0e-12 &&
            reverse_non_loop_clamp_spans.values[1u].clip.clip_time_start == 0.0 &&
            reverse_non_loop_clamp_spans.values[1u].clip.clip_time_delta == 0.0 &&
            std::abs(reverse_non_loop_clamp_spans.values[1u].world_time_start - 60.25) <= 1.0e-12,
            "Reverse non-looping Playback must hold the first Clip Pose after reaching zero.");

        const skin_physics::physics::animation_clip_span_schedule_result
            paused_playback_spans =
                skin_physics::physics::schedule_animation_clip_playback_spans(
                    synchronized_clip, 5u, 0.75, 0.0, true, 70.0, 0.4);
        require(paused_playback_spans.success &&
            paused_playback_spans.values.size() == 1u &&
            std::abs(paused_playback_spans.values[0u].clip.clip_time_start - 0.75) <= 1.0e-12 &&
            paused_playback_spans.values[0u].clip.clip_time_delta == 0.0 &&
            std::abs(paused_playback_spans.values[0u].world_time_duration - 0.4) <= 1.0e-12,
            "Paused Playback must preserve one zero-Clip-Time Span across the entire Logical Tick.");

        const skin_physics::physics::animation_clip_span_schedule_result
            exact_forward_wrap_boundary =
                skin_physics::physics::schedule_animation_clip_playback_spans(
                    synchronized_clip, 5u, 1.75, 1.0, true, 80.0, 0.25);
        require(exact_forward_wrap_boundary.success &&
            exact_forward_wrap_boundary.values.size() == 1u &&
            exact_forward_wrap_boundary.values[0u].clip.clip_time_delta == 0.25,
            "A Tick ending exactly on a Loop boundary must not emit an empty post-wrap Span.");

        const skin_physics::physics::animation_clip_span_schedule_result
            playback_span_overflow =
                skin_physics::physics::schedule_animation_clip_playback_spans(
                    synchronized_clip, 5u, 1.75, 1.0, true, 90.0, 5.0, 2u);
        require(!playback_span_overflow.success &&
            playback_span_overflow.values.empty() &&
            playback_span_overflow.error ==
                skin_physics::physics::animation_channel_span_error::capacity_exceeded,
            "Pathological Loop Playback must fail atomically when the immutable Span count exceeds its configured limit.");

        const skin_physics::physics::animation_clip_span_schedule_result
            invalid_playback_start =
                skin_physics::physics::schedule_animation_clip_playback_spans(
                    synchronized_clip, 5u, 2.1, 1.0, false, 100.0, 0.5);
        require(!invalid_playback_start.success &&
            invalid_playback_start.error ==
                skin_physics::physics::animation_channel_span_error::invalid_time &&
            invalid_playback_start.values.empty(),
            "Playback scheduling must reject a Clip cursor outside the Asset Clip duration.");

        skin_physics::physics::animation_blend_weight_curve_desc synchronized_blend_curve;
        synchronized_blend_curve.blend_start_time = 10.1;
        synchronized_blend_curve.blend_end_time = 10.4;
        synchronized_blend_curve.weight_start = 0.0f;
        synchronized_blend_curve.weight_end = 1.0f;
        synchronized_blend_curve.tangent_start = 0.0f;
        synchronized_blend_curve.tangent_end = 0.0f;

        const skin_physics::physics::animation_blend_clip_span_schedule_result
            synchronized_blend_spans =
                skin_physics::physics::schedule_animation_blend_clip_spans(
                    synchronized_clip, 5u, 0.75, 1.0, false,
                    synchronized_clip, 5u, 1.75, -1.5, false,
                    synchronized_blend_curve, 10.0, 0.5);
        require(synchronized_blend_spans.success &&
            synchronized_blend_spans.values.size() == 5u,
            std::string(
                "Source / Target Key boundaries and Blend Start / End boundaries must merge into one shared World-Time partition. error=") +
                std::to_string(static_cast<int>(synchronized_blend_spans.error)) +
                " count=" +
                std::to_string(synchronized_blend_spans.values.size()) +
                " diagnostic=" + synchronized_blend_spans.message);
        for (std::size_t span_index = 0u;
            span_index < synchronized_blend_spans.values.size();
            ++span_index)
        {
            const skin_physics::physics::animation_blend_clip_span& span =
                synchronized_blend_spans.values[span_index];
            require(span.span_index == span_index &&
                span.source.clip.span_index == span_index &&
                span.target.clip.span_index == span_index &&
                span.source.world_time_start == span.world_time_start &&
                span.target.world_time_start == span.world_time_start &&
                span.blend_weight.world_time_start == span.world_time_start &&
                span.source.world_time_duration == span.world_time_duration &&
                span.target.world_time_duration == span.world_time_duration &&
                span.blend_weight.world_time_duration == span.world_time_duration &&
                span.source.clip.channels.size() == synchronized_clip.tracks.size() &&
                span.target.clip.channels.size() == synchronized_clip.tracks.size(),
                "A synchronized Blend Span must retain one World-Time interval for Source, Target and Weight while preserving every channel.");
            for (std::size_t channel_index = 0u;
                channel_index < synchronized_clip.tracks.size();
                ++channel_index)
            {
                const skin_physics::physics::animation_channel_span& source_channel =
                    span.source.clip.channels[channel_index];
                const skin_physics::physics::animation_channel_span& target_channel =
                    span.target.clip.channels[channel_index];
                require(source_channel.track_index == channel_index &&
                    target_channel.track_index == channel_index &&
                    source_channel.clip_time_start ==
                        span.source.clip.clip_time_start &&
                    source_channel.clip_time_delta ==
                        span.source.clip.clip_time_delta &&
                    target_channel.clip_time_start ==
                        span.target.clip.clip_time_start &&
                    target_channel.clip_time_delta ==
                        span.target.clip.clip_time_delta,
                    "Source / Target channels must preserve Track order and each synchronized Clip's own signed Clip-Time interval.");
            }

            const skin_physics::physics::animation_blend_weight_sample_result weight_sample =
                skin_physics::physics::evaluate_animation_blend_weight_span(
                    span.blend_weight, 0.5);
            require(weight_sample.success &&
                weight_sample.value.weight >= 0.0 &&
                weight_sample.value.weight <= 1.0 &&
                std::isfinite(weight_sample.value.world_time_derivative),
                "Every merged Blend Weight Span must retain a finite in-range Weight and analytic World-Time derivative.");
        }
        require(
            std::abs(
                synchronized_blend_spans.values.front().source.clip.clip_time_start -
                0.75) <= 1.0e-12 &&
            std::abs(
                synchronized_blend_spans.values.front().source.clip.clip_time_delta -
                0.1) <= 1.0e-12 &&
            std::abs(
                synchronized_blend_spans.values.front().target.clip.clip_time_start -
                1.75) <= 1.0e-12 &&
            std::abs(
                synchronized_blend_spans.values.front().target.clip.clip_time_delta +
                0.15) <= 1.0e-12,
            "Merged Blend intervals must preserve independent Source / Target Playback Speeds when rebuilding Clip-Time coefficients.");

        const skin_physics::physics::animation_blend_clip_span_schedule_result
            blend_span_overflow =
                skin_physics::physics::schedule_animation_blend_clip_spans(
                    synchronized_clip, 5u, 0.75, 1.0, false,
                    synchronized_clip, 5u, 1.75, -1.5, false,
                    synchronized_blend_curve, 10.0, 0.5, 2u);
        require(!blend_span_overflow.success &&
            blend_span_overflow.values.empty() &&
            blend_span_overflow.error ==
                skin_physics::physics::animation_blend_clip_span_error::capacity_exceeded,
            "Synchronized Blend scheduling must fail atomically when merged Source / Target / Weight boundaries exceed the Span limit.");

        skin_physics::physics::animation_blend_weight_curve_desc invalid_synchronized_blend =
            synchronized_blend_curve;
        invalid_synchronized_blend.tangent_start = 100.0f;
        invalid_synchronized_blend.tangent_end = 100.0f;
        const skin_physics::physics::animation_blend_clip_span_schedule_result
            invalid_blend_schedule =
                skin_physics::physics::schedule_animation_blend_clip_spans(
                    synchronized_clip, 5u, 0.75, 1.0, false,
                    synchronized_clip, 5u, 1.75, -1.5, false,
                    invalid_synchronized_blend, 10.0, 0.5);
        require(!invalid_blend_schedule.success &&
            invalid_blend_schedule.values.empty() &&
            invalid_blend_schedule.error ==
                skin_physics::physics::animation_blend_clip_span_error::invalid_blend_curve,
            "An invalid Hermite Blend Curve must reject the complete synchronized Blend schedule before publishing partial spans.");

        const skin_physics::physics::animation_clip_span_schedule_result
            scheduled_clip_spans =
                skin_physics::physics::schedule_animation_clip_spans(
                    synchronized_clip, 5u, 0.25, 1.5, 10.0, 0.6);
        require(scheduled_clip_spans.success &&
            scheduled_clip_spans.values.size() == 4u,
            "Synchronized multi-Track Clip spans must also be mapped into World-Time intervals.");
        const double expected_clip_world_starts[4] = {
            10.0, 10.1, 10.3, 10.5
        };
        const double expected_clip_world_durations[4] = {
            0.1, 0.2, 0.2, 0.1
        };
        for (std::size_t index = 0u;
            index < scheduled_clip_spans.values.size(); ++index)
        {
            require(std::abs(
                    scheduled_clip_spans.values[index].world_time_start -
                    expected_clip_world_starts[index]) <= 1.0e-12 &&
                std::abs(
                    scheduled_clip_spans.values[index].world_time_duration -
                    expected_clip_world_durations[index]) <= 1.0e-12 &&
                scheduled_clip_spans.values[index].clip.channels.size() ==
                    synchronized_clip.tracks.size(),
                "Forward synchronized Clip spans must partition the Logical Tick without losing channel records.");
        }

        const skin_physics::physics::animation_clip_span_schedule_result
            scheduled_reverse_clip_spans =
                skin_physics::physics::schedule_animation_clip_spans(
                    synchronized_clip, 5u, 1.75, -1.5, 20.0, 0.9);
        require(scheduled_reverse_clip_spans.success &&
            scheduled_reverse_clip_spans.values.size() == 4u,
            "Reverse synchronized Clip spans must retain their World-Time schedule.");
        const double expected_reverse_world_starts[4] = {
            20.0, 20.15, 20.45, 20.75
        };
        const double expected_reverse_world_durations[4] = {
            0.15, 0.3, 0.3, 0.15
        };
        for (std::size_t index = 0u;
            index < scheduled_reverse_clip_spans.values.size(); ++index)
        {
            require(std::abs(
                    scheduled_reverse_clip_spans.values[index].world_time_start -
                    expected_reverse_world_starts[index]) <= 1.0e-12 &&
                std::abs(
                    scheduled_reverse_clip_spans.values[index].world_time_duration -
                    expected_reverse_world_durations[index]) <= 1.0e-12,
                "Reverse Clip-Time traversal must map into monotonic positive World-Time partitions.");
        }

        const skin_physics::physics::animation_clip_span_schedule_result
            scheduled_paused_clip =
                skin_physics::physics::schedule_animation_clip_spans(
                    synchronized_clip, 5u, 1.0, 0.0, 30.0, 0.4);
        require(scheduled_paused_clip.success &&
            scheduled_paused_clip.values.size() == 1u &&
            scheduled_paused_clip.values[0u].clip.channels.size() ==
                synchronized_clip.tracks.size() &&
            scheduled_paused_clip.values[0u].clip.clip_time_delta == 0.0 &&
            std::abs(scheduled_paused_clip.values[0u].world_time_start - 30.0) <= 1.0e-12 &&
            std::abs(scheduled_paused_clip.values[0u].world_time_duration - 0.4) <= 1.0e-12,
            "Paused multi-Track Clips must retain all channels across the complete Logical Tick.");

        const skin_physics::physics::animation_clip_spans_build_result
            unrepresentable_clip_time =
                skin_physics::physics::build_animation_clip_spans(
                    synchronized_clip, 5u, 1.0e16, 1.0);
        require(!unrepresentable_clip_time.success &&
            unrepresentable_clip_time.error ==
                skin_physics::physics::animation_channel_span_error::invalid_time,
            "A non-zero multi-Track Clip delta below double-precision resolution must fail closed.");

        const skin_physics::physics::animation_clip_spans_build_result clip_out_of_range =
            skin_physics::physics::build_animation_clip_spans(
                synchronized_clip, 4u, 1.75, 0.5);
        require(!clip_out_of_range.success &&
            clip_out_of_range.error ==
                skin_physics::physics::animation_channel_span_error::invalid_time,
            "A Clip Span must be split at loop-wrap boundaries before a scheduler accepts a range outside the Clip duration.");

        const skin_physics::physics::animation_clip_spans_build_result invalid_clip_node =
            skin_physics::physics::build_animation_clip_spans(
                synchronized_clip, 3u, 0.25, 0.5);
        require(!invalid_clip_node.success &&
            invalid_clip_node.error ==
                skin_physics::physics::animation_channel_span_error::invalid_track,
            "Multi-Track Clip scheduling must reject a Track Node Index outside the uploaded Model.");

        skin_physics::asset::animation_clip duplicate_channel_clip = synchronized_clip;
        duplicate_channel_clip.tracks.push_back(synchronized_translation);
        const skin_physics::physics::animation_clip_spans_build_result duplicate_channel =
            skin_physics::physics::build_animation_clip_spans(
                duplicate_channel_clip, 4u, 0.25, 0.5);
        require(!duplicate_channel.success &&
            duplicate_channel.values.empty() &&
            duplicate_channel.error ==
                skin_physics::physics::animation_channel_span_error::invalid_track,
            "An Animation Clip must not publish ambiguous duplicate Node / Path channels.");

        const animation_channel_span_build_result invalid_world_duration_span =
            skin_physics::physics::build_animation_channel_span(
                linear_translation, 0.5, 1.0);
        const animation_channel_span_sample_result invalid_world_duration =
            skin_physics::physics::evaluate_animation_channel_span(
                invalid_world_duration_span.value, 0.5, 0.0);
        require(!invalid_world_duration.success &&
            invalid_world_duration.error ==
                skin_physics::physics::animation_channel_span_error::invalid_time,
            "Span derivative evaluation must reject a zero World-Time duration.");

        animation_track malformed_cubic = cubic_translation;
        malformed_cubic.keys[1].in_tangent.pop_back();
        const animation_channel_span_build_result malformed_span =
            skin_physics::physics::build_animation_channel_span(
                malformed_cubic, 0.5, 0.5);
        require(!malformed_span.success &&
            malformed_span.error ==
                skin_physics::physics::animation_channel_span_error::malformed_key,
            "CUBICSPLINE Span construction must reject a tangent array with the wrong component count.");
    }

    void test_phase09_animation_node_hierarchy_and_skin_matrix_derivatives()
    {
        using skin_physics::asset::animation_key;
        using skin_physics::asset::animation_path;
        using skin_physics::asset::animation_track;
        using skin_physics::asset::mat4;
        using skin_physics::asset::model;
        using skin_physics::asset::node;
        using skin_physics::asset::skin;
        using skin_physics::asset::interpolation;

        model source_model;
        source_model.meshes.resize(1u);
        source_model.nodes.resize(3u);

        node& root_node = source_model.nodes[0u];
        root_node.name = "animated_root";
        root_node.children = { 1u, 2u };

        node& joint_node = source_model.nodes[1u];
        joint_node.name = "animated_joint";
        joint_node.parent_index = 0u;
        joint_node.translation = { 0.0f, 1.0f, 0.0f };

        node& mesh_node = source_model.nodes[2u];
        mesh_node.name = "static_mesh_node";
        mesh_node.parent_index = 0u;
        mesh_node.mesh_index = 0u;
        mesh_node.skin_index = 0u;
        mesh_node.uses_matrix = true;
        mesh_node.local_matrix = mat4{};
        mesh_node.local_matrix.value[14u] = 2.0f;

        skin source_skin;
        source_skin.name = "single_joint_skin";
        source_skin.skeleton_node_index = 1u;
        source_skin.joints = { 1u };
        source_skin.inverse_bind_matrices = { mat4{} };
        source_model.skins.push_back(source_skin);

        skin_physics::asset::animation_clip clip;
        clip.name = "hierarchy_translation";
        clip.duration = 2.0f;

        animation_track root_translation;
        root_translation.node_index = 0u;
        root_translation.path = animation_path::translation;
        root_translation.mode = interpolation::linear;
        root_translation.component_count = 3u;
        animation_key root_key_start;
        root_key_start.time = 0.0f;
        root_key_start.value = { 0.0f, 0.0f, 0.0f };
        animation_key root_key_end;
        root_key_end.time = 2.0f;
        root_key_end.value = { 2.0f, 0.0f, 0.0f };
        root_translation.keys = { root_key_start, root_key_end };

        animation_track joint_translation;
        joint_translation.node_index = 1u;
        joint_translation.path = animation_path::translation;
        joint_translation.mode = interpolation::linear;
        joint_translation.component_count = 3u;
        animation_key joint_key_start;
        joint_key_start.time = 0.0f;
        joint_key_start.value = { 0.0f, 1.0f, 0.0f };
        animation_key joint_key_end;
        joint_key_end.time = 2.0f;
        joint_key_end.value = { 0.0f, 3.0f, 0.0f };
        joint_translation.keys = { joint_key_start, joint_key_end };
        clip.tracks = { root_translation, joint_translation };

        const skin_physics::physics::animation_clip_spans_build_result clip_spans =
            skin_physics::physics::build_animation_clip_spans(
                clip, static_cast<uint32_t>(source_model.nodes.size()), 0.0, 2.0);
        require(clip_spans.success && clip_spans.values.size() == 1u,
            "Hierarchy regression must build one synchronized Clip Span for the animated Root and Joint.");

        const skin_physics::physics::animation_hierarchy_sample_result evaluated =
            skin_physics::physics::evaluate_animation_clip_hierarchy(
                source_model, clip, clip_spans.values[0u], 0.5, 1.0);
        require(evaluated.success &&
            evaluated.value.local_matrices.size() == 3u &&
            evaluated.value.global_matrices.size() == 3u &&
            evaluated.value.skin_poses.size() == 1u,
            "CPU Animation Hierarchy evaluation must publish every Node matrix and one Skin Pose.");

        const mat4& root_global =
            evaluated.value.global_matrices[0u];
        const mat4& joint_global =
            evaluated.value.global_matrices[1u];
        const mat4& mesh_global =
            evaluated.value.global_matrices[2u];
        require(std::abs(root_global.value[12u] - 1.0f) <= 1.0e-6f &&
            std::abs(joint_global.value[12u] - 1.0f) <= 1.0e-6f &&
            std::abs(joint_global.value[13u] - 2.0f) <= 1.0e-6f &&
            std::abs(mesh_global.value[12u] - 1.0f) <= 1.0e-6f &&
            std::abs(mesh_global.value[14u] - 2.0f) <= 1.0e-6f,
            "Node Global Matrices must accumulate Parent * Local in topological order and preserve static Matrix Nodes.");

        const mat4& root_global_derivative =
            evaluated.value.global_matrix_derivatives[0u];
        const mat4& joint_global_derivative =
            evaluated.value.global_matrix_derivatives[1u];
        const mat4& mesh_global_derivative =
            evaluated.value.global_matrix_derivatives[2u];
        require(std::abs(root_global_derivative.value[12u] - 2.0f) <= 1.0e-6f &&
            std::abs(joint_global_derivative.value[12u] - 2.0f) <= 1.0e-6f &&
            std::abs(joint_global_derivative.value[13u] - 2.0f) <= 1.0e-6f &&
            std::abs(mesh_global_derivative.value[12u] - 2.0f) <= 1.0e-6f &&
            std::abs(mesh_global_derivative.value[14u]) <= 1.0e-6f,
            "Global Matrix Derivatives must apply the Product Rule through animated and static Nodes.");

        const skin_physics::physics::animation_skin_pose& skin_pose =
            evaluated.value.skin_poses[0u];
        require(skin_pose.mesh_node_index == 2u &&
            skin_pose.skin_index == 0u &&
            skin_pose.skin_matrices.size() == 1u &&
            skin_pose.skin_matrix_derivatives.size() == 1u,
            "Each skinned Mesh Node must expose one Skin Matrix and analytic derivative per Joint.");
        const mat4& skin_matrix = skin_pose.skin_matrices[0u];
        const mat4& skin_matrix_derivative =
            skin_pose.skin_matrix_derivatives[0u];
        require(std::abs(skin_matrix.value[12u]) <= 1.0e-6f &&
            std::abs(skin_matrix.value[13u] - 2.0f) <= 1.0e-6f &&
            std::abs(skin_matrix.value[14u] + 2.0f) <= 1.0e-6f,
            "Skin Matrix must be inverse(Mesh Node Global) * Joint Global * Inverse Bind.");
        require(std::abs(skin_matrix_derivative.value[12u]) <= 1.0e-6f &&
            std::abs(skin_matrix_derivative.value[13u] - 2.0f) <= 1.0e-6f &&
            std::abs(skin_matrix_derivative.value[14u]) <= 1.0e-6f,
            "Skin Matrix derivative must cancel shared Root motion and retain the Joint's relative analytic velocity.");

        model invalid_hierarchy = source_model;
        invalid_hierarchy.nodes[0u].parent_index = 1u;
        const skin_physics::physics::animation_hierarchy_sample_result cyclic_hierarchy =
            skin_physics::physics::evaluate_animation_clip_hierarchy(
                invalid_hierarchy, clip, clip_spans.values[0u], 0.5, 1.0);
        require(!cyclic_hierarchy.success &&
            cyclic_hierarchy.value.global_matrices.empty() &&
            cyclic_hierarchy.error ==
                skin_physics::physics::animation_hierarchy_error::invalid_hierarchy,
            "A cyclic Asset Node hierarchy must fail atomically without publishing partial Global Matrices.");

        model singular_mesh = source_model;
        singular_mesh.nodes[2u].local_matrix.value[0u] = 0.0f;
        const skin_physics::physics::animation_hierarchy_sample_result singular_skin =
            skin_physics::physics::evaluate_animation_clip_hierarchy(
                singular_mesh, clip, clip_spans.values[0u], 0.5, 1.0);
        require(!singular_skin.success &&
            singular_skin.value.skin_poses.empty() &&
            singular_skin.error ==
                skin_physics::physics::animation_hierarchy_error::singular_mesh_node,
            "A singular Mesh Node Global Matrix must fail before emitting an invalid Skin Matrix.");

        skin_physics::asset::animation_clip source_blend_clip = clip;
        animation_track source_root_rotation;
        source_root_rotation.node_index = 0u;
        source_root_rotation.path = animation_path::rotation;
        source_root_rotation.mode = interpolation::linear;
        source_root_rotation.component_count = 4u;
        animation_key source_rotation_start;
        source_rotation_start.time = 0.0f;
        source_rotation_start.value = { 0.0f, 0.0f, 0.0f, 1.0f };
        animation_key source_rotation_end;
        source_rotation_end.time = 2.0f;
        source_rotation_end.value = {
            0.0f, 0.0f,
            std::sin(3.14159265358979323846f / 4.0f),
            std::cos(3.14159265358979323846f / 4.0f)
        };
        source_root_rotation.keys = {
            source_rotation_start, source_rotation_end
        };
        source_blend_clip.tracks.push_back(source_root_rotation);

        skin_physics::asset::animation_clip target_blend_clip =
            source_blend_clip;
        target_blend_clip.name = "target_hierarchy_translation_rotation";
        target_blend_clip.tracks[0u].keys[0u].value[0u] = 2.0f;
        target_blend_clip.tracks[0u].keys[1u].value[0u] = 4.0f;
        target_blend_clip.tracks[1u].keys[0u].value[1u] = 3.0f;
        target_blend_clip.tracks[1u].keys[1u].value[1u] = 5.0f;
        target_blend_clip.tracks[2u].keys[1u].value = {
            0.0f, 0.0f, 1.0f, 0.0f
        };

        skin_physics::physics::animation_blend_weight_curve_desc blend_curve;
        blend_curve.blend_start_time = 10.0;
        blend_curve.blend_end_time = 12.0;
        blend_curve.weight_start = 0.0f;
        blend_curve.weight_end = 1.0f;
        blend_curve.tangent_start = 0.0f;
        blend_curve.tangent_end = 0.0f;
        const skin_physics::physics::animation_blend_clip_span_schedule_result
            blend_schedule =
                skin_physics::physics::schedule_animation_blend_clip_spans(
                    source_blend_clip, 3u, 0.0, 1.0, false,
                    target_blend_clip, 3u, 0.0, 1.0, false,
                    blend_curve, 10.0, 2.0);
        require(blend_schedule.success && blend_schedule.values.size() == 1u,
            "Source / Target Clip and Hermite Weight boundaries must produce one shared World-Time Pose Span.");

        const skin_physics::physics::animation_hierarchy_sample_result blended_pose =
            skin_physics::physics::evaluate_animation_blend_clip_hierarchy(
                source_model,
                source_blend_clip,
                target_blend_clip,
                blend_schedule.values[0u],
                0.5);
        require(blended_pose.success &&
            blended_pose.value.local_transforms.size() == source_model.nodes.size() &&
            blended_pose.value.global_matrices.size() == source_model.nodes.size() &&
            blended_pose.value.skin_poses.size() == 1u,
            "CPU Blend Pose evaluation must combine Source / Target local transforms before rebuilding the complete hierarchy and Skin Pose.");

        const skin_physics::animation_node_local_transform& blended_root =
            blended_pose.value.local_transforms[0u];
        const skin_physics::animation_node_local_transform& blended_joint =
            blended_pose.value.local_transforms[1u];
        require(std::abs(blended_root.translation.x - 2.0f) <= 1.0e-6f &&
            std::abs(blended_root.translation_velocity.x - 2.5f) <= 1.0e-5f &&
            std::abs(blended_joint.translation.y - 3.0f) <= 1.0e-6f &&
            std::abs(blended_joint.translation_velocity.y - 2.5f) <= 1.0e-5f,
            "Translation Blend must include w_dot * (Target - Source) in its analytic World-Time derivative.");

        const float blended_rotation_norm = std::sqrt(
            blended_root.rotation.x * blended_root.rotation.x +
            blended_root.rotation.y * blended_root.rotation.y +
            blended_root.rotation.z * blended_root.rotation.z +
            blended_root.rotation.w * blended_root.rotation.w);
        const float blended_rotation_tangent_dot =
            blended_root.rotation.x * blended_root.rotation_velocity.x +
            blended_root.rotation.y * blended_root.rotation_velocity.y +
            blended_root.rotation.z * blended_root.rotation_velocity.z +
            blended_root.rotation.w * blended_root.rotation_velocity.w;
        require(std::abs(blended_rotation_norm - 1.0f) <= 1.0e-5f &&
            std::abs(blended_rotation_tangent_dot) <= 1.0e-5f &&
            std::isfinite(blended_root.rotation_velocity.z) &&
            std::isfinite(blended_root.rotation_velocity.w),
            "Quaternion Blend must hemisphere-align Source / Target, normalize the result and project its analytic derivative into the quaternion tangent space.");

        const mat4& blended_root_global =
            blended_pose.value.global_matrices[0u];
        const mat4& blended_joint_global =
            blended_pose.value.global_matrices[1u];
        const mat4& blended_root_global_derivative =
            blended_pose.value.global_matrix_derivatives[0u];
        require(std::abs(blended_root_global.value[12u] - 2.0f) <= 1.0e-5f &&
            std::abs(blended_joint_global.value[13u] - 3.0f) <= 1.0e-5f &&
            std::abs(blended_root_global_derivative.value[12u] - 2.5f) <= 1.0e-5f,
            "Blended Local Pose must be propagated through Parent * Local hierarchy matrices and analytic derivatives.");

        const mat4& blended_skin_matrix =
            blended_pose.value.skin_poses[0u].skin_matrices[0u];
        const mat4& blended_skin_derivative =
            blended_pose.value.skin_poses[0u].skin_matrix_derivatives[0u];
        require(std::abs(blended_skin_matrix.value[12u]) <= 1.0e-5f &&
            std::abs(blended_skin_matrix.value[13u] - 3.0f) <= 1.0e-5f &&
            std::abs(blended_skin_matrix.value[14u] + 2.0f) <= 1.0e-5f &&
            std::abs(blended_skin_derivative.value[12u]) <= 1.0e-5f &&
            std::abs(blended_skin_derivative.value[13u] - 2.5f) <= 1.0e-5f,
            "Blended Skin Matrix / derivative must be rebuilt after the Source / Target local Pose blend and cancel shared Root motion.");

        skin_physics::physics::animation_blend_clip_span mismatched_blend_span =
            blend_schedule.values[0u];
        mismatched_blend_span.target.world_time_duration += 0.25;
        const skin_physics::physics::animation_hierarchy_sample_result
            mismatched_blend_pose =
                skin_physics::physics::evaluate_animation_blend_clip_hierarchy(
                    source_model,
                    source_blend_clip,
                    target_blend_clip,
                    mismatched_blend_span,
                    0.5);
        require(!mismatched_blend_pose.success &&
            mismatched_blend_pose.value.global_matrices.empty() &&
            mismatched_blend_pose.error ==
                skin_physics::physics::animation_hierarchy_error::invalid_blend,
            "Source / Target / Weight interval mismatch must fail atomically before publishing a blended hierarchy.");
    }

    void test_phase10_attribute_paint_and_serialization()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase10_paint/paint.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase10_paint/paint.expected.json"));

        std::vector<float> weights;
        for (const test_json::value& weight : input.at("initial_weights").as_array())
        {
            weights.push_back(static_cast<float>(weight.as_number()));
        }
        std::vector<std::vector<uint32_t>> adjacency;
        for (const test_json::value& neighbors : input.at("adjacency").as_array())
        {
            std::vector<uint32_t> vertex_neighbors;
            for (const test_json::value& neighbor : neighbors.as_array())
            {
                vertex_neighbors.push_back(static_cast<uint32_t>(neighbor.as_number()));
            }
            adjacency.push_back(std::move(vertex_neighbors));
        }
        std::vector<uint32_t> selection = { 0, 1, 2 };
        const float minimum_value = static_cast<float>(input.at("clamp_range").at(0).as_number());
        const float maximum_value = static_cast<float>(input.at("clamp_range").at(1).as_number());

        const auto painted = skin_physics::physics::apply_attribute_paint(
            weights, adjacency, selection,
            skin_physics::physics::attribute_paint_mode::paint,
            static_cast<float>(input.at("operations").at(0).at("target").as_number()),
            static_cast<float>(input.at("operations").at(0).at("strength").as_number()),
            minimum_value, maximum_value);
        require(painted.success, "Phase 10 Paint operation should succeed.");
        weights = painted.weights;
        for (std::size_t index = 0; index < weights.size(); ++index)
        {
            require(std::abs(weights[index] -
                expected.at("after_paint").at(index).as_number()) <= 1.0e-6,
                "Paint output must match the golden weight.");
        }

        const auto smoothed = skin_physics::physics::apply_attribute_paint(
            weights, adjacency, selection,
            skin_physics::physics::attribute_paint_mode::smooth,
            0.0f, 1.0f, minimum_value, maximum_value);
        require(smoothed.success, "Phase 10 Smooth operation should succeed.");
        weights = smoothed.weights;
        for (std::size_t index = 0; index < weights.size(); ++index)
        {
            require(std::abs(weights[index] -
                expected.at("after_smooth").at(index).as_number()) <= 1.0e-6,
                "Smooth operation should converge the selected region to its mean.");
        }

        const auto averaged = skin_physics::physics::apply_attribute_paint(
            weights, adjacency, selection,
            skin_physics::physics::attribute_paint_mode::average,
            0.0f, 1.0f, minimum_value, maximum_value);
        require(averaged.success, "Phase 10 Average operation should succeed.");
        weights = averaged.weights;
        for (std::size_t index = 0; index < weights.size(); ++index)
        {
            require(std::abs(weights[index] -
                expected.at("after_average").at(index).as_number()) <= 1.0e-6,
                "Average result must match the reviewed golden.");
        }

        const auto erased = skin_physics::physics::apply_attribute_paint(
            weights, adjacency, selection,
            skin_physics::physics::attribute_paint_mode::erase,
            0.0f,
            static_cast<float>(input.at("operations").at(3).at("strength").as_number()),
            minimum_value, maximum_value);
        require(erased.success, "Phase 10 Erase operation should succeed.");
        weights = erased.weights;
        for (std::size_t index = 0; index < weights.size(); ++index)
        {
            require(std::abs(weights[index] -
                expected.at("after_erase").at(index).as_number()) <= 1.0e-6,
                "Erase output must match the golden weight.");
            require(weights[index] >= minimum_value && weights[index] <= maximum_value,
                "Paint output must remain inside the configured clamp range.");
        }

        std::vector<uint8_t> serialized;
        require(skin_physics::physics::serialize_attribute_weights(weights, serialized),
            "Attribute weight serialization should succeed.");
        std::vector<float> loaded;
        require(skin_physics::physics::deserialize_attribute_weights(serialized, loaded),
            "Attribute weight deserialization should succeed.");
        require(loaded == weights && expected.at("save_load_exact").as_bool(),
            "Attribute weights must round-trip exactly.");

        std::vector<float> preserved = loaded;
        serialized.back() ^= 0x01u;
        require(!skin_physics::physics::deserialize_attribute_weights(serialized, loaded) &&
            loaded == preserved,
            "Checksum failure must reject the payload without partially replacing loaded weights.");
    }

    void test_phase11_performance_profile_and_invariant_collector()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase11_performance/scale_profiles.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase11_performance/scale_profiles.expected.json"));

        const auto& tet_sizes = input.at("soft_body_tetrahedra").as_array();
        const auto& cloth_sizes = input.at("cloth_triangles").as_array();
        const auto& expected_tet_sizes =
            expected.at("profile_sizes").at("soft_body_tetrahedra").as_array();
        const auto& expected_cloth_sizes =
            expected.at("profile_sizes").at("cloth_triangles").as_array();
        require(tet_sizes.size() == 3 && cloth_sizes.size() == 3,
            "Phase 11 fixture should define three workload sizes per solver.");
        for (std::size_t index = 0; index < 3; ++index)
        {
            require(tet_sizes[index].as_number() == expected_tet_sizes[index].as_number() &&
                cloth_sizes[index].as_number() == expected_cloth_sizes[index].as_number(),
                "Phase 11 workload sizes must match the reviewed profile.");
        }
        require(input.at("repeated_runs").as_number() == 5,
            "Phase 11 profile must use five repeated runs.");
        require(std::holds_alternative<std::nullptr_t>(
            expected.at("absolute_fps_threshold").data),
            "Performance validation must not enforce a hardware-independent FPS golden.");

        skin_physics::physics::performance_collector collector;
        for (uint32_t run_index = 0; run_index < 5; ++run_index)
        {
            skin_physics::physics::performance_sample sample;
            sample.scenario_hash = "phase11_fixture_scenario";
            sample.cpu_time_ms = static_cast<double>(run_index + 1);
            sample.counters.tetrahedron_count = 32000;
            sample.counters.cloth_triangle_count = 32000;
            sample.counters.candidate_count = 256 + run_index;
            sample.counters.contact_count = 128 + run_index;
            sample.counters.ccd_iteration_count = 8 + run_index;
            sample.counters.library_allocated_bytes = 64u * 1024u * 1024u;
            sample.counters.dxgi_memory_available = false;
            sample.all_values_finite = true;
            sample.all_capacity_overflows_reported = true;
            sample.candidate_set_contains_reference = true;
            sample.state_committed_only_after_complete_step = true;

            skin_physics::physics::performance_error error;
            require(collector.add_sample(sample, error) &&
                error == skin_physics::physics::performance_error::none,
                "Valid deterministic performance sample should be accepted.");
        }

        const auto summary = collector.summarize(
            "phase11_fixture_scenario",
            static_cast<std::size_t>(input.at("repeated_runs").as_number()));
        require(summary.success && summary.sample_count == 5,
            "Performance summary should accept five samples with the same scenario hash.");
        require(std::abs(summary.cpu_time_ms.median - 3.0) <= 1.0e-12 &&
            std::abs(summary.cpu_time_ms.percentile_95 - 5.0) <= 1.0e-12 &&
            std::abs(summary.cpu_time_ms.maximum - 5.0) <= 1.0e-12,
            "CPU median, p95 and maximum must match the deterministic sample set.");
        require(summary.maximum_counters.tetrahedron_count == 32000 &&
            summary.maximum_counters.cloth_triangle_count == 32000 &&
            summary.maximum_counters.candidate_count == 260 &&
            summary.maximum_counters.contact_count == 132,
            "Workload and contact counters must retain the maximum observed values.");
        require(!summary.gpu_timing_available && !summary.dxgi_memory_available,
            "Unavailable physical GPU and DXGI readings must remain explicitly unavailable.");

        const auto mismatch = collector.summarize("different_scenario", 1);
        require(!mismatch.success &&
            mismatch.diagnostic.find("scenario hashes") != std::string::npos,
            "Performance runs with different scenario hashes must not be compared.");

        skin_physics::physics::performance_sample invalid_sample;
        invalid_sample.scenario_hash = "phase11_fixture_scenario";
        invalid_sample.cpu_time_ms = 1.0;
        invalid_sample.candidate_set_contains_reference = false;
        skin_physics::physics::performance_error error;
        require(!collector.add_sample(invalid_sample, error) &&
            error == skin_physics::physics::performance_error::correctness_invariant_failed,
            "A sample that violates a correctness invariant must be rejected.");
        require(collector.sample_count() == 5,
            "Rejected performance samples must not mutate collector state.");

        collector.reset();
        require(collector.sample_count() == 0,
            "Performance collector Reset must clear all previous samples.");
    }

    void test_phase03_warp_compute_render_feedback()
    {
        using Microsoft::WRL::ComPtr;
        struct gpu_float4
        {
            float x;
            float y;
            float z;
            float w;
        };
        struct gpu_render_binding
        {
            uint32_t indices[4];
            float weights[4];
            float rest_offset[4];
        };
        static_assert(sizeof(gpu_float4) == 16, "GPU float4 must match HLSL float4.");
        static_assert(sizeof(gpu_render_binding) == 48, "GPU binding stride must match HLSL layout.");

        UINT creation_flags = 0;
#if defined(_DEBUG)
        creation_flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
        const D3D_FEATURE_LEVEL requested_levels[] = {
            D3D_FEATURE_LEVEL_11_0
        };
        D3D_FEATURE_LEVEL feature_level = D3D_FEATURE_LEVEL_11_0;
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        HRESULT hr = D3D11CreateDevice(
            nullptr,
            D3D_DRIVER_TYPE_WARP,
            nullptr,
            creation_flags,
            requested_levels,
            1,
            D3D11_SDK_VERSION,
            device.GetAddressOf(),
            &feature_level,
            context.GetAddressOf());
        require(SUCCEEDED(hr) && device != nullptr && context != nullptr,
            "D3D11 WARP device creation must succeed for GPU reference tests.");

        const std::filesystem::path shader_path =
            test_support::fixture_path("phase03_render_binding/render_feedback.cs.hlsl");
        ComPtr<ID3DBlob> shader_bytecode;
        ComPtr<ID3DBlob> shader_errors;
        hr = D3DCompileFromFile(
            shader_path.c_str(),
            nullptr,
            D3D_COMPILE_STANDARD_FILE_INCLUDE,
            "main",
            "cs_5_0",
            D3DCOMPILE_ENABLE_STRICTNESS,
            0,
            shader_bytecode.GetAddressOf(),
            shader_errors.GetAddressOf());
        if (FAILED(hr))
        {
            std::string detail;
            if (shader_errors != nullptr)
            {
                const char* bytes = static_cast<const char*>(shader_errors->GetBufferPointer());
                detail.assign(bytes, bytes + shader_errors->GetBufferSize());
            }
            throw std::runtime_error("Phase 3 HLSL compile failed: " + detail);
        }

        ComPtr<ID3D11ComputeShader> shader;
        hr = device->CreateComputeShader(
            shader_bytecode->GetBufferPointer(),
            shader_bytecode->GetBufferSize(),
            nullptr,
            shader.GetAddressOf());
        require(SUCCEEDED(hr), "Phase 3 compute shader creation must succeed.");

        const std::vector<gpu_float4> physics_positions = {
            { 0.0f, 0.0f, 0.0f, 1.0f },
            { 1.0f, 0.0f, 0.0f, 1.0f },
            { 0.0f, 1.0f, 0.0f, 1.0f },
            { 0.0f, 0.0f, 1.0f, 1.0f }
        };
        const std::vector<gpu_render_binding> bindings = {
            { { 0, 1, 2, 3 }, { 0.4f, 0.1f, 0.2f, 0.3f }, { 0.0f, 0.0f, 0.0f, 0.0f } },
            { { 2, 0, 0, 0 }, { 1.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } },
            { { 0, 1, 2, 3 }, { 0.6f, 0.2f, 0.2f, 0.0f }, { 0.0f, 0.0f, -0.1f, 0.0f } }
        };
        std::vector<gpu_float4> initial_output(bindings.size(), { -99.0f, -99.0f, -99.0f, -99.0f });

        const auto create_structured_buffer =
            [&device](const void* data, UINT element_count, UINT stride,
                UINT bind_flags, ID3D11Buffer** output) -> HRESULT
        {
            D3D11_BUFFER_DESC description = {};
            description.ByteWidth = element_count * stride;
            description.Usage = D3D11_USAGE_DEFAULT;
            description.BindFlags = bind_flags;
            description.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
            description.StructureByteStride = stride;
            D3D11_SUBRESOURCE_DATA initial_data = {};
            initial_data.pSysMem = data;
            return device->CreateBuffer(&description, &initial_data, output);
        };

        ComPtr<ID3D11Buffer> physics_buffer;
        ComPtr<ID3D11Buffer> binding_buffer;
        ComPtr<ID3D11Buffer> output_buffer;
        hr = create_structured_buffer(
            physics_positions.data(),
            static_cast<UINT>(physics_positions.size()),
            sizeof(gpu_float4),
            D3D11_BIND_SHADER_RESOURCE,
            physics_buffer.GetAddressOf());
        require(SUCCEEDED(hr), "Physics position buffer creation must succeed.");
        hr = create_structured_buffer(
            bindings.data(),
            static_cast<UINT>(bindings.size()),
            sizeof(gpu_render_binding),
            D3D11_BIND_SHADER_RESOURCE,
            binding_buffer.GetAddressOf());
        require(SUCCEEDED(hr), "Render binding buffer creation must succeed.");
        hr = create_structured_buffer(
            initial_output.data(),
            static_cast<UINT>(initial_output.size()),
            sizeof(gpu_float4),
            D3D11_BIND_UNORDERED_ACCESS,
            output_buffer.GetAddressOf());
        require(SUCCEEDED(hr), "Render output buffer creation must succeed.");

        D3D11_SHADER_RESOURCE_VIEW_DESC physics_view_description = {};
        physics_view_description.Format = DXGI_FORMAT_UNKNOWN;
        physics_view_description.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
        physics_view_description.Buffer.NumElements =
            static_cast<UINT>(physics_positions.size());
        ComPtr<ID3D11ShaderResourceView> physics_view;
        hr = device->CreateShaderResourceView(
            physics_buffer.Get(), &physics_view_description, physics_view.GetAddressOf());
        require(SUCCEEDED(hr), "Physics position SRV creation must succeed.");

        D3D11_SHADER_RESOURCE_VIEW_DESC binding_view_description = {};
        binding_view_description.Format = DXGI_FORMAT_UNKNOWN;
        binding_view_description.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
        binding_view_description.Buffer.NumElements = static_cast<UINT>(bindings.size());
        ComPtr<ID3D11ShaderResourceView> binding_view;
        hr = device->CreateShaderResourceView(
            binding_buffer.Get(), &binding_view_description, binding_view.GetAddressOf());
        require(SUCCEEDED(hr), "Render binding SRV creation must succeed.");

        D3D11_UNORDERED_ACCESS_VIEW_DESC output_view_description = {};
        output_view_description.Format = DXGI_FORMAT_UNKNOWN;
        output_view_description.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
        output_view_description.Buffer.NumElements = static_cast<UINT>(bindings.size());
        ComPtr<ID3D11UnorderedAccessView> output_view;
        hr = device->CreateUnorderedAccessView(
            output_buffer.Get(), &output_view_description, output_view.GetAddressOf());
        require(SUCCEEDED(hr), "Render output UAV creation must succeed.");

        ID3D11ShaderResourceView* resource_views[] = {
            physics_view.Get(), binding_view.Get()
        };
        ID3D11UnorderedAccessView* unordered_views[] = { output_view.Get() };
        context->CSSetShader(shader.Get(), nullptr, 0);
        context->CSSetShaderResources(0, 2, resource_views);
        context->CSSetUnorderedAccessViews(0, 1, unordered_views, nullptr);
        context->Dispatch(1, 1, 1);

        ID3D11ShaderResourceView* null_resources[] = { nullptr, nullptr };
        ID3D11UnorderedAccessView* null_unordered[] = { nullptr };
        context->CSSetShaderResources(0, 2, null_resources);
        context->CSSetUnorderedAccessViews(0, 1, null_unordered, nullptr);
        context->CSSetShader(nullptr, nullptr, 0);

        D3D11_BUFFER_DESC staging_description = {};
        staging_description.ByteWidth =
            static_cast<UINT>(initial_output.size() * sizeof(gpu_float4));
        staging_description.Usage = D3D11_USAGE_STAGING;
        staging_description.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        ComPtr<ID3D11Buffer> staging_buffer;
        hr = device->CreateBuffer(&staging_description, nullptr, staging_buffer.GetAddressOf());
        require(SUCCEEDED(hr), "GPU staging buffer creation must succeed.");
        context->CopyResource(staging_buffer.Get(), output_buffer.Get());

        D3D11_MAPPED_SUBRESOURCE mapped = {};
        hr = context->Map(staging_buffer.Get(), 0, D3D11_MAP_READ, 0, &mapped);
        require(SUCCEEDED(hr), "GPU output Readback Map must succeed.");
        const gpu_float4* gpu_output = static_cast<const gpu_float4*>(mapped.pData);
        const double tolerance = 1.0e-5;
        require(std::abs(gpu_output[0].x - 0.1) <= tolerance &&
            std::abs(gpu_output[0].y - 0.2) <= tolerance &&
            std::abs(gpu_output[0].z - 0.3) <= tolerance,
            "WARP tetrahedral barycentric feedback must match the analytical reference.");
        require(std::abs(gpu_output[1].x) <= tolerance &&
            std::abs(gpu_output[1].y - 1.0) <= tolerance &&
            std::abs(gpu_output[1].z) <= tolerance,
            "WARP direct-vertex feedback must match the source physics vertex.");
        require(std::abs(gpu_output[2].x - 0.2) <= tolerance &&
            std::abs(gpu_output[2].y - 0.2) <= tolerance &&
            std::abs(gpu_output[2].z + 0.1) <= tolerance,
            "WARP boundary feedback must preserve the stored rest offset.");
        context->Unmap(staging_buffer.Get(), 0);
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

    void test_phase04_pause_stop_and_reconfigure_preserve_accumulator()
    {
        skin_physics::simulation::fixed_step_settings settings;
        settings.fixed_delta_time = 0.01;
        settings.time_scale = 1.0;
        settings.maximum_steps_per_advance = 1;
        fixed_step_controller controller(settings);

        uint32_t callback_count = 0;
        const skin_physics::simulation::fixed_step_callback callback =
            [&callback_count](double delta_time)
            {
                ++callback_count;
                return std::isfinite(delta_time) && delta_time > 0.0;
            };

        controller.play();
        const auto first = controller.advance(0.025, callback);
        require(first.completed_steps == 1 &&
            std::abs(first.accumulator - 0.015) <= 1.0e-9,
            "Step admission must retain the unconsumed fixed-step time.");

        controller.pause();
        const auto paused = controller.advance(0.5, callback);
        require(paused.completed_steps == 0 &&
            std::abs(paused.accumulator - 0.015) <= 1.0e-9,
            "Pause must stop wall-clock ingestion and preserve the accumulator.");

        controller.stop();
        require(controller.state() == skin_physics::simulation::playback_state::stopped &&
            std::abs(controller.accumulator() - 0.015) <= 1.0e-9,
            "Stop must preserve unassigned accumulator time; only Reset discards it.");

        require(controller.set_fixed_delta_time(0.005) &&
            std::abs(controller.accumulator() - 0.015) <= 1.0e-9,
            "Changing the fixed interval must preserve already requested simulation time.");

        settings = controller.settings();
        settings.maximum_steps_per_advance = 1;
        require(controller.configure(settings) &&
            std::abs(controller.accumulator() - 0.015) <= 1.0e-9,
            "Reconfiguration must preserve unassigned time instead of silently dropping it.");

        controller.play();
        const auto resumed = controller.advance(0.0, callback);
        require(resumed.completed_steps == 1 &&
            std::abs(resumed.simulation_time - 0.015) <= 1.0e-9 &&
            std::abs(resumed.accumulator - 0.010) <= 1.0e-9,
            "Resuming must process preserved backlog using the current fixed interval.");

        controller.reset();
        require(controller.state() == skin_physics::simulation::playback_state::stopped &&
            controller.simulation_time() == 0.0 &&
            controller.accumulator() == 0.0,
            "Reset must explicitly discard the accumulator and restore time zero.");
        require(callback_count == 2,
            "Paused ingestion must not invoke the solver callback.");
    }

    const test_json::value& find_correction_fixture_case(
        const test_json::value& fixture,
        const std::string& case_id)
    {
        for (const test_json::value& item : fixture.at("cases").as_array())
        {
            if (item.at("id").as_string() == case_id)
            {
                return item;
            }
        }
        throw std::runtime_error("Missing correction CCD fixture case: " + case_id);
    }

    std::array<skin_physics::asset::vec3, 3> read_vec3_triplet(
        const test_json::value& value)
    {
        std::array<skin_physics::asset::vec3, 3> result = {};
        for (std::size_t index = 0; index < result.size(); ++index)
        {
            result[index] = read_vec3(value.at(index));
        }
        return result;
    }

    std::array<skin_physics::asset::vec3, 2> read_vec3_pair(
        const test_json::value& value)
    {
        return { read_vec3(value.at(0)), read_vec3(value.at(1)) };
    }

    skin_physics::physics::vertex_triangle_correction_motion make_vt_correction_motion(
        const test_json::value& input)
    {
        using skin_physics::physics::vertex_triangle_correction_motion;

        vertex_triangle_correction_motion motion;
        motion.point.before = read_vec3(input.at("point_before"));
        motion.point.after = read_vec3(input.at("point_after"));
        const auto triangle_before = read_vec3_triplet(
            input.at("triangle_vertices_before"));
        const auto triangle_after = read_vec3_triplet(
            input.at("triangle_vertices_after"));
        for (std::size_t index = 0; index < 3; ++index)
        {
            motion.triangle[index].before = triangle_before[index];
            motion.triangle[index].after = triangle_after[index];
        }

        const auto& fields = input.as_object();
        const auto segment_point = fields.find("segment_start_point");
        motion.segment_start_point = segment_point != fields.end()
            ? read_vec3(segment_point->second) : motion.point.before;
        const auto segment_triangle = fields.find("triangle_vertices_segment_start");
        motion.segment_start_triangle = segment_triangle != fields.end()
            ? read_vec3_triplet(segment_triangle->second) : triangle_before;

        const auto cache_flag = fields.find("has_valid_persistent_contact_cache");
        motion.has_valid_persistent_contact_cache = cache_flag != fields.end() &&
            cache_flag->second.as_bool();
        const auto cache_normal = fields.find("persistent_contact_normal");
        if (cache_normal != fields.end())
        {
            motion.persistent_contact_normal = read_vec3(cache_normal->second);
        }
        return motion;
    }

    skin_physics::physics::edge_edge_correction_motion make_ee_correction_motion(
        const test_json::value& input)
    {
        using skin_physics::physics::edge_edge_correction_motion;

        edge_edge_correction_motion motion;
        const auto edge_a_before = read_vec3_pair(input.at("edge_a_before"));
        const auto edge_a_after = read_vec3_pair(input.at("edge_a_after"));
        const auto edge_b_before = read_vec3_pair(input.at("edge_b_before"));
        const auto edge_b_after = read_vec3_pair(input.at("edge_b_after"));
        for (std::size_t index = 0; index < 2; ++index)
        {
            motion.edge_a[index].before = edge_a_before[index];
            motion.edge_a[index].after = edge_a_after[index];
            motion.edge_b[index].before = edge_b_before[index];
            motion.edge_b[index].after = edge_b_after[index];
        }
        motion.segment_start_edge_a = edge_a_before;
        motion.segment_start_edge_b = edge_b_before;
        const auto& fields = input.as_object();
        const auto cache_flag = fields.find("has_valid_persistent_contact_cache");
        motion.has_valid_persistent_contact_cache = cache_flag != fields.end() &&
            cache_flag->second.as_bool();
        const auto cache_normal = fields.find("persistent_contact_normal");
        if (cache_normal != fields.end())
        {
            motion.persistent_contact_normal = read_vec3(cache_normal->second);
        }
        return motion;
    }

    skin_physics::physics::correction_ccd_settings read_correction_ccd_settings(
        const test_json::value& input)
    {
        skin_physics::physics::correction_ccd_settings settings;
        settings.contact_thickness = static_cast<float>(
            input.at("contact_thickness_m").as_number());
        return settings;
    }

    std::vector<skin_physics::physics::correction_primitive_epoch_input>
    make_vt_position_writer_epoch_primitives(const test_json::value& input)
    {
        using namespace skin_physics::physics;
        const vertex_triangle_correction_motion motion =
            make_vt_correction_motion(input);

        correction_primitive_epoch_input point;
        point.stable_id = 101;
        point.mesh_id = 1;
        point.primitive_id = 0;
        point.role = correction_primitive_role::vertex;
        point.vertex_stable_ids = { 1001 };
        point.vertex_motions = { motion.point };
        point.segment_start_positions = { motion.segment_start_point };

        correction_primitive_epoch_input triangle;
        triangle.stable_id = 102;
        triangle.mesh_id = 1;
        triangle.primitive_id = 1;
        triangle.role = correction_primitive_role::triangle;
        triangle.vertex_stable_ids = { 1002, 1003, 1004 };
        for (std::size_t index = 0; index < 3; ++index)
        {
            triangle.vertex_motions.push_back(motion.triangle[index]);
            triangle.segment_start_positions.push_back(
                motion.segment_start_triangle[index]);
        }
        return { point, triangle };
    }

    std::vector<skin_physics::physics::correction_primitive_epoch_input>
    make_ee_position_writer_epoch_primitives(const test_json::value& input)
    {
        using namespace skin_physics::physics;
        const edge_edge_correction_motion motion =
            make_ee_correction_motion(input);

        correction_primitive_epoch_input edge_a;
        edge_a.stable_id = 201;
        edge_a.mesh_id = 1;
        edge_a.primitive_id = 2;
        edge_a.role = correction_primitive_role::edge;
        edge_a.vertex_stable_ids = { 2001, 2002 };
        correction_primitive_epoch_input edge_b;
        edge_b.stable_id = 202;
        edge_b.mesh_id = 1;
        edge_b.primitive_id = 3;
        edge_b.role = correction_primitive_role::edge;
        edge_b.vertex_stable_ids = { 2003, 2004 };
        for (std::size_t endpoint = 0; endpoint < 2; ++endpoint)
        {
            edge_a.vertex_motions.push_back(motion.edge_a[endpoint]);
            edge_a.segment_start_positions.push_back(
                motion.segment_start_edge_a[endpoint]);
            edge_b.vertex_motions.push_back(motion.edge_b[endpoint]);
            edge_b.segment_start_positions.push_back(
                motion.segment_start_edge_b[endpoint]);
        }
        return { edge_a, edge_b };
    }

    skin_physics::physics::broadphase_settings make_correction_epoch_broadphase_settings()
    {
        skin_physics::physics::broadphase_settings settings;
        settings.cell_size = 0.25f;
        settings.contact_thickness = 0.0f;
        settings.cell_record_capacity = 4096;
        settings.candidate_capacity = 16;
        settings.max_cells_per_primitive = 4096;
        settings.self_collision_enabled = true;
        return settings;
    }

    void test_phase12_correction_motion_ccd_reference()
    {
        using skin_physics::physics::correction_ccd_status;

        const test_json::value fixture = test_json::load(
            test_support::fixture_path(
                "design_review/correction_sweep_broadphase.json"));

        const auto& vt_crossing = find_correction_fixture_case(
            fixture, "DCR_49_VT_POINT_THROUGH_TRIANGLE");
        const auto vt_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                make_vt_correction_motion(vt_crossing.at("input")),
                read_correction_ccd_settings(vt_crossing.at("input")));
        require(vt_result.status == correction_ccd_status::path_crossing &&
            vt_result.hit_witness_valid,
            "Correction VT path crossing must produce a hit witness and REJECTED crossing status.");

        const auto& vt_negative = find_correction_fixture_case(
            fixture, "DCR_49_VT_POINT_THROUGH_TRIANGLE_NEGATIVE_START_SIDE");
        const auto vt_negative_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                make_vt_correction_motion(vt_negative.at("input")),
                read_correction_ccd_settings(vt_negative.at("input")));
        require(vt_negative_result.status == correction_ccd_status::path_crossing &&
            vt_negative_result.allowed_side_sign == -1,
            "Negative initial separation must latch sigma=-1 and detect the reverse crossing.");

        const auto& ee_crossing = find_correction_fixture_case(
            fixture, "DCR_49_EE_EDGE_THROUGH_EDGE");
        const auto ee_result =
            skin_physics::physics::detect_edge_edge_correction_motion(
                make_ee_correction_motion(ee_crossing.at("input")),
                read_correction_ccd_settings(ee_crossing.at("input")));
        require(ee_result.status == correction_ccd_status::path_crossing &&
            ee_result.hit_witness_valid,
            std::string(
                "Correction EE crossing must produce a hit witness and REJECTED crossing status. status=") +
                std::to_string(static_cast<int>(ee_result.status)) +
                " witness=" + (ee_result.hit_witness_valid ? "true" : "false") +
                " interval_evaluations=" +
                std::to_string(ee_result.interval_evaluations) +
                " diagnostic=" + ee_result.diagnostic);

        auto endpoint_clamped_ee = make_ee_correction_motion(
            ee_crossing.at("input"));
        endpoint_clamped_ee.segment_start_edge_a = {
            skin_physics::asset::vec3{ 1.0f, 0.0f, 0.0f },
            skin_physics::asset::vec3{ 2.0f, 0.0f, 0.0f }
        };
        endpoint_clamped_ee.segment_start_edge_b = {
            skin_physics::asset::vec3{ 0.0f, 1.0f, -1.0f },
            skin_physics::asset::vec3{ 0.0f, 2.0f, -1.0f }
        };
        const auto endpoint_clamped_result =
            skin_physics::physics::detect_edge_edge_correction_motion(
                endpoint_clamped_ee,
                read_correction_ccd_settings(ee_crossing.at("input")));
        require(endpoint_clamped_result.status == correction_ccd_status::unresolved,
            "Endpoint-clamped EE features must remain unresolved until a certified endpoint normal is implemented.");

        const auto& transient = find_correction_fixture_case(
            fixture, "DCR_49_VT_SAME_SIDE_TRANSIENT_PROXIMITY");
        const auto transient_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                make_vt_correction_motion(transient.at("input")),
                read_correction_ccd_settings(transient.at("input")));
        require(transient_result.status == correction_ccd_status::hit &&
            transient_result.hit_witness_valid,
            "Same-side transient proximity must be detected as a hit without crossing.");

        const auto& endpoint = find_correction_fixture_case(
            fixture, "DCR_49_VT_ENDPOINT_ACTIVE_CONTACT");
        const auto endpoint_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                make_vt_correction_motion(endpoint.at("input")),
                read_correction_ccd_settings(endpoint.at("input")));
        require(endpoint_result.status == correction_ccd_status::hit &&
            endpoint_result.hit_witness_valid &&
            endpoint_result.witness_s > 0.0f &&
            endpoint_result.witness_s <= 1.0f,
            "An endpoint-active VT contact must be discovered along the correction path.");

        const auto& miss = find_correction_fixture_case(
            fixture, "DCR_49_VT_CERTIFIED_MISS");
        const auto miss_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                make_vt_correction_motion(miss.at("input")),
                read_correction_ccd_settings(miss.at("input")));
        require(miss_result.status == correction_ccd_status::no_hit &&
            !miss_result.hit_witness_valid,
            "A certified VT miss must be returned only after all intervals are clear.");

        const auto& side_latch = find_correction_fixture_case(
            fixture, "DCR_49_SEGMENT_SIDE_LATCH_PRESERVES_START_SIDE");
        const auto& side_input = side_latch.at("input");
        const auto& epoch = side_input.at("position_writer_epochs").at(0);
        const auto side_triangle = read_vec3_triplet(
            side_input.at("triangle_vertices_segment_start"));
        skin_physics::physics::vertex_triangle_correction_motion side_motion;
        side_motion.point.before = read_vec3(epoch.at("point_before"));
        side_motion.point.after = read_vec3(epoch.at("point_after"));
        side_motion.segment_start_point = read_vec3(
            side_input.at("segment_start_point"));
        side_motion.segment_start_triangle = side_triangle;
        for (std::size_t index = 0; index < 3; ++index)
        {
            side_motion.triangle[index].before = side_triangle[index];
            side_motion.triangle[index].after = side_triangle[index];
        }
        skin_physics::physics::correction_ccd_settings side_settings;
        side_settings.contact_thickness = static_cast<float>(
            side_input.at("contact_thickness_m").as_number());
        const auto side_latch_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                side_motion, side_settings);
        require(side_latch_result.status == correction_ccd_status::path_crossing &&
            side_latch_result.allowed_side_sign == 1,
            "A later Position-Writer Epoch must retain the Integration Segment-start side latch.");

        auto invalid_motion = make_vt_correction_motion(vt_crossing.at("input"));
        invalid_motion.point.after.x = std::numeric_limits<float>::infinity();
        const auto invalid_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                invalid_motion, read_correction_ccd_settings(vt_crossing.at("input")));
        require(invalid_result.status == correction_ccd_status::invalid_input,
            "Non-finite correction displacements must never be classified as No-Hit.");

        skin_physics::physics::correction_ccd_settings invalid_settings;
        invalid_settings.maximum_interval_evaluations = 33;
        const auto settings_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                make_vt_correction_motion(vt_crossing.at("input")), invalid_settings);
        require(settings_result.status == correction_ccd_status::invalid_input,
            "Correction CCD must enforce the specified maximum 32 interval evaluations.");

        std::vector<skin_physics::physics::correction_vertex_motion> point_sweep = {{
            read_vec3(vt_crossing.at("input").at("point_before")),
            read_vec3(vt_crossing.at("input").at("point_after"))
        }};
        const auto swept_triangle_before = read_vec3_triplet(
            vt_crossing.at("input").at("triangle_vertices_before"));
        const auto swept_triangle_after = read_vec3_triplet(
            vt_crossing.at("input").at("triangle_vertices_after"));
        std::vector<skin_physics::physics::correction_vertex_motion> triangle_sweep(3);
        for (std::size_t index = 0; index < 3; ++index)
        {
            triangle_sweep[index] = {
                swept_triangle_before[index], swept_triangle_after[index]
            };
        }
        skin_physics::physics::collision_aabb point_bounds;
        skin_physics::physics::collision_aabb triangle_bounds;
        require(skin_physics::physics::build_correction_swept_aabb(
            point_sweep, 0.02f, point_bounds) &&
            skin_physics::physics::build_correction_swept_aabb(
                triangle_sweep, 0.02f, triangle_bounds),
            "Correction Swept AABB generation must accept finite per-primitive vertex motions.");
        require(point_bounds.minimum.x <= 0.25f &&
            point_bounds.maximum.x >= 0.25f &&
            point_bounds.minimum.y <= 0.25f &&
            point_bounds.maximum.y >= 0.25f &&
            point_bounds.minimum.z < -1.0f &&
            point_bounds.maximum.z > 1.0f &&
            triangle_bounds.minimum.x < 0.0f &&
            triangle_bounds.maximum.x > 1.0f &&
            triangle_bounds.minimum.y < 0.0f &&
            triangle_bounds.maximum.y > 1.0f,
            "Per-primitive Correction Swept AABBs must enclose all paths and half-thickness expansion.");

        const skin_physics::physics::collision_aabb physical_bounds = {
            { -0.5f, -0.5f, -0.5f },
            { 0.5f, 0.5f, 0.5f }
        };
        skin_physics::physics::collision_aabb segment_envelope;
        require(skin_physics::physics::union_correction_envelope_aabb(
            physical_bounds, point_bounds, segment_envelope),
            "Physical and correction per-primitive AABBs must form a valid Segment envelope.");
        require(segment_envelope.minimum.x <= physical_bounds.minimum.x &&
            segment_envelope.minimum.x <= point_bounds.minimum.x &&
            segment_envelope.maximum.z >= physical_bounds.maximum.z &&
            segment_envelope.maximum.z >= point_bounds.maximum.z,
            "Segment envelope union must enclose both physical and correction sweeps.");

        using skin_physics::physics::correction_primitive_role;
        std::vector<skin_physics::physics::correction_broadphase_primitive>
            correction_primitives(7);
        correction_primitives[0].stable_id = 100;
        correction_primitives[0].mesh_id = 1;
        correction_primitives[0].primitive_id = 0;
        correction_primitives[0].role = correction_primitive_role::vertex;
        correction_primitives[0].envelope_bounds = point_bounds;
        correction_primitives[0].vertex_stable_ids = { 11 };

        correction_primitives[1].stable_id = 200;
        correction_primitives[1].mesh_id = 1;
        correction_primitives[1].primitive_id = 1;
        correction_primitives[1].role = correction_primitive_role::triangle;
        correction_primitives[1].envelope_bounds = triangle_bounds;
        correction_primitives[1].vertex_stable_ids = { 21, 22, 23 };

        correction_primitives[2].stable_id = 300;
        correction_primitives[2].mesh_id = 1;
        correction_primitives[2].primitive_id = 2;
        correction_primitives[2].role = correction_primitive_role::edge;
        correction_primitives[2].envelope_bounds = point_bounds;
        correction_primitives[2].vertex_stable_ids = { 31, 32 };

        correction_primitives[3].stable_id = 400;
        correction_primitives[3].mesh_id = 1;
        correction_primitives[3].primitive_id = 3;
        correction_primitives[3].role = correction_primitive_role::edge;
        correction_primitives[3].envelope_bounds = triangle_bounds;
        correction_primitives[3].vertex_stable_ids = { 41, 42 };

        // This Vertex overlaps the Triangle AABB but shares a topology Vertex,
        // so it must be excluded before Candidate capacity is evaluated.
        correction_primitives[4].stable_id = 500;
        correction_primitives[4].mesh_id = 1;
        correction_primitives[4].primitive_id = 4;
        correction_primitives[4].role = correction_primitive_role::vertex;
        correction_primitives[4].envelope_bounds = point_bounds;
        correction_primitives[4].vertex_stable_ids = { 21 };

        // The pair overlaps geometrically but is explicitly excluded by the
        // topology owner's one-ring adjacency policy.
        const skin_physics::physics::collision_aabb one_ring_exclusion_bounds = {
            { 4.0f, 4.0f, 4.0f },
            { 4.25f, 4.25f, 4.25f }
        };
        correction_primitives[5].stable_id = 600;
        correction_primitives[5].mesh_id = 1;
        correction_primitives[5].primitive_id = 5;
        correction_primitives[5].role = correction_primitive_role::edge;
        correction_primitives[5].envelope_bounds = one_ring_exclusion_bounds;
        correction_primitives[5].vertex_stable_ids = { 51, 52 };
        correction_primitives[5].excluded_primitive_ids = { { 700, 1, 6 } };

        correction_primitives[6].stable_id = 700;
        correction_primitives[6].mesh_id = 1;
        correction_primitives[6].primitive_id = 6;
        correction_primitives[6].role = correction_primitive_role::edge;
        correction_primitives[6].envelope_bounds = one_ring_exclusion_bounds;
        correction_primitives[6].vertex_stable_ids = { 61, 62 };
        skin_physics::physics::broadphase_settings correction_broadphase_settings;
        correction_broadphase_settings.cell_size = 0.25f;
        correction_broadphase_settings.contact_thickness = 0.0f;
        correction_broadphase_settings.cell_record_capacity = 4096;
        correction_broadphase_settings.candidate_capacity = 2;
        correction_broadphase_settings.max_cells_per_primitive = 4096;
        correction_broadphase_settings.self_collision_enabled = true;
        const auto correction_broadphase =
            skin_physics::physics::generate_correction_broadphase_candidates(
                correction_primitives, {}, correction_broadphase_settings);
        require(correction_broadphase.success &&
            correction_broadphase.required_candidates == 2 &&
            correction_broadphase.candidates.size() == 2,
            "Correction Broadphase must map Spatial Hash pairs to VT / EE and remove topology-adjacent pairs before capacity counting.");
        require(correction_broadphase.broadphase_candidate_count == 2 &&
            correction_broadphase.required_candidates == 2,
            "Topology filtering must happen before Candidate Capacity accounting, matching the GPU Count pass.");
        require(std::any_of(
            correction_broadphase.candidates.begin(),
            correction_broadphase.candidates.end(),
            [](const skin_physics::physics::correction_candidate_pair_key& key)
            {
                return key.contact_type ==
                    skin_physics::physics::correction_contact_type::vertex_triangle &&
                    key.stable_id_a == 100 && key.stable_id_b == 200;
            }),
            "Correction Broadphase must preserve the canonical VT stable-ID pair.");
        require(std::any_of(
            correction_broadphase.candidates.begin(),
            correction_broadphase.candidates.end(),
            [](const skin_physics::physics::correction_candidate_pair_key& key)
            {
                return key.contact_type ==
                    skin_physics::physics::correction_contact_type::edge_edge &&
                    key.stable_id_a == 300 && key.stable_id_b == 400;
            }),
            "Correction Broadphase must preserve the canonical EE stable-ID pair.");

        require(std::none_of(
            correction_broadphase.candidates.begin(),
            correction_broadphase.candidates.end(),
            [](const skin_physics::physics::correction_candidate_pair_key& key)
            {
                return (key.stable_id_a == 200 && key.stable_id_b == 500) ||
                    (key.stable_id_a == 500 && key.stable_id_b == 200) ||
                    (key.stable_id_a == 600 && key.stable_id_b == 700);
            }),
            "Shared-vertex and explicit one-ring topology pairs must be excluded before Canonical Candidate output.");

        auto duplicate_primitive_identity =
            correction_primitives;
        duplicate_primitive_identity[4].stable_id =
            correction_primitives[1].stable_id;
        duplicate_primitive_identity[4].mesh_id =
            correction_primitives[1].mesh_id;
        duplicate_primitive_identity[4].primitive_id =
            correction_primitives[1].primitive_id;
        const auto duplicate_identity_broadphase =
            skin_physics::physics::generate_correction_broadphase_candidates(
                duplicate_primitive_identity, {},
                correction_broadphase_settings);
        require(!duplicate_identity_broadphase.success &&
            !duplicate_identity_broadphase.diagnostics.empty() &&
            duplicate_identity_broadphase.diagnostics[0].code ==
                skin_physics::physics::collision_error::invalid_geometry,
            "Standalone Correction Broadphase must reject duplicate Stable / Mesh / Primitive identities.");

        skin_physics::physics::broadphase_settings double_expansion_settings =
            correction_broadphase_settings;
        double_expansion_settings.contact_thickness = 0.02f;
        const auto double_expansion_result =
            skin_physics::physics::generate_correction_broadphase_candidates(
                correction_primitives, {}, double_expansion_settings);
        require(!double_expansion_result.success &&
            !double_expansion_result.diagnostics.empty() &&
            double_expansion_result.diagnostics[0].code ==
                skin_physics::physics::collision_error::invalid_settings,
            "Correction Broadphase must reject a non-zero second thickness expansion.");

        skin_physics::physics::broadphase_settings correction_overflow_settings =
            correction_broadphase_settings;
        correction_overflow_settings.candidate_capacity = 0;
        const auto correction_overflow =
            skin_physics::physics::generate_correction_broadphase_candidates(
                correction_primitives, {}, correction_overflow_settings);
        require(!correction_overflow.success && correction_overflow.overflow &&
            correction_overflow.candidates.empty(),
            "Correction Broadphase overflow must not return a partial typed Candidate Set.");

        std::vector<skin_physics::physics::correction_vertex_motion> empty_sweep;
        require(!skin_physics::physics::build_correction_swept_aabb(
            empty_sweep, 0.02f, point_bounds),
            "An empty correction primitive must not produce an AABB.");

        auto non_finite_sweep = triangle_sweep;
        non_finite_sweep[2].after.y =
            std::numeric_limits<float>::quiet_NaN();
        skin_physics::physics::collision_aabb correction_bounds;
        require(!skin_physics::physics::build_correction_swept_aabb(
            non_finite_sweep, 0.02f, correction_bounds),
            "A non-finite correction vertex must fail AABB construction.");

        using skin_physics::physics::correction_contact_type;
        const auto candidate_vt = skin_physics::physics::make_canonical_correction_candidate_key(
            correction_contact_type::vertex_triangle, 700, 7, 12, 200, 2, 4);
        const auto candidate_vt_reversed =
            skin_physics::physics::make_canonical_correction_candidate_key(
                correction_contact_type::vertex_triangle, 200, 2, 4, 700, 7, 12);
        const auto candidate_ee =
            skin_physics::physics::make_canonical_correction_candidate_key(
                correction_contact_type::edge_edge, 700, 7, 12, 200, 2, 4);
        const auto candidate_union =
            skin_physics::physics::merge_correction_candidate_union(
                {}, { candidate_vt, candidate_vt_reversed, candidate_ee }, 2);
        require(candidate_union.success && !candidate_union.overflow &&
            candidate_union.required_count == 2 &&
            candidate_union.candidates.size() == 2,
            "Canonical Candidate Union must sort, deduplicate reversed pairs, and preserve contact type.");

        const auto segment_candidate_union =
            skin_physics::physics::merge_correction_candidate_union(
                { candidate_ee }, { candidate_vt, candidate_vt_reversed }, 2);
        require(segment_candidate_union.success &&
            segment_candidate_union.required_count == 2 &&
            segment_candidate_union.candidates[0] == candidate_vt &&
            segment_candidate_union.candidates[1] == candidate_ee,
            "Each correction epoch must retain existing physical candidates and append canonical new candidates.");

        auto segment_state_result =
            skin_physics::physics::create_correction_segment_candidate_state(
                1001, { candidate_vt }, 2);
        require(segment_state_result.success &&
            segment_state_result.state.candidate_generation == 1 &&
            segment_state_result.state.position_writer_epoch_generation == 0 &&
            segment_state_result.state.candidates.size() == 1 &&
            !segment_state_result.state.contact_set_current &&
            !segment_state_result.state.vertex_contact_csr_current,
            "A new Integration Segment must begin with its physical candidates and unbuilt Contact/CSR generations.");

        const auto make_vt_discrete_input =
            [](const skin_physics::physics::correction_candidate_pair_key& key)
        {
            skin_physics::physics::correction_discrete_contact_input input;
            input.key = key;
            input.vertex_stable_ids = { 11, 12, 13, 14 };
            input.positions = {
                skin_physics::asset::vec3{ 0.25f, 0.25f, 0.005f },
                skin_physics::asset::vec3{ 0.0f, 0.0f, 0.0f },
                skin_physics::asset::vec3{ 1.0f, 0.0f, 0.0f },
                skin_physics::asset::vec3{ 0.0f, 1.0f, 0.0f }
            };
            input.contact_thickness = 0.01f;
            return input;
        };
        const auto make_ee_discrete_input =
            [](const skin_physics::physics::correction_candidate_pair_key& key)
        {
            skin_physics::physics::correction_discrete_contact_input input;
            input.key = key;
            input.vertex_stable_ids = { 21, 22, 23, 24 };
            input.positions = {
                skin_physics::asset::vec3{ 0.0f, 0.0f, 0.0f },
                skin_physics::asset::vec3{ 1.0f, 0.0f, 0.0f },
                skin_physics::asset::vec3{ 0.5f, -0.5f, 0.0f },
                skin_physics::asset::vec3{ 0.5f, 0.5f, 0.0f }
            };
            input.contact_thickness = 0.02f;
            return input;
        };

        auto segment_state = segment_state_result.state;
        const std::vector<skin_physics::physics::correction_segment_vertex_snapshot>
            segment_start_vertices = {
                { 11u, { 0.25f, 0.25f, 0.0f } },
                { 12u, { 0.0f, 0.0f, 0.0f } },
                { 13u, { 1.0f, 0.0f, 0.0f } },
                { 14u, { 0.0f, 1.0f, 0.0f } },
                { 21u, { 0.0f, 0.0f, 0.0f } },
                { 22u, { 1.0f, 0.0f, 0.0f } },
                { 23u, { 0.5f, -0.5f, 0.0f } },
                { 24u, { 0.5f, 0.5f, 0.0f } }
            };
        require(skin_physics::physics::initialize_correction_segment_geometry(
                segment_state, 1001u, segment_start_vertices, physical_bounds),
            "Segment Commit Gate fixtures must initialize the immutable Segment-start Vertex Snapshot and cumulative broadphase envelope.");
        std::vector<skin_physics::physics::correction_discrete_contact_input>
            vt_discrete_inputs = { make_vt_discrete_input(candidate_vt) };
        const auto initial_contact_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 1, 0, vt_discrete_inputs);
        require(initial_contact_refresh.success &&
            initial_contact_refresh.candidate_count == 1 &&
            initial_contact_refresh.active_contact_count == 1 &&
            segment_state.contact_set_current &&
            segment_state.vertex_contact_csr_current &&
            segment_state.contact_set.size() == 1 &&
            segment_state.vertex_contact_csr.vertex_stable_ids.size() == 4 &&
            segment_state.vertex_contact_csr.offsets.size() == 5 &&
            segment_state.vertex_contact_csr.contact_indices.size() == 4,
            "Final Discrete VT Narrowphase must build the active Contact Set and four-participant Vertex-to-Contact CSR for the current generations.");

        const auto contact_capacity_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 1, 0, vt_discrete_inputs, 1.0e-7f, 0, 16);
        require(!contact_capacity_refresh.success &&
            contact_capacity_refresh.capacity_overflow &&
            contact_capacity_refresh.required_contact_count == 1 &&
            segment_state.contact_set_current &&
            segment_state.contact_set.size() == 1 &&
            segment_state.vertex_contact_csr.current,
            "Contact Set Capacity overflow must report the required count and preserve the prior published Contact Set / CSR atomically.");

        const auto csr_capacity_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 1, 0, vt_discrete_inputs, 1.0e-7f, 16, 0);
        require(!csr_capacity_refresh.success &&
            csr_capacity_refresh.capacity_overflow &&
            csr_capacity_refresh.required_vertex_contact_indices == 4 &&
            segment_state.contact_set_current &&
            segment_state.contact_set.size() == 1 &&
            segment_state.vertex_contact_csr.current &&
            segment_state.vertex_contact_csr.contact_indices.size() == 4,
            "Vertex-to-Contact CSR Capacity overflow must preserve both prior published structures atomically.");

        const auto stale_initial_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 0, 0, vt_discrete_inputs);
        require(!stale_initial_refresh.success &&
            stale_initial_refresh.generation_mismatch &&
            segment_state.contact_set_current &&
            segment_state.contact_set.size() == 1,
            "A stale Candidate Generation refresh must not overwrite a previously valid Contact Set.");

        const auto begin_epoch_1 =
            skin_physics::physics::begin_correction_position_writer_epoch(
                segment_state, 1001, 1);
        require(begin_epoch_1.success && begin_epoch_1.changed &&
            segment_state.last_position_writer_epoch_id == 1 &&
            segment_state.position_writer_epoch_generation == 1 &&
            !segment_state.contact_set_current &&
            !segment_state.vertex_contact_csr_current &&
            !segment_state.vertex_contact_csr.current,
            "Every Position-Writer Epoch must invalidate Contact Set and CSR freshness even when its candidate set is unchanged.");

        const auto duplicate_epoch_update =
            skin_physics::physics::merge_correction_position_writer_epoch_candidates(
                segment_state, 1001, 1, { candidate_vt_reversed }, 2);
        require(duplicate_epoch_update.success &&
            !duplicate_epoch_update.changed &&
            segment_state.candidate_generation == 1 &&
            !segment_state.contact_set_current &&
            !segment_state.vertex_contact_csr_current &&
            !segment_state.vertex_contact_csr.current,
            "An Epoch that adds only duplicate canonical pairs must not advance Candidate Generation or falsely mark geometry-dependent data current.");

        const auto stale_epoch_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 1, 0, vt_discrete_inputs);
        require(!stale_epoch_refresh.success &&
            stale_epoch_refresh.generation_mismatch &&
            !segment_state.contact_set_current &&
            !segment_state.vertex_contact_csr_current,
            "A Contact Set / CSR completion from the pre-Epoch Position Generation must be rejected.");
        const auto epoch_1_contact_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 1, 1, vt_discrete_inputs);
        require(epoch_1_contact_refresh.success &&
            segment_state.contact_set_current &&
            segment_state.vertex_contact_csr_current &&
            segment_state.contact_set.size() == 1,
            "Contact Set and CSR refresh must run Final Discrete Narrowphase for the latest Position-Writer Epoch.");

        const auto begin_epoch_2 =
            skin_physics::physics::begin_correction_position_writer_epoch(
                segment_state, 1001, 2);
        require(begin_epoch_2.success &&
            segment_state.position_writer_epoch_generation == 2 &&
            !segment_state.contact_set_current &&
            !segment_state.vertex_contact_csr_current &&
            !segment_state.vertex_contact_csr.current,
            "A later position write must invalidate Contact Set / CSR even when no new broadphase pair is emitted.");

        const auto stale_epoch_refresh_2 =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 1, 1, vt_discrete_inputs);
        require(!stale_epoch_refresh_2.success &&
            stale_epoch_refresh_2.generation_mismatch &&
            !segment_state.contact_set_current,
            "A stale Position-Writer Epoch rebuild completion must not clear a newer geometry generation.");

        const auto stale_epoch_candidate_update =
            skin_physics::physics::merge_correction_position_writer_epoch_candidates(
                segment_state, 1001, 1, { candidate_ee }, 2);
        require(!stale_epoch_candidate_update.success &&
            stale_epoch_candidate_update.invalid_epoch_order &&
            segment_state.candidate_generation == 1 &&
            segment_state.candidates.size() == 1,
            "A delayed Candidate result from a previous Position-Writer Epoch must not mutate the active Segment Candidate Set.");

        const auto new_epoch_update =
            skin_physics::physics::merge_correction_position_writer_epoch_candidates(
                segment_state, 1001, 2, { candidate_ee }, 2);
        require(new_epoch_update.success && new_epoch_update.changed &&
            segment_state.candidate_generation == 2 &&
            segment_state.candidates.size() == 2 &&
            !segment_state.contact_set_current &&
            !segment_state.vertex_contact_csr_current &&
            !segment_state.vertex_contact_csr.current,
            "A newly discovered Epoch candidate must advance the Candidate Generation and invalidate Contact Set / CSR.");

        std::vector<skin_physics::physics::correction_discrete_contact_input>
            complete_discrete_inputs = {
                make_ee_discrete_input(candidate_ee),
                make_vt_discrete_input(candidate_vt)
            };
        const auto missing_candidate_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 2, 2, { make_vt_discrete_input(candidate_vt) });
        require(!missing_candidate_refresh.success &&
            missing_candidate_refresh.invalid_input &&
            !segment_state.contact_set_current &&
            !segment_state.vertex_contact_csr_current &&
            segment_state.contact_set.size() == 1,
            "Final Discrete Narrowphase must cover every Candidate Pair and preserve prior data on incomplete input.");
        const auto latest_contact_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                segment_state, 2, 2, complete_discrete_inputs);
        require(latest_contact_refresh.success &&
            latest_contact_refresh.candidate_count == 2 &&
            latest_contact_refresh.active_contact_count == 2 &&
            segment_state.contact_set_current &&
            segment_state.vertex_contact_csr_current &&
            segment_state.contact_set.size() == 2 &&
            segment_state.vertex_contact_csr.vertex_stable_ids.size() == 8 &&
            segment_state.vertex_contact_csr.offsets.size() == 9 &&
            segment_state.vertex_contact_csr.contact_indices.size() == 8,
            "Final Discrete Narrowphase must refresh all active VT / EE Contacts and build the complete Vertex-to-Contact CSR.");

        const auto valid_segment_commit_gate =
            skin_physics::physics::validate_correction_segment_commit_gate(
                segment_state);
        require(valid_segment_commit_gate.commit_allowed &&
            valid_segment_commit_gate.status ==
                skin_physics::physics::correction_segment_commit_gate_status::ready &&
            valid_segment_commit_gate.candidate_generation == 2 &&
            valid_segment_commit_gate.position_writer_epoch_generation == 2,
            "The Segment Commit Gate must allow publication only after Candidate, Active Contact Set and CSR generations are mutually current.");

        auto stale_csr_payload_state = segment_state;
        stale_csr_payload_state.vertex_contact_csr.current = false;
        const auto stale_csr_commit_gate =
            skin_physics::physics::validate_correction_segment_commit_gate(
                stale_csr_payload_state);
        require(!stale_csr_commit_gate.commit_allowed &&
            stale_csr_commit_gate.status ==
                skin_physics::physics::correction_segment_commit_gate_status::rejected,
            "The Segment Commit Gate must inspect the CSR payload freshness flag as well as its outer Generation state.");

        auto invalid_csr_index_state = segment_state;
        invalid_csr_index_state.vertex_contact_csr.contact_indices[0] =
            std::numeric_limits<uint32_t>::max();
        const auto invalid_csr_commit_gate =
            skin_physics::physics::validate_correction_segment_commit_gate(
                invalid_csr_index_state);
        require(!invalid_csr_commit_gate.commit_allowed &&
            invalid_csr_commit_gate.status ==
                skin_physics::physics::correction_segment_commit_gate_status::rejected,
            "The Segment Commit Gate must reject CSR indices outside the Active Contact Set.");

        const auto transient_candidate =
            skin_physics::physics::make_canonical_correction_candidate_key(
                correction_contact_type::vertex_triangle, 1500, 15, 150,
                1600, 16, 160);
        auto transient_state_result =
            skin_physics::physics::create_correction_segment_candidate_state(
                2001, { transient_candidate }, 4);
        require(transient_state_result.success &&
            transient_state_result.state.candidate_generation == 1,
            "An initialized Segment Candidate Set must have a non-zero generation even when its input set is empty or non-empty.");
        auto transient_state = transient_state_result.state;
        auto transient_input = make_vt_discrete_input(transient_candidate);
        transient_input.positions[0].z = 0.5f;
        const auto transient_refresh =
            skin_physics::physics::refresh_correction_segment_contact_set_and_csr(
                transient_state, 1, 0, { transient_input });
        require(transient_refresh.success &&
            transient_refresh.candidate_count == 1 &&
            transient_refresh.active_contact_count == 0 &&
            transient_state.candidates.size() == 1 &&
            transient_state.contact_set.empty() &&
            transient_state.vertex_contact_csr.vertex_stable_ids.empty() &&
            transient_state.vertex_contact_csr.offsets.size() == 1 &&
            transient_state.vertex_contact_csr.offsets[0] == 0,
            "A same-side or transient Candidate must remain in the Candidate Union while staying out of the Active Contact Set and Vertex CSR.");

        auto empty_state_result =
            skin_physics::physics::create_correction_segment_candidate_state(
                2002, {}, 0);
        require(empty_state_result.success &&
            empty_state_result.state.candidate_generation == 1 &&
            empty_state_result.state.candidates.empty(),
            "An empty initial Candidate Set must still carry a valid non-zero generation.");

        const auto wrong_segment_begin =
            skin_physics::physics::begin_correction_position_writer_epoch(
                segment_state, 1002, 3);
        const auto wrong_segment_update =
            skin_physics::physics::merge_correction_position_writer_epoch_candidates(
                segment_state, 1002, 3, {}, 2);
        require(!wrong_segment_begin.success && wrong_segment_begin.wrong_segment &&
            !wrong_segment_update.success && wrong_segment_update.wrong_segment &&
            segment_state.base_integration_segment_id == 1001 &&
            segment_state.candidate_generation == 2 &&
            segment_state.position_writer_epoch_generation == 2,
            "Position-Writer Epochs and Candidate Pairs from another base_integration_segment_id must be rejected.");

        const auto duplicate_epoch_id =
            skin_physics::physics::begin_correction_position_writer_epoch(
                segment_state, 1001, 2);
        require(!duplicate_epoch_id.success && duplicate_epoch_id.invalid_epoch_order &&
            segment_state.position_writer_epoch_generation == 2,
            "A repeated or out-of-order Position-Writer Epoch ID must not mutate Segment State.");

        const auto begin_epoch_3 =
            skin_physics::physics::begin_correction_position_writer_epoch(
                segment_state, 1001, 3);
        require(begin_epoch_3.success &&
            !segment_state.contact_set_current &&
            !segment_state.vertex_contact_csr_current,
            "Starting the next Position-Writer Epoch must mark prior Contact Set / CSR data stale.");

        const auto new_capacity_candidate =
            skin_physics::physics::make_canonical_correction_candidate_key(
                correction_contact_type::vertex_triangle, 900, 9, 90, 800, 8, 80);
        const auto state_capacity_overflow =
            skin_physics::physics::merge_correction_position_writer_epoch_candidates(
                segment_state, 1001, 3, { new_capacity_candidate }, 2);
        require(!state_capacity_overflow.success &&
            state_capacity_overflow.capacity_overflow &&
            segment_state.candidates.size() == 2 &&
            segment_state.candidate_generation == 2 &&
            segment_state.position_writer_epoch_generation == 3 &&
            !segment_state.contact_set_current &&
            !segment_state.vertex_contact_csr_current,
            "Candidate Capacity Retry must preserve the previous Candidate Set while keeping geometry-dependent Contact/CSR data stale for the active Epoch.");

        auto generation_exhausted_state = segment_state;
        generation_exhausted_state.candidate_generation =
            std::numeric_limits<uint64_t>::max();
        const auto generation_overflow =
            skin_physics::physics::merge_correction_position_writer_epoch_candidates(
                generation_exhausted_state, 1001, 3, { new_capacity_candidate }, 4);
        require(!generation_overflow.success &&
            generation_overflow.generation_overflow &&
            generation_exhausted_state.candidates.size() == 2 &&
            generation_exhausted_state.position_writer_epoch_generation == 3,
            "Candidate Generation overflow must fail without replacing the prior Segment Candidate Set.");

        const auto candidate_overflow =
            skin_physics::physics::merge_correction_candidate_union(
                { candidate_vt }, { candidate_ee }, 1);
        require(!candidate_overflow.success && candidate_overflow.overflow &&
            candidate_overflow.required_count == 2 &&
            candidate_overflow.candidates.empty(),
            "Candidate Union capacity overflow must return no partial replacement set.");

        const auto make_vt_epoch_primitives =
            [](const test_json::value& input,
                uint64_t point_stable_id,
                uint64_t triangle_stable_id)
                -> std::vector<skin_physics::physics::correction_primitive_epoch_input>
        {
            std::vector<skin_physics::physics::correction_primitive_epoch_input> result;
            skin_physics::physics::correction_primitive_epoch_input point;
            point.stable_id = point_stable_id;
            point.mesh_id = 1;
            point.primitive_id = 0;
            point.role = skin_physics::physics::correction_primitive_role::vertex;
            point.vertex_stable_ids.push_back(point_stable_id * 10u + 1u);
            point.vertex_motions.push_back({
                read_vec3(input.at("point_before")),
                read_vec3(input.at("point_after"))
            });
            point.segment_start_positions.push_back(
                read_vec3(input.at("point_before")));
            result.push_back(point);

            const auto triangle_before = read_vec3_triplet(
                input.at("triangle_vertices_before"));
            const auto triangle_after = read_vec3_triplet(
                input.at("triangle_vertices_after"));
            skin_physics::physics::correction_primitive_epoch_input triangle;
            triangle.stable_id = triangle_stable_id;
            triangle.mesh_id = 2;
            triangle.primitive_id = 0;
            triangle.role = skin_physics::physics::correction_primitive_role::triangle;
            for (std::size_t index = 0; index < 3; ++index)
            {
                triangle.vertex_stable_ids.push_back(
                    triangle_stable_id * 10u + static_cast<uint64_t>(index) + 1u);
                triangle.vertex_motions.push_back({
                    triangle_before[index], triangle_after[index]
                });
                triangle.segment_start_positions.push_back(triangle_before[index]);
            }
            result.push_back(triangle);
            return result;
        };

        const std::vector<skin_physics::physics::broadphase_mesh_pair>
            epoch_mesh_pairs = { { 1, 2, true } };
        skin_physics::physics::broadphase_settings epoch_broadphase_settings;
        epoch_broadphase_settings.cell_size = 0.25f;
        epoch_broadphase_settings.contact_thickness = 0.0f;
        epoch_broadphase_settings.max_cells_per_primitive = 4096;
        epoch_broadphase_settings.cell_record_capacity = 4096;
        epoch_broadphase_settings.candidate_capacity = 16;
        epoch_broadphase_settings.self_collision_enabled = false;

        auto crossing_epoch_state_result =
            skin_physics::physics::create_correction_segment_candidate_state(
                3001, {}, 16);
        require(crossing_epoch_state_result.success,
            "CPU Position-Writer Epoch integration test must initialize an empty Segment Candidate State.");
        auto crossing_epoch_state = crossing_epoch_state_result.state;
        const auto crossing_epoch_primitives = make_vt_epoch_primitives(
            vt_crossing.at("input"), 3100, 3200);
        require(initialize_test_correction_segment_geometry(
            crossing_epoch_state, 3001, crossing_epoch_primitives,
            static_cast<float>(vt_crossing.at("input").at(
                "contact_thickness_m").as_number())),
            "Crossing Epoch must initialize its immutable Segment-start and Physical Sweep snapshots.");
        const auto crossing_epoch = 
            skin_physics::physics::process_correction_position_writer_epoch(
                crossing_epoch_state, 3001, 1, crossing_epoch_primitives,
                epoch_mesh_pairs, epoch_broadphase_settings,
                read_correction_ccd_settings(vt_crossing.at("input")));
        require(crossing_epoch.status ==
                skin_physics::physics::correction_writer_epoch_status::rejected &&
            !crossing_epoch.state_committed &&
            crossing_epoch.correction_ccd_results.size() == 1 &&
            crossing_epoch.correction_ccd_results[0].result.status ==
                correction_ccd_status::path_crossing &&
            crossing_epoch_state.position_writer_epoch_generation == 0 &&
            crossing_epoch_state.candidates.empty(),
            "A Position-Writer Epoch that crosses the latched side must reject atomically without appending Candidates or advancing Segment state.");

        const auto transient_epoch_primitives = make_vt_epoch_primitives(
            transient.at("input"), 3300, 3400);
        auto transient_epoch_state_result =
            skin_physics::physics::create_correction_segment_candidate_state(
                3002, {}, 16);
        auto transient_epoch_state = transient_epoch_state_result.state;
        require(initialize_test_correction_segment_geometry(
            transient_epoch_state, 3002, transient_epoch_primitives,
            static_cast<float>(transient.at("input").at(
                "contact_thickness_m").as_number())),
            "Transient Epoch must initialize its immutable Segment-start and Physical Sweep snapshots.");
        const auto transient_epoch =
            skin_physics::physics::process_correction_position_writer_epoch(
                transient_epoch_state, 3002, 1, transient_epoch_primitives,
                epoch_mesh_pairs, epoch_broadphase_settings,
                read_correction_ccd_settings(transient.at("input")));
        require(transient_epoch.status ==
                skin_physics::physics::correction_writer_epoch_status::accepted &&
            transient_epoch.state_committed &&
            transient_epoch.correction_ccd_results.size() == 1 &&
            transient_epoch.correction_ccd_results[0].result.status ==
                correction_ccd_status::hit &&
            transient_epoch.active_contacts.empty() &&
            transient_epoch_state.candidates.size() == 1 &&
            transient_epoch_state.candidate_generation == 2 &&
            transient_epoch_state.position_writer_epoch_generation == 1 &&
            transient_epoch_state.contact_set_current &&
            transient_epoch_state.vertex_contact_csr_current &&
            transient_epoch_state.contact_set.empty() &&
            transient_epoch_state.vertex_contact_csr.offsets ==
                std::vector<uint32_t>{ 0u },
            "A same-side transient proximity must remain in the Segment Candidate Union without becoming an endpoint Active Contact.");

        const auto endpoint_epoch_primitives = make_vt_epoch_primitives(
            endpoint.at("input"), 3500, 3600);
        auto endpoint_epoch_state_result =
            skin_physics::physics::create_correction_segment_candidate_state(
                3003, {}, 16);
        auto endpoint_epoch_state = endpoint_epoch_state_result.state;
        require(initialize_test_correction_segment_geometry(
            endpoint_epoch_state, 3003, endpoint_epoch_primitives,
            static_cast<float>(endpoint.at("input").at(
                "contact_thickness_m").as_number())),
            "Endpoint Epoch must initialize its immutable Segment-start and Physical Sweep snapshots.");
        const auto endpoint_epoch =
            skin_physics::physics::process_correction_position_writer_epoch(
                endpoint_epoch_state, 3003, 1, endpoint_epoch_primitives,
                epoch_mesh_pairs, epoch_broadphase_settings,
                read_correction_ccd_settings(endpoint.at("input")));
        require(endpoint_epoch.status ==
                skin_physics::physics::correction_writer_epoch_status::accepted &&
            endpoint_epoch.state_committed &&
            endpoint_epoch.active_contacts.size() == 1 &&
            endpoint_epoch_state.candidates.size() == 1,
            "A Correction Hit that remains active at s=1 must be returned as an endpoint Contact candidate.");

        const auto miss_epoch_primitives = make_vt_epoch_primitives(
            miss.at("input"), 3700, 3800);
        auto miss_epoch_state_result =
            skin_physics::physics::create_correction_segment_candidate_state(
                3004, {}, 16);
        auto miss_epoch_state = miss_epoch_state_result.state;
        require(initialize_test_correction_segment_geometry(
            miss_epoch_state, 3004, miss_epoch_primitives,
            static_cast<float>(miss.at("input").at(
                "contact_thickness_m").as_number())),
            "Certified-miss Epoch must initialize its immutable Segment-start and Physical Sweep snapshots.");
        const auto miss_epoch =
            skin_physics::physics::process_correction_position_writer_epoch(
                miss_epoch_state, 3004, 1, miss_epoch_primitives,
                epoch_mesh_pairs, epoch_broadphase_settings,
                read_correction_ccd_settings(miss.at("input")));
        require(miss_epoch.status ==
                skin_physics::physics::correction_writer_epoch_status::accepted &&
            miss_epoch.state_committed &&
            miss_epoch.correction_ccd_results.empty() &&
            miss_epoch_state.candidates.empty() &&
            miss_epoch_state.position_writer_epoch_generation == 1 &&
            miss_epoch_state.contact_set_current &&
            miss_epoch_state.vertex_contact_csr_current &&
            miss_epoch_state.vertex_contact_csr.offsets ==
                std::vector<uint32_t>{ 0u },
            "A certified Broadphase miss must still advance the Position-Writer Epoch while leaving the Candidate Union empty.");

        skin_physics::physics::broadphase_settings epoch_capacity_settings =
            epoch_broadphase_settings;
        epoch_capacity_settings.candidate_capacity = 0;
        auto blocked_epoch_state_result =
            skin_physics::physics::create_correction_segment_candidate_state(
                3005, {}, 16);
        auto blocked_epoch_state = blocked_epoch_state_result.state;
        require(initialize_test_correction_segment_geometry(
            blocked_epoch_state, 3005, crossing_epoch_primitives,
            static_cast<float>(vt_crossing.at("input").at(
                "contact_thickness_m").as_number())),
            "Capacity-blocked Epoch must retain an initialized immutable Segment-start and Physical Sweep snapshot.");
        const auto blocked_epoch =
            skin_physics::physics::process_correction_position_writer_epoch(
                blocked_epoch_state, 3005, 1, crossing_epoch_primitives,
                epoch_mesh_pairs, epoch_capacity_settings,
                read_correction_ccd_settings(vt_crossing.at("input")));
        require(blocked_epoch.status ==
                skin_physics::physics::correction_writer_epoch_status::blocked &&
            !blocked_epoch.state_committed &&
            blocked_epoch.capacity_overflow &&
            blocked_epoch_state.position_writer_epoch_generation == 0 &&
            blocked_epoch_state.candidates.empty(),
            "Correction Broadphase Capacity exhaustion must BLOCK the same Epoch without partial Candidate or Generation mutation.");

        skin_physics::physics::correction_ccd_settings limited_settings =
            read_correction_ccd_settings(vt_crossing.at("input"));
        limited_settings.maximum_interval_evaluations = 2;
        const auto limited_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                make_vt_correction_motion(vt_crossing.at("input")), limited_settings);
        require(limited_result.status == correction_ccd_status::unresolved,
            "Interval budget exhaustion must fail closed as Unresolved.");

        auto ambiguous_side_motion = make_vt_correction_motion(vt_crossing.at("input"));
        ambiguous_side_motion.segment_start_point = { 0.25f, 0.25f, 0.0f };
        const auto ambiguous_side_result =
            skin_physics::physics::detect_vertex_triangle_correction_motion(
                ambiguous_side_motion,
                read_correction_ccd_settings(vt_crossing.at("input")));
        require(ambiguous_side_result.status == correction_ccd_status::unresolved,
            "A Segment-start Pair exactly on the separating plane must not invent an allowed side.");
    }

    void test_phase12_correction_position_writer_epoch_pipeline()
    {
        using namespace skin_physics::physics;
        const test_json::value fixture = test_json::load(
            test_support::fixture_path(
                "design_review/correction_sweep_broadphase.json"));

        const test_json::value& endpoint_case = find_correction_fixture_case(
            fixture, "DCR_49_VT_ENDPOINT_ACTIVE_CONTACT");
        correction_ccd_settings endpoint_settings =
            read_correction_ccd_settings(endpoint_case.at("input"));
        auto endpoint_state_result =
            create_correction_segment_candidate_state(7001, {}, 16);
        require(endpoint_state_result.success,
            "Position-Writer Epoch endpoint test must initialize its Segment Candidate State.");
        auto endpoint_state = endpoint_state_result.state;
        const auto endpoint_epoch_primitives =
            make_vt_position_writer_epoch_primitives(endpoint_case.at("input"));
        require(initialize_test_correction_segment_geometry(
            endpoint_state, 7001, endpoint_epoch_primitives,
            endpoint_settings.contact_thickness),
            "Endpoint-active Epoch must initialize its immutable Segment-start and Physical Sweep snapshots.");
        const correction_writer_epoch_result endpoint_epoch =
            process_correction_position_writer_epoch(
                endpoint_state, 7001, 1, endpoint_epoch_primitives,
                {}, make_correction_epoch_broadphase_settings(),
                endpoint_settings);
        const std::string endpoint_epoch_ccd_diagnostic =
            endpoint_epoch.correction_ccd_results.empty()
                ? "none"
                : endpoint_epoch.correction_ccd_results[0].result.diagnostic;
        require(endpoint_epoch.status == correction_writer_epoch_status::accepted &&
            endpoint_epoch.state_committed &&
            endpoint_state.last_position_writer_epoch_id == 1 &&
            endpoint_state.position_writer_epoch_generation == 1 &&
            endpoint_state.candidate_generation == 2 &&
            endpoint_state.candidates.size() == 1,
            std::string(
                "A valid VT Position-Writer Epoch must atomically commit its correction and endpoint candidate union. status=") +
                std::to_string(static_cast<int>(endpoint_epoch.status)) +
                " committed=" + (endpoint_epoch.state_committed ? "true" : "false") +
                " diagnostic=" + endpoint_epoch.diagnostic +
                " ccd_diagnostic=" + endpoint_epoch_ccd_diagnostic +
                " candidates=" + std::to_string(endpoint_epoch.required_candidates) +
                " contacts=" + std::to_string(endpoint_epoch.required_contacts));
        require(endpoint_epoch.correction_ccd_results.size() == 1 &&
            endpoint_epoch.correction_ccd_results[0].result.status ==
                correction_ccd_status::hit,
            "The Epoch pipeline must execute Correction Motion CCD for the correction-swept VT pair.");
        require(endpoint_epoch.active_contacts.size() == 1 &&
            endpoint_state.contact_set.size() == 1 &&
            endpoint_state.contact_set_current &&
            endpoint_state.vertex_contact_csr_current,
            "An endpoint-active VT pair must enter the rebuilt Contact Set and Vertex-to-Contact CSR.");
        require(endpoint_state.vertex_contact_csr.vertex_stable_ids.size() == 4 &&
            endpoint_state.vertex_contact_csr.offsets.size() == 5 &&
            endpoint_state.vertex_contact_csr.contact_indices.size() == 4 &&
            endpoint_state.vertex_contact_csr.current,
            "The Epoch pipeline must produce a complete CSR for the active VT Contact.");

        const test_json::value& transient_case = find_correction_fixture_case(
            fixture, "DCR_49_VT_SAME_SIDE_TRANSIENT_PROXIMITY");
        correction_ccd_settings transient_settings =
            read_correction_ccd_settings(transient_case.at("input"));
        auto transient_state_result =
            create_correction_segment_candidate_state(7002, {}, 16);
        auto transient_state = transient_state_result.state;
        const auto transient_epoch_primitives =
            make_vt_position_writer_epoch_primitives(transient_case.at("input"));
        require(initialize_test_correction_segment_geometry(
            transient_state, 7002, transient_epoch_primitives,
            transient_settings.contact_thickness),
            "Transient pipeline Epoch must initialize its Segment-start Snapshot and Physical Sweep Envelope.");
        const correction_writer_epoch_result transient_epoch =
            process_correction_position_writer_epoch(
                transient_state, 7002, 1, transient_epoch_primitives,
                {}, make_correction_epoch_broadphase_settings(),
                transient_settings);
        require(transient_epoch.status == correction_writer_epoch_status::accepted &&
            transient_epoch.correction_ccd_results.size() == 1 &&
            transient_epoch.correction_ccd_results[0].result.status ==
                correction_ccd_status::hit &&
            transient_epoch.active_contacts.empty() &&
            transient_state.contact_set.empty() &&
            transient_state.vertex_contact_csr.current &&
            transient_state.vertex_contact_csr.offsets ==
                std::vector<uint32_t>{ 0u },
            "A same-side transient Correction Hit must remain a Candidate without creating an endpoint Contact.");

        auto latched_state_result =
            create_correction_segment_candidate_state(7010, {}, 16);
        auto latched_state = latched_state_result.state;
        const auto latched_epoch_primitives =
            make_vt_position_writer_epoch_primitives(transient_case.at("input"));
        require(initialize_test_correction_segment_geometry(
            latched_state, 7010, latched_epoch_primitives,
            transient_settings.contact_thickness),
            "Side-latch pipeline Epoch must initialize its Segment-start Snapshot and Physical Sweep Envelope.");
        const correction_writer_epoch_result latch_epoch_1 =
            process_correction_position_writer_epoch(
                latched_state, 7010, 1, latched_epoch_primitives,
                {}, make_correction_epoch_broadphase_settings(),
                transient_settings);
        require(latch_epoch_1.status == correction_writer_epoch_status::accepted &&
            latched_state.side_latches.size() == 1 &&
            latched_state.side_latches[0].allowed_side_sign == 1 &&
            latched_state.side_latches[0].reference_normal.z > 0.99f,
            "The first accepted Position-Writer Epoch must latch one positive Segment-side normal for its Canonical Pair.");

        auto latched_side_crossing =
            make_vt_position_writer_epoch_primitives(transient_case.at("input"));
        latched_side_crossing[0].vertex_motions[0].before =
            { 2.0f, 0.2f, 0.01f };
        latched_side_crossing[0].vertex_motions[0].after =
            { 0.25f, 0.2f, -0.01f };
        const correction_writer_epoch_result latch_epoch_2 =
            process_correction_position_writer_epoch(
                latched_state, 7010, 2, latched_side_crossing,
                {}, make_correction_epoch_broadphase_settings(),
                transient_settings);
        require(latch_epoch_2.status == correction_writer_epoch_status::rejected &&
            !latch_epoch_2.state_committed &&
            latch_epoch_2.correction_ccd_results.size() == 1 &&
            latch_epoch_2.correction_ccd_results[0].result.status ==
                correction_ccd_status::path_crossing &&
            latched_state.side_latches.size() == 1 &&
            latched_state.side_latches[0].allowed_side_sign == 1 &&
            latched_state.position_writer_epoch_generation == 1,
            "A later Epoch must reuse the original positive Segment-side latch and reject a path crossing without relatching or committing.");

        auto topology_changed_state = latched_state;
        auto changed_topology_epoch =
            make_vt_position_writer_epoch_primitives(transient_case.at("input"));
        changed_topology_epoch[1].vertex_stable_ids[0] += 100000u;
        const correction_writer_epoch_result topology_changed_result =
            process_correction_position_writer_epoch(
                topology_changed_state, 7010, 2, changed_topology_epoch,
                {}, make_correction_epoch_broadphase_settings(),
                transient_settings);
        require(topology_changed_result.status ==
                correction_writer_epoch_status::rejected &&
            !topology_changed_result.state_committed &&
            topology_changed_result.correction_ccd_results.empty() &&
            topology_changed_state.side_latches.size() == 1 &&
            topology_changed_state.position_writer_epoch_generation == 1,
            "A Segment Side Latch must be invalidated by participant topology identity changes rather than reused across a different feature.");

        const test_json::value& crossing_case = find_correction_fixture_case(
            fixture, "DCR_49_VT_POINT_THROUGH_TRIANGLE");
        correction_ccd_settings crossing_settings =
            read_correction_ccd_settings(crossing_case.at("input"));
        auto crossing_state_result =
            create_correction_segment_candidate_state(7003, {}, 16);
        auto crossing_state = crossing_state_result.state;
        const auto crossing_epoch_primitives =
            make_vt_position_writer_epoch_primitives(crossing_case.at("input"));
        require(initialize_test_correction_segment_geometry(
            crossing_state, 7003, crossing_epoch_primitives,
            crossing_settings.contact_thickness),
            "Crossing pipeline Epoch must initialize its Segment-start Snapshot and Physical Sweep Envelope.");
        const correction_writer_epoch_result crossing_epoch =
            process_correction_position_writer_epoch(
                crossing_state, 7003, 1, crossing_epoch_primitives,
                {}, make_correction_epoch_broadphase_settings(),
                crossing_settings);
        require(crossing_epoch.status == correction_writer_epoch_status::rejected &&
            !crossing_epoch.state_committed &&
            crossing_state.candidates.empty() &&
            crossing_state.position_writer_epoch_generation == 0,
            "A Correction Path Crossing must reject the entire Epoch without committing Candidate or Contact state.");

        const test_json::value& ee_case = find_correction_fixture_case(
            fixture, "DCR_49_EE_EDGE_THROUGH_EDGE");
        correction_ccd_settings ee_settings =
            read_correction_ccd_settings(ee_case.at("input"));
        auto ee_state_result =
            create_correction_segment_candidate_state(7004, {}, 16);
        auto ee_state = ee_state_result.state;
        const auto ee_epoch_primitives =
            make_ee_position_writer_epoch_primitives(ee_case.at("input"));
        require(initialize_test_correction_segment_geometry(
            ee_state, 7004, ee_epoch_primitives,
            ee_settings.contact_thickness),
            "EE crossing Epoch must initialize its Segment-start Snapshot and Physical Sweep Envelope.");
        const correction_writer_epoch_result ee_epoch =
            process_correction_position_writer_epoch(
                ee_state, 7004, 1, ee_epoch_primitives,
                {}, make_correction_epoch_broadphase_settings(),
                ee_settings);
        require(ee_epoch.status == correction_writer_epoch_status::rejected &&
            !ee_epoch.state_committed &&
            ee_state.candidates.empty() &&
            ee_state.position_writer_epoch_generation == 0,
            "An EE correction crossing must reject the entire Epoch before Contact Set or CSR publication.");

        correction_ccd_settings contact_capacity_settings = endpoint_settings;
        contact_capacity_settings.contact_set_capacity = 0;
        auto contact_capacity_state_result =
            create_correction_segment_candidate_state(7005, {}, 16);
        auto contact_capacity_state = contact_capacity_state_result.state;
        const auto contact_capacity_primitives =
            make_vt_position_writer_epoch_primitives(endpoint_case.at("input"));
        require(initialize_test_correction_segment_geometry(
            contact_capacity_state, 7005, contact_capacity_primitives,
            contact_capacity_settings.contact_thickness),
            "Contact-capacity Epoch must initialize its Segment-start Snapshot and Physical Sweep Envelope.");
        const correction_writer_epoch_result contact_capacity_epoch =
            process_correction_position_writer_epoch(
                contact_capacity_state, 7005, 1, contact_capacity_primitives,
                {}, make_correction_epoch_broadphase_settings(),
                contact_capacity_settings);
        require(contact_capacity_epoch.status == correction_writer_epoch_status::blocked &&
            contact_capacity_epoch.capacity_overflow &&
            contact_capacity_epoch.required_contacts == 1 &&
            !contact_capacity_epoch.state_committed &&
            contact_capacity_state.candidates.empty() &&
            contact_capacity_state.side_latches.empty() &&
            contact_capacity_state.position_writer_epoch_generation == 0,
            "Active Contact Set Capacity overflow must BLOCK the complete Epoch without publishing Candidate or Side-Latch state.");

        correction_ccd_settings csr_capacity_settings = endpoint_settings;
        csr_capacity_settings.vertex_contact_csr_capacity = 0;
        auto csr_capacity_state_result =
            create_correction_segment_candidate_state(7006, {}, 16);
        auto csr_capacity_state = csr_capacity_state_result.state;
        const auto csr_capacity_primitives =
            make_vt_position_writer_epoch_primitives(endpoint_case.at("input"));
        require(initialize_test_correction_segment_geometry(
            csr_capacity_state, 7006, csr_capacity_primitives,
            csr_capacity_settings.contact_thickness),
            "CSR-capacity Epoch must initialize its Segment-start Snapshot and Physical Sweep Envelope.");
        const correction_writer_epoch_result csr_capacity_epoch =
            process_correction_position_writer_epoch(
                csr_capacity_state, 7006, 1, csr_capacity_primitives,
                {}, make_correction_epoch_broadphase_settings(),
                csr_capacity_settings);
        require(csr_capacity_epoch.status == correction_writer_epoch_status::blocked &&
            csr_capacity_epoch.capacity_overflow &&
            csr_capacity_epoch.required_vertex_contact_indices == 4 &&
            !csr_capacity_epoch.state_committed &&
            csr_capacity_state.candidates.empty() &&
            csr_capacity_state.side_latches.empty() &&
            csr_capacity_state.position_writer_epoch_generation == 0,
            "Vertex-to-Contact CSR Capacity overflow must BLOCK the complete Epoch without publishing Candidate or Contact state.");

        auto blocked_state_result =
            create_correction_segment_candidate_state(7005, {}, 16);
        auto blocked_state = blocked_state_result.state;
        const auto blocked_epoch_primitives =
            make_vt_position_writer_epoch_primitives(endpoint_case.at("input"));
        require(initialize_test_correction_segment_geometry(
            blocked_state, 7005, blocked_epoch_primitives,
            endpoint_settings.contact_thickness),
            "Broadphase-capacity Epoch must initialize its Segment-start Snapshot and Physical Sweep Envelope.");
        broadphase_settings blocked_broadphase =
            make_correction_epoch_broadphase_settings();
        blocked_broadphase.candidate_capacity = 0;
        const correction_writer_epoch_result blocked_epoch =
            process_correction_position_writer_epoch(
                blocked_state, 7005, 1, blocked_epoch_primitives,
                {}, blocked_broadphase, endpoint_settings);
        require(blocked_epoch.status == correction_writer_epoch_status::blocked &&
            blocked_epoch.capacity_overflow &&
            !blocked_epoch.state_committed &&
            blocked_state.candidates.empty() &&
            blocked_state.position_writer_epoch_generation == 0,
            "Broadphase Capacity overflow must BLOCK the Epoch and preserve the prior Candidate / Contact / CSR State.");
    }

    void test_runtime_mass_weighted_wendland_c2_reference()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path(
                "phase13_force_distribution/wendland_c2.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path(
                "phase13_force_distribution/wendland_c2.expected.json"));

        const test_json::value::array& mesh_inputs =
            input.at("meshes").as_array();
        std::vector<std::vector<skin_physics::asset::vec3>> positions(
            mesh_inputs.size());
        std::vector<std::vector<float>> rest_masses(mesh_inputs.size());

        for (std::size_t mesh_index = 0u;
            mesh_index < mesh_inputs.size(); ++mesh_index)
        {
            const test_json::value::array& position_inputs =
                mesh_inputs[mesh_index].at("positions").as_array();
            const test_json::value::array& mass_inputs =
                mesh_inputs[mesh_index].at("rest_lumped_masses").as_array();
            positions[mesh_index].reserve(position_inputs.size());
            rest_masses[mesh_index].reserve(mass_inputs.size());

            for (const test_json::value& position_input : position_inputs)
            {
                positions[mesh_index].push_back(read_vec3(position_input));
            }
            for (const test_json::value& mass_input : mass_inputs)
            {
                rest_masses[mesh_index].push_back(
                    static_cast<float>(mass_input.as_number()));
            }
        }

        std::vector<skin_physics::runtime::detail::force_distribution_mesh_view>
            mesh_views(mesh_inputs.size());
        for (std::size_t mesh_index = 0u;
            mesh_index < mesh_inputs.size(); ++mesh_index)
        {
            mesh_views[mesh_index].segment_start_positions =
                &positions[mesh_index];
            mesh_views[mesh_index].rest_lumped_masses =
                &rest_masses[mesh_index];
        }

        skin_physics::runtime::detail::force_distribution_weights result;
        std::string diagnostic;
        const bool built =
            skin_physics::runtime::detail::build_mass_weighted_wendland_c2_weights(
                read_vec3(input.at("action_position")),
                static_cast<float>(input.at("radius").as_number()),
                mesh_views,
                result,
                diagnostic);
        require(built, "Mass-weighted Wendland C2 reference must build: " + diagnostic);

        require(std::abs(
                result.unnormalized_weight_sum -
                expected.at("unnormalized_weight_sum").as_number()) <= 1.0e-12,
            "Pooled Rest Lumped Mass / Wendland C2 sum must match the golden.");
        const test_json::value::array& expected_mesh_weights =
            expected.at("weights").as_array();
        require(result.mesh_weights.size() == expected_mesh_weights.size(),
            "Pooled Wendland C2 result must preserve the selected Mesh ranges.");

        double normalized_weight_sum = 0.0;
        for (std::size_t mesh_index = 0u;
            mesh_index < result.mesh_weights.size(); ++mesh_index)
        {
            const test_json::value::array& expected_vertex_weights =
                expected_mesh_weights[mesh_index].as_array();
            require(result.mesh_weights[mesh_index].size() ==
                    expected_vertex_weights.size(),
                "Wendland C2 result must preserve the Physics Vertex count.");

            for (std::size_t vertex_index = 0u;
                vertex_index < result.mesh_weights[mesh_index].size();
                ++vertex_index)
            {
                const double actual = result.mesh_weights[mesh_index][vertex_index];
                const double golden =
                    expected_vertex_weights[vertex_index].as_number();
                require(std::abs(actual - golden) <= 1.0e-12,
                    "Mass-weighted Wendland C2 vertex weight must match the golden.");
                normalized_weight_sum += actual;
            }
        }

        require(std::abs(
                normalized_weight_sum -
                expected.at("normalized_weight_sum").as_number()) <= 1.0e-12,
            "The independently normalized Event weights must sum to one across all selected Meshes.");

        const test_json::value::array& fixed_vertices =
            input.at("fixed_vertices").as_array();
        const uint32_t fixed_mesh_index = static_cast<uint32_t>(
            fixed_vertices[0].at("mesh_index").as_number());
        const uint32_t fixed_vertex_index = static_cast<uint32_t>(
            fixed_vertices[0].at("vertex_index").as_number());
        require(result.mesh_weights[fixed_mesh_index][fixed_vertex_index] > 0.0,
            "A Fixed Physics Vertex inside the support must still receive its Rest-Mass share.");

        require(result.mesh_weights[0u][2u] == 0.0,
            "A Physics Vertex exactly on the compact-support radius must receive zero weight.");

        const skin_physics::asset::vec3 direction =
            read_vec3(input.at("force_direction"));
        const double force_magnitude =
            input.at("force_magnitude").as_number();
        const double impulse_magnitude =
            input.at("impulse_magnitude").as_number();
        double distributed_force_y = 0.0;
        double distributed_impulse_y = 0.0;
        for (const std::vector<double>& mesh_weights : result.mesh_weights)
        {
            for (double weight : mesh_weights)
            {
                distributed_force_y +=
                    weight * force_magnitude * static_cast<double>(direction.y);
                distributed_impulse_y +=
                    weight * impulse_magnitude * static_cast<double>(direction.y);
            }
        }
        require(std::abs(distributed_force_y - force_magnitude) <= 1.0e-10,
            "Mass-weighted Force distribution must conserve the descriptor magnitude.");
        require(std::abs(distributed_impulse_y - impulse_magnitude) <= 1.0e-10,
            "Mass-weighted Impulse distribution must conserve the descriptor magnitude.");

        std::vector<std::vector<skin_physics::asset::vec3>> moved_positions = positions;
        moved_positions[0u][0u].x = 0.25f;
        std::vector<skin_physics::runtime::detail::force_distribution_mesh_view>
            moved_mesh_views(mesh_views.size());
        for (std::size_t mesh_index = 0u;
            mesh_index < moved_mesh_views.size(); ++mesh_index)
        {
            moved_mesh_views[mesh_index].segment_start_positions =
                &moved_positions[mesh_index];
            moved_mesh_views[mesh_index].rest_lumped_masses =
                &rest_masses[mesh_index];
        }

        skin_physics::runtime::detail::force_distribution_weights moved_result;
        require(
            skin_physics::runtime::detail::build_mass_weighted_wendland_c2_weights(
                read_vec3(input.at("action_position")),
                static_cast<float>(input.at("radius").as_number()),
                moved_mesh_views,
                moved_result,
                diagnostic),
            "Wendland C2 distribution must be recomputable from the next Segment-start positions.");
        require(std::abs(
                moved_result.mesh_weights[0u][0u] - result.mesh_weights[0u][0u]) > 1.0e-6,
            "Moving a Physics Vertex between Segments must refresh its spatial Falloff weight.");

        std::vector<skin_physics::asset::vec3> outside_positions = {
            { 3.0f, 0.0f, 0.0f }
        };
        std::vector<float> outside_masses = { 1.0f };
        std::vector<skin_physics::runtime::detail::force_distribution_mesh_view>
            outside_mesh_views(1u);
        outside_mesh_views[0u].segment_start_positions = &outside_positions;
        outside_mesh_views[0u].rest_lumped_masses = &outside_masses;

        skin_physics::runtime::detail::force_distribution_weights preserved_output;
        preserved_output.mesh_weights = { { 0.25 } };
        preserved_output.unnormalized_weight_sum = 7.0;
        require(
            !skin_physics::runtime::detail::build_mass_weighted_wendland_c2_weights(
                read_vec3(input.at("action_position")),
                static_cast<float>(input.at("radius").as_number()),
                outside_mesh_views,
                preserved_output,
                diagnostic),
            "No-support input must be rejected and reported.");
        require(diagnostic.find("NO_FORCE_TARGET") != std::string::npos,
            "No-support input must use the documented NO_FORCE_TARGET diagnostic.");
        require(preserved_output.mesh_weights.size() == 1u &&
            preserved_output.mesh_weights[0u].size() == 1u &&
            preserved_output.mesh_weights[0u][0u] == 0.25 &&
            preserved_output.unnormalized_weight_sum == 7.0,
            "A failed distribution must leave the caller's previous output unchanged.");

        std::vector<skin_physics::asset::vec3> tiny_mass_positions = {
            { 0.0f, 0.0f, 0.0f }
        };
        std::vector<float> tiny_masses = { 1.0e-30f };
        std::vector<skin_physics::runtime::detail::force_distribution_mesh_view>
            tiny_mass_views(1u);
        tiny_mass_views[0u].segment_start_positions = &tiny_mass_positions;
        tiny_mass_views[0u].rest_lumped_masses = &tiny_masses;
        require(
            !skin_physics::runtime::detail::build_mass_weighted_wendland_c2_weights(
                read_vec3(input.at("action_position")),
                static_cast<float>(input.at("radius").as_number()),
                tiny_mass_views,
                preserved_output,
                diagnostic),
            "A positive but sub-threshold SI mass support must be rejected consistently with the GPU minimum weight sum.");
        require(diagnostic.find("NO_FORCE_TARGET") != std::string::npos &&
            preserved_output.mesh_weights[0u][0u] == 0.25,
            "A sub-threshold normalization sum must preserve output and report NO_FORCE_TARGET.");

        require(
            !skin_physics::runtime::detail::build_mass_weighted_wendland_c2_weights(
                read_vec3(input.at("action_position")),
                0.0f,
                mesh_views,
                preserved_output,
                diagnostic),
            "A zero-radius Force / Impulse must be rejected.");
        require(preserved_output.mesh_weights[0u][0u] == 0.25,
            "Invalid radius must not partially overwrite a prior distribution.");

        std::vector<float> mismatched_masses = { 1.0f };
        std::vector<skin_physics::runtime::detail::force_distribution_mesh_view>
            mismatched_views(1u);
        mismatched_views[0u].segment_start_positions = &outside_positions;
        mismatched_views[0u].rest_lumped_masses = &mismatched_masses;
        require(
            !skin_physics::runtime::detail::build_mass_weighted_wendland_c2_weights(
                read_vec3(input.at("action_position")),
                static_cast<float>(input.at("radius").as_number()),
                mismatched_views,
                preserved_output,
                diagnostic),
            "Position / Rest Mass count mismatch must be rejected.");
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
    test_phase03_warp_compute_render_feedback();
    std::cout << "[PASS] P03_WARP_RENDER_FEEDBACK\n";
    test_phase04_overload_and_callback_failure();
    std::cout << "[PASS] P04_FIXED_STEP_OVERLOAD\n";
    test_phase04_pause_stop_and_reconfigure_preserve_accumulator();
    std::cout << "[PASS] P04_STOP_PRESERVES_ACCUMULATOR\n";
    test_phase05_cloth_fixture_and_bend_gradient();
    std::cout << "[PASS] P05_CLOTH_GOLDEN_AND_BEND_GRADIENT\n";
    test_phase05_cloth_compliance_monotonicity();
    std::cout << "[PASS] P05_CLOTH_COMPLIANCE\n";
    test_phase06_corotational_fem_golden();
    std::cout << "[PASS] P06_COROTATIONAL_FEM_GOLDEN\n";
    test_phase06_inversion_penalty_is_finite_and_not_committed();
    std::cout << "[PASS] P06_INVERSION_ROLLBACK\n";
    test_phase07_vertex_triangle_and_edge_edge_contacts();
    std::cout << "[PASS] P07_VT_EE_CONTACTS\n";
    test_phase07_broadphase_filters_and_capacity_overflow();
    std::cout << "[PASS] P07_BROADPHASE_OVERFLOW\n";
    test_phase08_ccd_and_initial_penetration_repair();
    std::cout << "[PASS] P08_CCD_INITIAL_REPAIR\n";
    test_phase09_animation_target_sampling();
    std::cout << "[PASS] P09_ANIMATION_TARGET\n";
    test_phase09_animation_channel_span_coefficients();
    std::cout << "[PASS] P09_ANIMATION_CHANNEL_SPAN_COEFFICIENTS\n";
    test_phase09_animation_node_hierarchy_and_skin_matrix_derivatives();
    std::cout << "[PASS] P09_ANIMATION_NODE_HIERARCHY_SKIN_MATRIX_DERIVATIVES\n";
    test_phase10_attribute_paint_and_serialization();
    std::cout << "[PASS] P10_ATTRIBUTE_PAINT\n";
    test_phase11_performance_profile_and_invariant_collector();
    std::cout << "[PASS] P11_PERFORMANCE_PROFILE\n";
    test_phase12_correction_motion_ccd_reference();
    std::cout << "[PASS] P12_CORRECTION_MOTION_CCD_REFERENCE\n";
    test_phase12_correction_position_writer_epoch_pipeline();
    std::cout << "[PASS] P12_CORRECTION_POSITION_WRITER_EPOCH\n";
    test_runtime_mass_weighted_wendland_c2_reference();
    std::cout << "[PASS] R1_FORCE_WENDLAND_C2_CPU_REFERENCE\n";
    require(run_environment_tests() == 0, "Environment tests failed.");
    require(run_gpu_solver_tests() == 0, "GPU solver tests failed.");
    std::cout << "SimulationCoreTests passed.\n";
    return 0;
}
