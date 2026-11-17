#include "skin_physics/physics/msh_reader.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"
#include "skin_physics/physics/render_binding.hpp"
#include "skin_physics/physics/cloth_solver.hpp"
#include "skin_physics/physics/soft_body_fem.hpp"
#include "skin_physics/physics/collision.hpp"
#include "skin_physics/physics/ccd.hpp"
#include "skin_physics/simulation/fixed_step.hpp"
#include "json_test_support.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
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
    std::cout << "SimulationCoreTests passed.\n";
    return 0;
}
