#include "skin_physics/asset/importer.hpp"
#include "skin_physics/physics/msh_reader.hpp"
#include "skin_physics/physics/ftetwild_runner.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"
#include "skin_physics/physics/render_binding.hpp"
#include "skin_physics/physics/skin_weights.hpp"
#include "skin_physics/physics/asset_cache.hpp"
#include "test_support.hpp"
#include "json_test_support.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace skin_physics::physics;

    void require(bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    std::filesystem::path test_directory()
    {
        const std::filesystem::path directory =
            std::filesystem::temp_directory_path() / "skin_physics_physics_mesh_tests";
        std::filesystem::create_directories(directory);
        return directory;
    }

    void write_msh_fixture(const std::filesystem::path& path)
    {
        std::ofstream file(path, std::ios::binary);
        require(static_cast<bool>(file), "Could not create MSH fixture.");
        file <<
            "$MeshFormat\n"
            "2.2 0 8\n"
            "$EndMeshFormat\n"
            "$Nodes\n"
            "4\n"
            "10 0 0 0\n"
            "20 1 0 0\n"
            "30 0 1 0\n"
            "40 0 0 1\n"
            "$EndNodes\n"
            "$Elements\n"
            "5\n"
            "1 2 2 1 1 10 30 20\n"
            "2 2 2 1 1 10 20 40\n"
            "3 2 2 1 1 20 30 40\n"
            "4 2 2 1 1 30 10 40\n"
            "5 4 2 1 1 10 20 30 40\n"
            "$EndElements\n";
        require(static_cast<bool>(file), "Could not write MSH fixture.");
    }

    raw_tet_mesh make_single_tetrahedron()
    {
        raw_tet_mesh input;
        input.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f }
        };
        input.tetrahedra = { { 0, 1, 2, 3 } };
        return input;
    }

    void test_msh_reader_maps_external_node_ids()
    {
        const std::filesystem::path path = test_directory() / "single_tet.msh";
        write_msh_fixture(path);

        const msh_read_result result = read_gmsh_2_2_ascii(path);
        require(result.success, "ASCII Gmsh 2.2 mesh should be imported.");
        require(result.mesh.positions.size() == 4, "MSH reader should retain all four nodes.");
        require(result.mesh.tetrahedra.size() == 1, "MSH reader should only import tetrahedron elements.");
        require(result.mesh.tetrahedra[0] == std::array<uint32_t, 4>{ 0, 1, 2, 3 },
            "Non-contiguous Gmsh node IDs should map to local zero-based indices.");
    }

    void test_physics_mesh_builder_creates_rest_data_and_adjacency()
    {
        const raw_tet_mesh input = make_single_tetrahedron();

        physics_mesh_build_settings settings;
        settings.density = 6.0f;

        const physics_mesh_build_result result = build_physics_mesh(input, settings);
        require(result.success, "A valid tetrahedron should build a physics mesh.");
        require(result.value.tetrahedra.size() == 1, "Physics mesh should contain one tetrahedron.");
        require(result.value.boundary_triangles.size() == 4, "Single tetrahedron should have four boundary faces.");
        require(result.value.vertex_tetrahedron_offsets.size() == 5, "Vertex-to-tet CSR should contain N+1 offsets.");
        require(result.value.vertex_tetrahedron_ids.size() == 4, "Each tetrahedron should be referenced by its four vertices.");
        require(result.value.tetrahedra[0].rest_volume > 0.0f, "Rest volume must be positive.");
        require(std::abs(result.value.tetrahedra[0].rest_volume - (1.0f / 6.0f)) < 1.0e-6f,
            "Rest volume should match the reference tetrahedron.");
        require(result.value.tetrahedra[0].quality > 0.0f, "Tetrahedron quality should be positive.");
        require(result.value.tetrahedra[0].minimum_dihedral_degrees > 0.0f,
            "Minimum dihedral angle should be calculated.");

        float total_mass = 0.0f;
        for (std::size_t index = 0; index < result.value.mass.size(); ++index)
        {
            total_mass += result.value.mass[index];
            require(std::abs(result.value.mass[index] - 0.25f) < 1.0e-6f,
                "Lumped mass should be distributed equally to four vertices.");
            require(std::abs(result.value.inverse_mass[index] - 4.0f) < 1.0e-5f,
                "Inverse mass should be calculated from lumped mass.");
        }
        require(std::abs(total_mass - 1.0f) < 1.0e-6f,
            "Total lumped mass should equal density multiplied by rest volume.");

        const auto& gradients = result.value.tetrahedra[0].rest_shape_gradients;
        const skin_physics::asset::vec3 gradient_sum = {
            gradients[0].x + gradients[1].x + gradients[2].x + gradients[3].x,
            gradients[0].y + gradients[1].y + gradients[2].y + gradients[3].y,
            gradients[0].z + gradients[1].z + gradients[2].z + gradients[3].z
        };
        require(std::abs(gradient_sum.x) < 1.0e-6f &&
                std::abs(gradient_sum.y) < 1.0e-6f &&
                std::abs(gradient_sum.z) < 1.0e-6f,
            "Shape function gradients should sum to zero.");
    }


    void test_persistent_physics_mesh_golden_fixtures()
    {
        const std::filesystem::path single_path =
            test_support::fixture_path("phase02_physics_mesh/single_tet.msh");
        const msh_read_result single_read = read_gmsh_2_2_ascii(single_path);
        require(single_read.success, "Persistent single-tetrahedron MSH fixture should load.");
        require(single_read.mesh.positions.size() == 4 && single_read.mesh.tetrahedra.size() == 1,
            "Single-tetrahedron fixture topology should match its golden.");

        physics_mesh_build_settings settings;
        settings.density = 6.0f;
        const physics_mesh_build_result single = build_physics_mesh(single_read.mesh, settings);
        require(single.success, "Single-tetrahedron golden should build.");
        require(single.value.tetrahedra.size() == 1 && single.value.boundary_triangles.size() == 4,
            "Single-tetrahedron golden should have one element and four boundary faces.");
        require(std::abs(single.value.tetrahedra[0].rest_volume - (1.0f / 6.0f)) < 1.0e-6f,
            "Single-tetrahedron golden rest volume should be 1/6.");
        require(std::abs(single.value.mass[0] - 0.25f) < 1.0e-6f &&
                std::abs(single.value.inverse_mass[0] - 4.0f) < 1.0e-5f,
            "Single-tetrahedron golden mass and inverse mass should match their references.");
        require(std::filesystem::exists(test_support::fixture_path(
            "phase02_physics_mesh/single_tet.expected.json")),
            "Single-tetrahedron golden expected-result file must exist.");

        const msh_read_result two_read = read_gmsh_2_2_ascii(
            test_support::fixture_path("phase02_physics_mesh/two_tet_shared_face.msh"));
        require(two_read.success, "Persistent two-tetrahedron MSH fixture should load.");
        require(two_read.mesh.positions.size() == 5 && two_read.mesh.tetrahedra.size() == 2,
            "Two-tetrahedron fixture topology should match its golden.");

        const physics_mesh_build_result two = build_physics_mesh(two_read.mesh, settings);
        require(two.success, "Two tetrahedra sharing one face should build.");
        require(two.value.tetrahedra.size() == 2 && two.value.boundary_triangles.size() == 6,
            "Shared internal face must be removed from the boundary.");
        float two_tet_total_mass = 0.0f;
        for (float mass : two.value.mass) two_tet_total_mass += mass;
        require(std::abs(two_tet_total_mass - 2.0f) < 1.0e-5f,
            "Two-tetrahedron golden mass must equal density multiplied by total rest volume.");
        require(std::filesystem::exists(test_support::fixture_path(
            "phase02_physics_mesh/two_tet.expected.json")),
            "Two-tetrahedron golden expected-result file must exist.");

        const msh_read_result center_read = read_gmsh_2_2_ascii(
            test_support::fixture_path("phase02_physics_mesh/center_star.msh"));
        require(center_read.success, "Persistent center-star MSH fixture should load.");
        require(center_read.mesh.positions.size() == 5 && center_read.mesh.tetrahedra.size() == 4,
            "Center-star fixture topology should match its golden.");
        const physics_mesh_build_result center = build_physics_mesh(center_read.mesh);
        require(center.success, "Center-star fixture should build successfully.");
        require(center.value.boundary_triangles.size() == 4,
            "Center-star fixture should retain only the four outer boundary triangles.");
        require(std::filesystem::exists(test_support::fixture_path(
            "phase02_physics_mesh/center_star_skin_weights.expected.json")),
            "Center-star skin weight expected-result file must exist.");
    }

    void test_negative_tetrahedron_orientation_is_repaired()
    {
        raw_tet_mesh input = make_single_tetrahedron();
        input.tetrahedra[0] = { 0, 2, 1, 3 };

        const physics_mesh_build_result result = build_physics_mesh(input);
        require(result.success, "A negatively oriented but non-degenerate tetrahedron should be reoriented.");
        require(result.value.tetrahedra[0].rest_volume > 0.0f, "Reoriented tetrahedron volume must be positive.");
        require(result.value.tetrahedra[0].vertices[1] == 1 &&
                result.value.tetrahedra[0].vertices[2] == 2,
            "Reorientation should swap a single vertex pair.");
    }

    void test_invalid_tetrahedron_index_is_rejected()
    {
        raw_tet_mesh input = make_single_tetrahedron();
        input.tetrahedra[0][3] = 99;

        const physics_mesh_build_result result = build_physics_mesh(input);
        require(!result.success, "Out-of-range tetrahedron indices must fail.");
        require(!result.diagnostics.empty(), "Invalid tetrahedron indices must be diagnosed.");
        require(result.diagnostics[0].code == physics_mesh_error::invalid_index,
            "Invalid tetrahedron index should produce invalid_index.");
    }

    void test_quality_threshold_is_enforced()
    {
        const raw_tet_mesh input = make_single_tetrahedron();
        physics_mesh_build_settings settings;
        settings.minimum_quality = 1.0f;

        const physics_mesh_build_result result = build_physics_mesh(input, settings);
        require(!result.success, "A quality threshold above the measured value must reject the mesh.");
        require(!result.diagnostics.empty(), "Rejected tetrahedron quality must be diagnosed.");
    }

    void test_non_manifold_tetrahedron_faces_are_rejected()
    {
        raw_tet_mesh input;
        input.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f },
            { 0.0f, 0.0f, -1.0f },
            { 1.0f, 1.0f, 1.0f }
        };
        input.tetrahedra = {
            { 0, 1, 2, 3 },
            { 0, 2, 1, 4 },
            { 0, 1, 2, 5 }
        };

        const physics_mesh_build_result result = build_physics_mesh(input);
        require(!result.success, "A face shared by three tetrahedra must be rejected.");
    }

    raw_tet_mesh make_center_tetrahedral_mesh()
    {
        raw_tet_mesh input;
        input.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f },
            { 0.25f, 0.25f, 0.25f }
        };
        input.tetrahedra = {
            { 0, 1, 2, 4 },
            { 0, 3, 1, 4 },
            { 1, 3, 2, 4 },
            { 2, 3, 0, 4 }
        };
        return input;
    }

    void test_skin_weight_harmonic_extension()
    {
        skin_physics::asset::mesh render_mesh;
        render_mesh.vertices.resize(4);
        render_mesh.indices = {
            0, 2, 1,
            0, 1, 3,
            1, 2, 3,
            2, 0, 3
        };

        for (uint16_t vertex_index = 0; vertex_index < 4; ++vertex_index)
        {
            skin_physics::asset::vertex& current_vertex = render_mesh.vertices[vertex_index];
            current_vertex.position = {
                vertex_index == 1 ? 1.0f : 0.0f,
                vertex_index == 2 ? 1.0f : 0.0f,
                vertex_index == 3 ? 1.0f : 0.0f
            };
            current_vertex.attributes |= skin_physics::asset::attribute_skinning;
            current_vertex.joints[0] = vertex_index;
            current_vertex.weights[0] = 1.0f;
        }

        skin_physics::asset::skin render_skin;
        render_skin.joints = { 0, 1, 2, 3 };
        render_skin.inverse_bind_matrices.resize(4);

        const msh_read_result center_star = read_gmsh_2_2_ascii(
            test_support::fixture_path("phase02_physics_mesh/center_star.msh"));
        require(center_star.success, "Persistent center-star fixture should load for skin transfer.");
        const physics_mesh_build_result built = build_physics_mesh(center_star.mesh);
        require(built.success, "Center tetrahedral golden fixture should build successfully.");

        physics_mesh physics_value = built.value;
        skin_weight_transfer_settings settings;
        settings.maximum_surface_distance = 1.0e-4f;
        settings.maximum_iterations = 32;
        settings.relative_residual_tolerance = 1.0e-6f;

        const skin_weight_transfer_result transferred =
            transfer_skin_weights(render_mesh, render_skin, physics_value, settings);
        require(transferred.success, "Skin weight harmonic extension should succeed.");
        require(physics_value.has_skin_weights, "Physics Mesh should flag transferred skin weights.");
        require(physics_value.skin_weights.size() == physics_value.rest_positions.size(),
            "Skin weight count should match Physics Mesh vertex count.");
        require(transferred.maximum_relative_residual <= settings.relative_residual_tolerance,
            "Harmonic extension residual should satisfy the configured tolerance.");

        const physics_skin_weight& center = physics_value.skin_weights[4];
        float center_sum = 0.0f;
        for (std::size_t influence = 0; influence < skin_physics::asset::max_skin_influences; ++influence)
        {
            center_sum += center.weights[influence];
        }
        require(std::abs(center_sum - 1.0f) < 1.0e-5f,
            "Interior skin weights should sum to one.");
        require(center.weights[0] > 0.0f && center.weights[1] > 0.0f &&
                center.weights[2] > 0.0f && center.weights[3] > 0.0f,
            "Interior vertex should receive all four boundary joint influences.");

        require(center.joints[0] == 0,
            "The nearest boundary influence should remain the largest interior influence.");
        require(std::abs(center.weights[0] - 0.55f) < 1.0e-4f,
            "Interior harmonic weight for the origin joint should match the analytical solution.");
        for (std::size_t influence = 1; influence < 4; ++influence)
        {
            require(std::abs(center.weights[influence] - 0.15f) < 1.0e-4f,
                "Interior harmonic weights should match the analytical edge-weighted solution.");
        }
    }

    void test_physics_attribute_weight_transfer_and_harmonic_extension()
    {
        const raw_tet_mesh raw_mesh = make_center_tetrahedral_mesh();
        const physics_mesh_build_result built = build_physics_mesh(raw_mesh);
        require(built.success,
            "Center-star Physics Mesh should build for attribute transfer.");

        skin_physics::asset::mesh render_mesh;
        render_mesh.name = "attribute_transfer_center_star";
        render_mesh.vertices.resize(4u);
        for (uint32_t vertex_index = 0u; vertex_index < 4u; ++vertex_index)
        {
            render_mesh.vertices[vertex_index].position =
                raw_mesh.positions[vertex_index];
        }
        for (const boundary_triangle& face : built.value.boundary_triangles)
        {
            render_mesh.indices.insert(
                render_mesh.indices.end(),
                face.vertices.begin(),
                face.vertices.end());
        }

        const std::size_t shape_channel = static_cast<std::size_t>(
            skin_physics::asset::physics_attribute_channel::shape_recovery);
        const std::size_t volume_channel = static_cast<std::size_t>(
            skin_physics::asset::physics_attribute_channel::volume);
        const std::size_t damping_channel = static_cast<std::size_t>(
            skin_physics::asset::physics_attribute_channel::damping);
        const std::array<float, 4> shape_values = { 0.0f, 1.0f, 0.0f, 1.0f };
        const std::array<float, 4> volume_values = { 1.0f, 1.0f, 0.0f, 0.0f };
        const std::array<float, 4> damping_values = { 0.0f, 0.25f, 0.75f, 1.0f };
        for (std::size_t vertex_index = 0u; vertex_index < 4u; ++vertex_index)
        {
            render_mesh.vertices[vertex_index].physics_weights[shape_channel] =
                shape_values[vertex_index];
            render_mesh.vertices[vertex_index].physics_weights[volume_channel] =
                volume_values[vertex_index];
            render_mesh.vertices[vertex_index].physics_weights[damping_channel] =
                damping_values[vertex_index];
        }

        physics_attribute_transfer_settings settings;
        settings.maximum_surface_distance = 1.0e-4f;
        settings.harmonic_relative_tolerance = 1.0e-8;
        const physics_attribute_transfer_result transferred =
            transfer_physics_vertex_attribute_weights(
                render_mesh, built.value, settings);
        require(transferred.success,
            "Render-to-Physics attribute transfer and harmonic extension should succeed.");
        require(transferred.vertex_weights.size() ==
                built.value.rest_positions.size(),
            "Transferred attribute count must match the Physics Vertex count.");
        require(transferred.maximum_surface_distance <=
                settings.maximum_surface_distance,
            "Boundary attribute projection must respect its distance limit.");
        require(transferred.maximum_harmonic_relative_residual <=
                settings.harmonic_relative_tolerance,
            "Interior harmonic extension residual must satisfy its tolerance.");

        for (uint32_t vertex_index = 0u; vertex_index < 4u; ++vertex_index)
        {
            const physics_vertex_attribute_weights& weights =
                transferred.vertex_weights[vertex_index];
            require(std::abs(weights[shape_channel] - shape_values[vertex_index]) < 1.0e-6f &&
                std::abs(weights[volume_channel] - volume_values[vertex_index]) < 1.0e-6f &&
                std::abs(weights[damping_channel] - damping_values[vertex_index]) < 1.0e-6f,
                "Boundary Physics Vertices must preserve their Render Mesh paint values.");
        }

        const physics_vertex_attribute_weights& center =
            transferred.vertex_weights[4u];
        require(std::abs(center[shape_channel] - 0.5f) < 1.0e-5f &&
            std::abs(center[volume_channel] - 0.5f) < 1.0e-5f &&
            std::abs(center[damping_channel] - 0.5f) < 1.0e-5f,
            "The center-star interior vertex must receive the analytical harmonic mean.");

        skin_physics::asset::mesh ambiguous_mesh = render_mesh;
        const std::array<uint32_t, 3> source_triangle = {
            ambiguous_mesh.indices[0u],
            ambiguous_mesh.indices[1u],
            ambiguous_mesh.indices[2u]
        };
        std::array<uint32_t, 3> duplicate_triangle = {};
        for (std::size_t local_index = 0u; local_index < 3u; ++local_index)
        {
            skin_physics::asset::vertex duplicate =
                ambiguous_mesh.vertices[source_triangle[local_index]];
            duplicate.physics_weights[shape_channel] = 1.0f;
            duplicate_triangle[local_index] =
                static_cast<uint32_t>(ambiguous_mesh.vertices.size());
            ambiguous_mesh.vertices.push_back(duplicate);
        }
        ambiguous_mesh.indices.insert(
            ambiguous_mesh.indices.end(),
            duplicate_triangle.begin(),
            duplicate_triangle.end());

        const physics_attribute_transfer_result ambiguous =
            transfer_physics_vertex_attribute_weights(
                ambiguous_mesh, built.value, settings);
        require(!ambiguous.success && !ambiguous.diagnostic.empty(),
            "Coincident Render surfaces with conflicting paint must be rejected as ambiguous.");
    }

    physics_asset_cache_identity make_test_cache_identity()
    {
        physics_asset_cache_identity identity;
        identity.schema_version = 1;
        identity.source_asset_id = "unit_test_asset";
        identity.source_asset_hash = "sha256:unit_test_hash";
        identity.generator_id = "test_tetrahedralizer";
        identity.generator_version = "1.0.0";
        identity.generator_options = "target_edge_length=1";
        identity.coordinate_system_version = 1;
        identity.material_layout_version = 1;
        return identity;
    }

    void test_ftetwild_resolution_settings_report_and_cache_identity()
    {
        const test_json::value input = test_json::load(
            test_support::fixture_path("phase02_physics_mesh/resolution.input.json"));
        const test_json::value expected = test_json::load(
            test_support::fixture_path("phase02_physics_mesh/resolution.expected.json"));

        skin_physics::asset::import_options import_options;
        import_options.has_obj_unit_scale = true;
        import_options.obj_unit_scale = 0.01f;
        const skin_physics::asset::import_result imported =
            skin_physics::asset::import_model(
                test_support::fixture_path("phase01_importer/quad.obj"), import_options);
        require(imported.success && imported.asset.meshes.size() == 1,
            "Phase 2 resolution source surface should import from the persistent OBJ fixture.");
        const skin_physics::asset::mesh& surface_mesh = imported.asset.meshes[0];

        ftetwild_settings edge_settings;
        edge_settings.target_edge_length = static_cast<float>(
            input.at("settings").at("target_tet_edge_length").as_number());
        edge_settings.maximum_tetrahedra = static_cast<uint32_t>(
            input.at("settings").at("max_tet_count").as_number());
        edge_settings.surface_approximation_tolerance = 1.0e-3f;
        edge_settings.tetra_quality_threshold = 1.0e-5f;
        edge_settings.generator_id = "fTetWild";
        edge_settings.generator_version = "fixture-version";

        const ftetwild_resolution_result edge_resolution =
            resolve_ftetwild_settings(surface_mesh, edge_settings);
        require(edge_resolution.success &&
            edge_resolution.value.resolution_mode == ftetwild_resolution_mode::target_edge_length &&
            !edge_resolution.value.edge_length_is_estimated &&
            std::abs(edge_resolution.value.target_edge_length -
                input.at("settings").at("target_tet_edge_length").as_number()) <= 1.0e-6,
            "Explicit target edge length should pass through unchanged.");

        ftetwild_settings count_settings = edge_settings;
        count_settings.target_edge_length = 0.0f;
        count_settings.target_tetrahedron_count = static_cast<uint32_t>(
            input.at("alternate_settings").at("target_tet_count").as_number());
        const ftetwild_resolution_result count_resolution =
            resolve_ftetwild_settings(surface_mesh, count_settings);
        require(count_resolution.success &&
            count_resolution.value.resolution_mode == ftetwild_resolution_mode::target_tetrahedron_count &&
            count_resolution.value.edge_length_is_estimated &&
            count_resolution.value.requested_tetrahedron_count == 64,
            "Target tetrahedron count should resolve into a deterministic target edge length.");

        const double expected_diagonal = std::sqrt(2.0);
        const double expected_edge_length = expected_diagonal / 4.0;
        require(std::abs(count_resolution.value.surface_bbox_diagonal - expected_diagonal) <= 1.0e-6 &&
            std::abs(count_resolution.value.target_edge_length - expected_edge_length) <= 1.0e-6,
            "Count mode edge length should match bbox diagonal divided by cube-root count.");

        physics_asset_cache_identity identity;
        identity.source_asset_id = "resolution_fixture";
        identity.source_asset_hash = "sha256:quad_fixture";
        require(apply_ftetwild_cache_identity(count_settings, count_resolution.value, identity),
            "fTetWild generator settings should populate the physics cache identity.");
        require(identity.generator_id == "fTetWild" &&
            identity.generator_version == "fixture-version" &&
            identity.generator_options.find("target_tetrahedron_count") != std::string::npos,
            "Generator id, version and resolved settings must participate in the cache identity.");

        ftetwild_settings changed_count_settings = count_settings;
        changed_count_settings.surface_approximation_tolerance *= 2.0f;
        physics_asset_cache_identity changed_identity = identity;
        require(apply_ftetwild_cache_identity(
            changed_count_settings, count_resolution.value, changed_identity),
            "Changed fTetWild settings should still produce a cache identity.");
        require(changed_identity.generator_options != identity.generator_options,
            "Changing a generation option must invalidate cache identity.");

        raw_tet_mesh generated_mesh = make_single_tetrahedron();
        tetrahedralization_report report;
        std::string report_error;
        require(measure_tetrahedralization_output(
            generated_mesh, count_resolution.value, report, report_error),
            "Reviewed tetrahedralization output should produce a generation report.");
        require(report.actual_vertex_count == 4 &&
            report.actual_tetrahedron_count == 1 &&
            report.has_edge_statistics,
            "Generation report should record actual vertex/tetrahedron counts and edge statistics.");

        const double expected_minimum_edge = 1.0;
        const double expected_average_edge = (1.0 + std::sqrt(2.0)) * 0.5;
        const double expected_maximum_edge = std::sqrt(2.0);
        require(std::abs(report.minimum_edge_length - expected_minimum_edge) <= 1.0e-6 &&
            std::abs(report.average_edge_length - expected_average_edge) <= 1.0e-6 &&
            std::abs(report.maximum_edge_length - expected_maximum_edge) <= 1.0e-6,
            "Generation report edge-length distribution should match a right unit tetrahedron.");
        const double expected_tetra_quality = 0.8399473666;
        require(std::abs(report.minimum_tetra_quality - expected_tetra_quality) <= 1.0e-5 &&
            std::abs(report.average_tetra_quality - expected_tetra_quality) <= 1.0e-5 &&
            std::abs(report.maximum_tetra_quality - expected_tetra_quality) <= 1.0e-5,
            "Generation report tetrahedron quality should match the analytical right-tet reference.");

        resolved_ftetwild_settings strict_quality_resolution = count_resolution.value;
        strict_quality_resolution.minimum_tetra_quality = 0.9f;
        tetrahedralization_report rejected_quality_report;
        require(!measure_tetrahedralization_output(
            generated_mesh, strict_quality_resolution, rejected_quality_report, report_error),
            "Generated tetrahedra below the configured quality threshold must be rejected.");

        require(std::holds_alternative<std::nullptr_t>(
            expected.at("absolute_tetrahedron_count").data),
            "Resolution fixture must not hardcode fTetWild-version-specific tetrahedron counts.");

        ftetwild_settings ambiguous_settings = edge_settings;
        ambiguous_settings.target_tetrahedron_count = 64;
        const ftetwild_resolution_result ambiguous =
            resolve_ftetwild_settings(surface_mesh, ambiguous_settings);
        require(!ambiguous.success &&
            ambiguous.diagnostics[0].code == tetrahedralizer_error::invalid_settings,
            "Resolution API must reject simultaneous edge-length and target-count selectors.");

        ftetwild_settings over_capacity_settings = count_settings;
        over_capacity_settings.target_tetrahedron_count = static_cast<uint32_t>(
            input.at("over_capacity_settings").at("target_tet_count").as_number());
        over_capacity_settings.maximum_tetrahedra = static_cast<uint32_t>(
            input.at("over_capacity_settings").at("max_tet_count").as_number());
        const ftetwild_resolution_result over_capacity =
            resolve_ftetwild_settings(surface_mesh, over_capacity_settings);
        require(!over_capacity.success &&
            over_capacity.diagnostics[0].code == tetrahedralizer_error::invalid_settings,
            "Requested target count greater than maximum_tetrahedra must be rejected before launch.");

        raw_tet_mesh invalid_output = generated_mesh;
        invalid_output.tetrahedra[0][3] = 999;
        tetrahedralization_report invalid_report;
        require(!measure_tetrahedralization_output(
            invalid_output, count_resolution.value, invalid_report, report_error),
            "Generation report must reject output with invalid element indices.");
    }

    void test_physics_asset_cache_round_trip_and_invalidation()
    {
        const std::filesystem::path cache_path = test_directory() / "single_tet.sphys";
        const physics_mesh_build_result built = build_physics_mesh(make_single_tetrahedron());
        require(built.success, "Physics Mesh for cache test should build.");

        const physics_asset_cache_identity identity = make_test_cache_identity();
        const physics_asset_cache_write_result saved =
            save_physics_asset_cache(cache_path, built.value, identity);
        require(saved.success, "Physics Asset Cache save should succeed.");

        const physics_asset_cache_read_result loaded =
            load_physics_asset_cache(cache_path, identity);
        require(loaded.success && !loaded.needs_rebuild,
            "Matching Physics Asset Cache identity should load without rebuilding.");
        require(loaded.value.rest_positions.size() == built.value.rest_positions.size(),
            "Loaded Physics Mesh vertex count should match the source.");
        require(loaded.value.tetrahedra.size() == built.value.tetrahedra.size(),
            "Loaded Physics Mesh tetrahedron count should match the source.");
        require(loaded.value.boundary_triangles.size() == built.value.boundary_triangles.size(),
            "Loaded Physics Mesh boundary count should match the source.");
        require(std::abs(loaded.value.tetrahedra[0].rest_volume - built.value.tetrahedra[0].rest_volume) < 1.0e-7f,
            "Loaded rest volume should match the source.");

        physics_asset_cache_identity changed_identity = identity;
        changed_identity.source_asset_hash = "sha256:changed";
        const physics_asset_cache_read_result stale =
            load_physics_asset_cache(cache_path, changed_identity);
        require(!stale.success && stale.needs_rebuild,
            "Stale source asset identity should require explicit cache regeneration.");
        require(!stale.diagnostics.empty() &&
                stale.diagnostics[0].code == physics_asset_cache_error::incompatible_identity,
            "Stale cache should report an identity mismatch.");

        {
            std::fstream stream(cache_path, std::ios::binary | std::ios::in | std::ios::out);
            require(static_cast<bool>(stream), "Could not reopen cache to corrupt checksum.");
            stream.seekg(-1, std::ios::end);
            char last_byte = 0;
            stream.read(&last_byte, 1);
            require(static_cast<bool>(stream), "Could not read cache checksum byte.");
            last_byte = static_cast<char>(last_byte ^ 0x5a);
            stream.seekp(-1, std::ios::end);
            stream.write(&last_byte, 1);
            require(static_cast<bool>(stream), "Could not corrupt cache checksum.");
        }

        const physics_asset_cache_read_result corrupted =
            load_physics_asset_cache(cache_path, identity);
        require(!corrupted.success && corrupted.needs_rebuild,
            "Corrupted cache checksum should require regeneration.");
        require(!corrupted.diagnostics.empty() &&
                corrupted.diagnostics[0].code == physics_asset_cache_error::corrupt_data,
            "Corrupted cache should report corrupt_data.");
    }

}

int run_physics_mesh_tests()
{
    test_msh_reader_maps_external_node_ids();
    std::cout << "[PASS] P02_MSH_NODE_ID_MAPPING\n";
    test_physics_mesh_builder_creates_rest_data_and_adjacency();
    std::cout << "[PASS] P02_TET_REST_DATA\n";
    test_persistent_physics_mesh_golden_fixtures();
    std::cout << "[PASS] P02_MESH_GOLDEN_FIXTURES\n";
    test_negative_tetrahedron_orientation_is_repaired();
    std::cout << "[PASS] P02_NEGATIVE_ORIENTATION\n";
    test_invalid_tetrahedron_index_is_rejected();
    std::cout << "[PASS] P02_INVALID_INDEX\n";
    test_quality_threshold_is_enforced();
    std::cout << "[PASS] P02_QUALITY_REJECTION\n";
    test_non_manifold_tetrahedron_faces_are_rejected();
    std::cout << "[PASS] P02_NON_MANIFOLD_REJECTION\n";
    test_skin_weight_harmonic_extension();
    std::cout << "[PASS] P02_SKIN_WEIGHT_HARMONIC\n";
    test_physics_attribute_weight_transfer_and_harmonic_extension();
    std::cout << "[PASS] P02_PHYSICS_ATTRIBUTE_HARMONIC\n";
    test_physics_asset_cache_round_trip_and_invalidation();
    std::cout << "[PASS] P02_ASSET_CACHE\n";
    test_ftetwild_resolution_settings_report_and_cache_identity();
    std::cout << "[PASS] P02_TETRAHEDRALIZATION_RESOLUTION\n";

    std::cout << "PhysicsMeshTests passed.\n";
    return 0;
}
