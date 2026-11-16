#include "skin_physics/physics/msh_reader.hpp"
#include "skin_physics/physics/physics_mesh_builder.hpp"

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
}

int run_physics_mesh_tests()
{
    test_msh_reader_maps_external_node_ids();
    test_physics_mesh_builder_creates_rest_data_and_adjacency();
    test_negative_tetrahedron_orientation_is_repaired();
    test_invalid_tetrahedron_index_is_rejected();
    test_quality_threshold_is_enforced();
    test_non_manifold_tetrahedron_faces_are_rejected();

    std::cout << "PhysicsMeshTests passed.\n";
    return 0;
}
