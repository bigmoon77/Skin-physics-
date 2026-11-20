#include "skin_physics/asset/importer.hpp"
#include "test_support.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int run_physics_mesh_tests();
int run_simulation_tests();

namespace
{
    using namespace skin_physics::asset;

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
            std::filesystem::temp_directory_path() / "skin_physics_asset_tests";
        std::filesystem::create_directories(directory);
        return directory;
    }

    void write_text(const std::filesystem::path& path, const std::string& content)
    {
        std::ofstream file(path, std::ios::binary);
        require(static_cast<bool>(file), "Could not create test file: " + path.string());
        file.write(content.data(), static_cast<std::streamsize>(content.size()));
        require(static_cast<bool>(file), "Could not write test file: " + path.string());
    }

    void write_skin_animation_gltf_fixture(
        const std::filesystem::path& gltf_path,
        const std::filesystem::path& buffer_path)
    {
        const std::array<float, 9> positions = {
            0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f
        };
        const std::array<uint16_t, 3> indices = { 0, 1, 2 };
        const std::array<uint8_t, 12> joints = {
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0
        };
        const std::array<float, 12> weights = {
            1.0f, 0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f, 0.0f
        };
        const std::array<float, 2> times = { 0.0f, 1.0f };
        const std::array<float, 6> translations = {
            0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f
        };
        const std::array<float, 3> pin_weights = { 0.0f, 0.5f, 1.0f };
        const std::array<float, 16> inverse_bind = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };

        std::ofstream binary(buffer_path, std::ios::binary);
        require(static_cast<bool>(binary), "Could not create glTF test buffer.");

        binary.write(reinterpret_cast<const char*>(positions.data()), sizeof(positions));
        binary.write(reinterpret_cast<const char*>(indices.data()), sizeof(indices));
        const uint16_t alignment_padding = 0;
        binary.write(reinterpret_cast<const char*>(&alignment_padding), sizeof(alignment_padding));
        binary.write(reinterpret_cast<const char*>(joints.data()), sizeof(joints));
        binary.write(reinterpret_cast<const char*>(weights.data()), sizeof(weights));
        binary.write(reinterpret_cast<const char*>(times.data()), sizeof(times));
        binary.write(reinterpret_cast<const char*>(translations.data()), sizeof(translations));
        binary.write(reinterpret_cast<const char*>(inverse_bind.data()), sizeof(inverse_bind));
        binary.write(reinterpret_cast<const char*>(pin_weights.data()), sizeof(pin_weights));
        require(static_cast<bool>(binary), "Could not write glTF test buffer.");

        const std::string json = R"({
            "asset": { "version": "2.0", "generator": "SkinPhysicsTests" },
            "buffers": [{ "uri": "skin_animation.bin", "byteLength": 212 }],
            "bufferViews": [
                { "buffer": 0, "byteOffset": 0, "byteLength": 36, "target": 34962 },
                { "buffer": 0, "byteOffset": 36, "byteLength": 6, "target": 34963 },
                { "buffer": 0, "byteOffset": 44, "byteLength": 12, "target": 34962 },
                { "buffer": 0, "byteOffset": 56, "byteLength": 48, "target": 34962 },
                { "buffer": 0, "byteOffset": 104, "byteLength": 8 },
                { "buffer": 0, "byteOffset": 112, "byteLength": 24 },
                { "buffer": 0, "byteOffset": 136, "byteLength": 64 },
                { "buffer": 0, "byteOffset": 200, "byteLength": 12 }
            ],
            "accessors": [
                { "bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3" },
                { "bufferView": 1, "componentType": 5123, "count": 3, "type": "SCALAR" },
                { "bufferView": 2, "componentType": 5121, "count": 3, "type": "VEC4" },
                { "bufferView": 3, "componentType": 5126, "count": 3, "type": "VEC4" },
                { "bufferView": 4, "componentType": 5126, "count": 2, "type": "SCALAR" },
                { "bufferView": 6, "componentType": 5126, "count": 1, "type": "MAT4" },
                { "bufferView": 5, "componentType": 5126, "count": 2, "type": "VEC3" },
                { "bufferView": 7, "componentType": 5126, "count": 3, "type": "SCALAR" }
            ],
            "meshes": [{
                "name": "Triangle",
                "primitives": [{
                    "attributes": { "POSITION": 0, "JOINTS_0": 2, "WEIGHTS_0": 3, "_PIN_WEIGHT": 7 },
                    "indices": 1,
                    "mode": 4
                }]
            }],
            "nodes": [
                { "name": "Body", "mesh": 0, "skin": 0, "children": [1] },
                { "name": "RootJoint" }
            ],
            "skins": [{
                "name": "BodySkin",
                "skeleton": 1,
                "joints": [1],
                "inverseBindMatrices": 5
            }],
            "animations": [{
                "name": "Raise",
                "samplers": [{ "input": 4, "output": 6, "interpolation": "LINEAR" }],
                "channels": [{ "sampler": 0, "target": { "node": 1, "path": "translation" } }]
            }],
            "scenes": [{ "nodes": [0] }],
            "scene": 0
        })";
        write_text(gltf_path, json);
    }

    void test_obj_negative_indices_and_polygon()
    {
        const std::filesystem::path directory = test_directory();
        const std::filesystem::path material_path = directory / "basic.mtl";
        const std::filesystem::path obj_path = directory / "quad.obj";

        write_text(material_path,
            "newmtl cloth\n"
            "Kd 0.2 0.4 0.8\n"
            "Pr 0.7\n"
            "Pm 0.1\n");

        write_text(obj_path,
            "mtllib basic.mtl\n"
            "o quad\n"
            "v 0 0 0\n"
            "v 100 0 0\n"
            "v 100 100 0\n"
            "v 0 100 0\n"
            "vt 0 0\n"
            "vt 1 0\n"
            "vt 1 1\n"
            "vt 0 1\n"
            "s 1\n"
            "usemtl cloth\n"
            "f -4/-4 -3/-3 -2/-2 -1/-1\n");

        import_options options;
        options.has_obj_unit_scale = true;
        options.obj_unit_scale = 0.01f;
        const import_result result = import_model(obj_path, options);

        require(result.success, "OBJ import should succeed.");
        require(result.asset.meshes.size() == 1, "OBJ should create one mesh.");
        require(result.asset.meshes[0].vertices.size() == 4, "Quad should create four deduplicated vertices.");
        require(result.asset.meshes[0].indices.size() == 6, "Quad should be triangulated into two triangles.");
        require(result.asset.materials.size() == 1, "MTL material should be imported.");
        require(result.asset.meshes[0].submeshes.size() == 1, "OBJ should create one submesh.");
        require(std::abs(result.asset.meshes[0].vertices[1].position.x - 1.0f) < 1.0e-5f,
            "OBJ unit scale should convert centimeters to meters.");
        require((result.asset.meshes[0].vertices[0].attributes & attribute_normal) != 0,
            "Missing normals should be generated.");
        require((result.asset.meshes[0].vertices[0].attributes & attribute_tangent) != 0,
            "Tangents should be generated when UVs are present.");
    }

    void test_obj_requires_explicit_unit_scale()
    {
        const std::filesystem::path obj_path =
            test_support::fixture_path("phase01_importer/unit_missing.obj");
        const import_result result = import_model(obj_path);
        require(!result.success, "OBJ import without an explicit unit scale must fail.");
        require(!result.diagnostics.empty(), "Missing OBJ unit scale should be diagnosed.");
        require(result.diagnostics[0].code == diagnostic_code::invalid_unit_scale,
            "Missing OBJ unit scale should report invalid_unit_scale.");
    }

    void test_gltf_skin_and_animation()
    {
        const std::filesystem::path directory = test_directory();
        const std::filesystem::path gltf_path = directory / "skin_animation.gltf";
        const std::filesystem::path buffer_path = directory / "skin_animation.bin";
        write_skin_animation_gltf_fixture(gltf_path, buffer_path);

        const import_result result = import_model(gltf_path);
        require(result.success, "glTF Skin / Animation fixture should import.");
        require(result.asset.meshes.size() == 1, "glTF should import one mesh.");
        require(result.asset.meshes[0].vertices.size() == 3, "glTF should import three vertices.");
        require(result.asset.meshes[0].indices.size() == 3, "glTF should import one triangle.");
        require(result.asset.meshes[0].vertex_groups.size() == 1, "glTF custom scalar attribute should become a named vertex group.");
        require(result.asset.meshes[0].vertex_groups[0].name == "_PIN_WEIGHT", "Named vertex group should preserve its attribute name.");
        require(std::abs(result.asset.meshes[0].vertex_groups[0].weights[1] - 0.5f) < 1.0e-6f,
            "Named vertex group weights should be imported.");
        require(result.asset.nodes.size() == 2, "glTF should import its node hierarchy.");
        require(result.asset.skins.size() == 1, "glTF should import one skin.");
        require(result.asset.skins[0].joints.size() == 1, "glTF skin should contain one joint.");
        require(result.asset.animations.size() == 1, "glTF should import one animation clip.");
        require(result.asset.animations[0].tracks.size() == 1, "glTF should import one animation track.");
        require(result.asset.animations[0].tracks[0].keys.size() == 2, "glTF animation should import two keys.");

        const vertex& first_vertex = result.asset.meshes[0].vertices[0];
        require((first_vertex.attributes & attribute_skinning) != 0, "Skin attributes should be present.");
        require(first_vertex.joints[0] == 0, "Skin joint index should be preserved.");
        require(std::abs(first_vertex.weights[0] - 1.0f) < 1.0e-6f,
            "Skin weights should be normalized.");
    }


    void test_persistent_obj_golden_fixture()
    {
        const std::filesystem::path fixture_path =
            test_support::fixture_path("phase01_importer/quad.obj");

        import_options options;
        options.has_obj_unit_scale = true;
        options.obj_unit_scale = 0.01f;

        const import_result result = import_model(fixture_path, options);
        require(result.success, "Persistent OBJ golden fixture should import successfully.");
        require(result.asset.meshes.size() == 1, "Golden OBJ should create one mesh.");
        require(result.asset.meshes[0].vertices.size() == 4, "Golden OBJ should create four deduplicated vertices.");
        require(result.asset.meshes[0].indices.size() == 6, "Golden OBJ should triangulate into two triangles.");
        require(result.asset.meshes[0].submeshes.size() == 1, "Golden OBJ should create one submesh.");
        require(result.asset.materials.size() == 1, "Golden OBJ should load its sibling MTL file.");
        require(std::abs(result.asset.meshes[0].vertices[1].position.x - 1.0f) < 1.0e-6f,
            "Golden OBJ centimeter coordinates should convert to meters.");
        require(std::abs(result.asset.materials[0].base_color_factor.x - 0.2f) < 1.0e-6f &&
                std::abs(result.asset.materials[0].base_color_factor.y - 0.4f) < 1.0e-6f &&
                std::abs(result.asset.materials[0].base_color_factor.z - 0.8f) < 1.0e-6f,
            "Golden OBJ MTL base color should match its reference values.");
        require(std::abs(result.asset.materials[0].roughness_factor - 0.7f) < 1.0e-6f &&
                std::abs(result.asset.materials[0].metallic_factor - 0.1f) < 1.0e-6f,
            "Golden OBJ MTL material factors should match the reference.");
        require(std::filesystem::exists(test_support::fixture_path("phase01_importer/obj_quad.expected.json")),
            "Golden OBJ expected-result file must exist.");
    }

    void test_persistent_gltf_golden_fixture()
    {
        const std::filesystem::path fixture_path =
            test_support::fixture_path("phase01_importer/skinned_triangle.gltf");
        const import_result result = import_model(fixture_path);

        require(result.success, "Persistent glTF golden fixture should import successfully.");
        require(result.asset.meshes.size() == 1, "Golden glTF should create one mesh.");
        require(result.asset.meshes[0].vertices.size() == 3, "Golden glTF should contain three vertices.");
        require(result.asset.meshes[0].indices == std::vector<uint32_t>({ 0, 1, 2 }),
            "Golden glTF triangle indices should remain unchanged.");
        require(result.asset.meshes[0].vertex_groups.size() == 1,
            "Golden glTF should import the named custom vertex group.");
        require(result.asset.meshes[0].vertex_groups[0].name == "_PIN_WEIGHT",
            "Golden glTF should preserve the custom vertex group name.");
        require(std::abs(result.asset.meshes[0].vertex_groups[0].weights[0] - 0.0f) < 1.0e-6f &&
                std::abs(result.asset.meshes[0].vertex_groups[0].weights[1] - 0.5f) < 1.0e-6f &&
                std::abs(result.asset.meshes[0].vertex_groups[0].weights[2] - 1.0f) < 1.0e-6f,
            "Golden glTF custom vertex group weights should match the reference.");
        require(result.asset.skins.size() == 1 && result.asset.skins[0].joints.size() == 1,
            "Golden glTF skin and joint count should match the reference.");
        require(result.asset.skins[0].inverse_bind_matrices.size() == 1,
            "Golden glTF should import its inverse bind matrix.");
        require(result.asset.animations.size() == 1 && result.asset.animations[0].tracks.size() == 1,
            "Golden glTF should import exactly one animation track.");
        require(result.asset.animations[0].tracks[0].keys.size() == 2,
            "Golden glTF animation should contain two reference keys.");
        require(std::abs(result.asset.animations[0].tracks[0].keys[1].time - 1.0f) < 1.0e-6f &&
                std::abs(result.asset.animations[0].tracks[0].keys[1].value[1] - 1.0f) < 1.0e-6f,
            "Golden glTF final animation key should translate the joint by one meter.");
        require(std::filesystem::exists(test_support::fixture_path("phase01_importer/gltf_skin.expected.json")),
            "Golden glTF expected-result file must exist.");
    }

    template<typename TestFunction>
    void run_test_case(const char* test_id, TestFunction test_function)
    {
        test_function();
        std::cout << "[PASS] " << test_id << "\n";
    }

    void test_fixture_catalog_files_exist()
    {
        const std::vector<const char*> fixture_files = {
            "README.md", "manifest.json",
            "phase01_importer/quad.obj",
            "phase01_importer/basic.mtl",
            "phase01_importer/skinned_triangle.gltf",
            "phase01_importer/obj_quad.expected.json",
            "phase01_importer/gltf_skin.expected.json",
            "phase01_importer/unit_missing.obj",
            "phase01_importer/unit_missing.expected.json",
            "phase02_physics_mesh/single_tet.msh",
            "phase02_physics_mesh/two_tet_shared_face.msh",
            "phase02_physics_mesh/center_star.msh",
            "phase02_physics_mesh/single_tet.expected.json",
            "phase02_physics_mesh/two_tet.expected.json",
            "phase02_physics_mesh/center_star_skin_weights.expected.json",
            "phase02_physics_mesh/resolution.input.json",
            "phase02_physics_mesh/resolution.expected.json",
            "phase03_render_binding/tet_binding.input.json",
            "phase03_render_binding/tet_binding.expected.json",
            "phase03_render_binding/tet_binding.invalid.json",
            "phase03_render_binding/tet_binding.invalid.expected.json",
            "phase04_test_environment/fixed_step.input.json",
            "phase04_test_environment/fixed_step.expected.json",
            "phase04_test_environment/scenario.input.json",
            "phase04_test_environment/scenario.expected.json",
            "phase04_test_environment/run_result.input.json",
            "phase04_test_environment/run_result.expected.json",
            "phase05_cloth/cloth_patch.input.json",
            "phase05_cloth/cloth_patch.expected.json",
            "phase06_soft_body/tet_deformation.input.json",
            "phase06_soft_body/tet_deformation.expected.json",
            "phase07_collision/vt_contact.input.json",
            "phase07_collision/vt_contact.expected.json",
            "phase07_collision/ee_contact.input.json",
            "phase07_collision/ee_contact.expected.json",
            "phase07_collision/broadphase.input.json",
            "phase07_collision/broadphase.expected.json",
            "phase07_collision/overflow.input.json",
            "phase07_collision/overflow.expected.json",
            "phase08_ccd/point_triangle.input.json",
            "phase08_ccd/point_triangle.expected.json",
            "phase08_ccd/initial_repair.input.json",
            "phase08_ccd/initial_repair.expected.json",
            "phase09_animation/target_motion.input.json",
            "phase09_animation/target_motion.expected.json",
            "phase09_animation/animation_trajectory_abi.cs.hlsl",
            "phase10_paint/paint.input.json",
            "phase10_paint/paint.expected.json",
            "phase11_performance/scale_profiles.input.json",
            "phase11_performance/scale_profiles.expected.json",
            "design_review/correction_sweep_broadphase.json"
        };

        for (const char* relative_path : fixture_files)
        {
            const std::filesystem::path path = test_support::fixture_path(relative_path);
            require(std::filesystem::file_size(path) > 0,
                std::string("Golden fixture file is empty: ") + relative_path);
        }
    }
    void test_model_validation_rejects_bad_indices()
    {
        model invalid_model;
        mesh invalid_mesh;
        invalid_mesh.vertices.resize(3);
        invalid_mesh.indices = { 0, 1, 3 };
        submesh section;
        section.first_index = 0;
        section.index_count = 3;
        invalid_mesh.submeshes.push_back(section);
        invalid_model.meshes.push_back(invalid_mesh);

        const std::vector<diagnostic> diagnostics = validate_model(invalid_model);
        bool found_invalid_index = false;
        for (const diagnostic& item : diagnostics)
        {
            if (item.code == diagnostic_code::invalid_index)
            {
                found_invalid_index = true;
                break;
            }
        }
        require(found_invalid_index, "Model validation should reject out-of-range indices.");
    }
}

int main()
{
    try
    {
        run_test_case("P01_OBJ_TEMP_EDGE_CASES", test_obj_negative_indices_and_polygon);
        run_test_case("P01_OBJ_UNIT_ERROR", test_obj_requires_explicit_unit_scale);
        run_test_case("P01_GLTF_TEMP_SKIN_ANIMATION", test_gltf_skin_and_animation);
        run_test_case("P01_OBJ_GOLDEN", test_persistent_obj_golden_fixture);
        run_test_case("P01_GLTF_GOLDEN", test_persistent_gltf_golden_fixture);
        run_test_case("FIXTURE_CATALOG", test_fixture_catalog_files_exist);
        run_test_case("P01_MODEL_VALIDATION", test_model_validation_rejects_bad_indices);
        require(run_physics_mesh_tests() == 0, "Physics Mesh tests failed.");
        require(run_simulation_tests() == 0, "Simulation core tests failed.");
    }
    catch (const std::exception& exception)
    {
        std::cerr << "SkinPhysicsTests failed: " << exception.what() << "\n";
        return 1;
    }

    std::cout << "SkinPhysicsTests passed.\n";
    return 0;
}
