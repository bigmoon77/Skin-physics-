#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"
#include "skin_physics/physics/asset_cache.hpp"
#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    enum class ftetwild_resolution_mode : uint8_t
    {
        target_edge_length,
        target_tetrahedron_count
    };

    struct ftetwild_settings
    {
        std::filesystem::path executable_path;

        // Exactly one selector must be active:
        // target_edge_length > 0 with target_tetrahedron_count == 0, or
        // target_edge_length == 0 with target_tetrahedron_count > 0.
        float target_edge_length = 0.05f;
        uint32_t target_tetrahedron_count = 0;
        uint32_t maximum_tetrahedra = 1'000'000;

        float surface_approximation_tolerance = 1.0e-3f;
        float tetra_quality_threshold = 1.0e-5f;
        uint32_t timeout_milliseconds = 300000;
        bool keep_temporary_files_on_failure = true;

        std::string generator_id = "fTetWild";
        std::string generator_version = "unversioned";
    };

    struct resolved_ftetwild_settings
    {
        ftetwild_resolution_mode resolution_mode =
            ftetwild_resolution_mode::target_edge_length;
        float target_edge_length = 0.0f;
        float surface_bbox_diagonal = 0.0f;
        uint32_t requested_tetrahedron_count = 0;
        uint32_t maximum_tetrahedra = 0;
        float minimum_tetra_quality = 0.0f;
        bool edge_length_is_estimated = false;
    };

    enum class tetrahedralizer_error : uint8_t
    {
        none,
        unsupported_platform,
        executable_not_found,
        invalid_input_mesh,
        invalid_settings,
        temporary_directory_error,
        input_write_error,
        process_start_error,
        process_timeout,
        process_failed,
        output_read_error,
        invalid_output_mesh,
        capacity_exceeded
    };

    struct tetrahedralizer_diagnostic
    {
        tetrahedralizer_error code = tetrahedralizer_error::none;
        std::string message;
        std::filesystem::path retained_directory;
    };

    struct tetrahedralization_report
    {
        resolved_ftetwild_settings resolution;
        uint64_t actual_vertex_count = 0;
        uint64_t actual_tetrahedron_count = 0;
        float minimum_edge_length = 0.0f;
        float average_edge_length = 0.0f;
        float maximum_edge_length = 0.0f;
        float minimum_tetra_quality = 0.0f;
        float average_tetra_quality = 0.0f;
        float maximum_tetra_quality = 0.0f;
        float target_tetrahedron_count_relative_error = 0.0f;
        bool has_edge_statistics = false;
    };

    struct tetrahedralizer_result
    {
        raw_tet_mesh mesh;
        tetrahedralization_report report;
        std::vector<tetrahedralizer_diagnostic> diagnostics;
        bool success = false;
    };

    struct ftetwild_resolution_result
    {
        resolved_ftetwild_settings value;
        std::vector<tetrahedralizer_diagnostic> diagnostics;
        bool success = false;
    };

    ftetwild_resolution_result resolve_ftetwild_settings(
        const asset::mesh& surface_mesh,
        const ftetwild_settings& settings);

    // Includes all generation and quality settings in a deterministic cache identity option string.
    std::string make_ftetwild_cache_options(
        const ftetwild_settings& settings,
        const resolved_ftetwild_settings& resolved);

    bool apply_ftetwild_cache_identity(
        const ftetwild_settings& settings,
        const resolved_ftetwild_settings& resolved,
        physics_asset_cache_identity& identity);

    bool measure_tetrahedralization_output(
        const raw_tet_mesh& mesh,
        const resolved_ftetwild_settings& resolved,
        tetrahedralization_report& report,
        std::string& error_message);

    // Runs an externally installed fTetWild executable. fTetWild is not linked into the runtime library.
    tetrahedralizer_result run_ftetwild(
        const asset::mesh& surface_mesh,
        const ftetwild_settings& settings);
}
