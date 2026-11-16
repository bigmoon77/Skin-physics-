#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "skin_physics/asset/types.hpp"
#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    struct ftetwild_settings
    {
        std::filesystem::path executable_path;
        float target_edge_length = 0.05f;
        float relative_envelope = 1.0e-3f;
        uint32_t timeout_milliseconds = 300000;
        bool keep_temporary_files_on_failure = true;
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
        output_read_error
    };

    struct tetrahedralizer_diagnostic
    {
        tetrahedralizer_error code = tetrahedralizer_error::none;
        std::string message;
        std::filesystem::path retained_directory;
    };

    struct tetrahedralizer_result
    {
        raw_tet_mesh mesh;
        std::vector<tetrahedralizer_diagnostic> diagnostics;
        bool success = false;
    };

    // Runs an externally installed fTetWild executable. fTetWild is not linked into the runtime library.
    tetrahedralizer_result run_ftetwild(
        const asset::mesh& surface_mesh,
        const ftetwild_settings& settings);
}
