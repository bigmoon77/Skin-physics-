#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "skin_physics/physics/physics_mesh.hpp"

namespace skin_physics::physics
{
    struct physics_asset_cache_identity
    {
        uint32_t schema_version = 1;
        std::string source_asset_id;
        std::string source_asset_hash;
        std::string generator_id;
        std::string generator_version;
        std::string generator_options;
        uint32_t coordinate_system_version = 1;
        uint32_t material_layout_version = 1;
    };

    enum class physics_asset_cache_error : uint8_t
    {
        none,
        io_error,
        invalid_identity,
        invalid_mesh,
        unsupported_version,
        incompatible_identity,
        corrupt_data,
        capacity_exceeded
    };

    struct physics_asset_cache_diagnostic
    {
        physics_asset_cache_error code = physics_asset_cache_error::none;
        std::string message;
    };

    struct physics_asset_cache_write_result
    {
        std::vector<physics_asset_cache_diagnostic> diagnostics;
        bool success = false;
    };

    struct physics_asset_cache_read_result
    {
        physics_mesh value;
        physics_asset_cache_identity identity;
        std::vector<physics_asset_cache_diagnostic> diagnostics;
        bool needs_rebuild = false;
        bool success = false;
    };

    // The cache uses an explicit little-endian binary layout and a payload checksum.
    // Cache identity must match exactly; this function never silently accepts stale data.
    physics_asset_cache_write_result save_physics_asset_cache(
        const std::filesystem::path& path,
        const physics_mesh& value,
        const physics_asset_cache_identity& identity);

    physics_asset_cache_read_result load_physics_asset_cache(
        const std::filesystem::path& path,
        const physics_asset_cache_identity& expected_identity,
        uint64_t maximum_cache_bytes = 8ull * 1024ull * 1024ull * 1024ull);
}
