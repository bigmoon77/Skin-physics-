#pragma once

#include <filesystem>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::asset::detail
{
    import_result import_obj(const std::filesystem::path& path, const import_options& options);
    import_result import_gltf(const std::filesystem::path& path, const import_options& options);

    void append_diagnostic(
        import_result& result,
        diagnostic_severity severity,
        diagnostic_code code,
        const std::string& message,
        uint32_t line = 0);

    void generate_missing_normals_and_tangents(mesh& target);
}
