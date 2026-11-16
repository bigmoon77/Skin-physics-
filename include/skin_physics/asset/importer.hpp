#pragma once

#include <filesystem>

#include "skin_physics/asset/types.hpp"

namespace skin_physics::asset
{
    import_result import_model(const std::filesystem::path& path, const import_options& options = {});
    std::vector<diagnostic> validate_model(const model& asset);
}
