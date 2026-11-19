#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "skin_physics/serialization/json.hpp"

namespace skin_physics::simulation
{
    struct scenario_validation_result
    {
        serialization::json_value document;
        std::vector<std::string> applied_defaults;
        std::string diagnostic;
        bool success = false;
    };

    struct run_result_validation_result
    {
        serialization::json_value document;
        std::string diagnostic;
        bool success = false;
    };

    scenario_validation_result validate_scenario(const serialization::json_value& document);
    scenario_validation_result parse_scenario(const std::string& text);
    scenario_validation_result load_scenario(const std::filesystem::path& path);
    bool save_scenario(
        const std::filesystem::path& path,
        const serialization::json_value& document,
        std::string& diagnostic);

    run_result_validation_result validate_run_result(const serialization::json_value& document);
    run_result_validation_result parse_run_result(const std::string& text);
    run_result_validation_result load_run_result(const std::filesystem::path& path);
    bool save_run_result(
        const std::filesystem::path& path,
        const serialization::json_value& document,
        std::string& diagnostic);

    bool run_results_comparable(
        const serialization::json_value& left,
        const serialization::json_value& right);
}
