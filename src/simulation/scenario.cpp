#include "skin_physics/simulation/scenario.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace skin_physics::simulation
{
    namespace
    {
        using serialization::json_value;

        const json_value* field(const json_value& value, const std::string& name)
        {
            return value.find(name);
        }

        bool object_field(const json_value& value, const std::string& name)
        {
            const json_value* item = field(value, name);
            return item != nullptr && item->is_object();
        }

        bool array_field(const json_value& value, const std::string& name)
        {
            const json_value* item = field(value, name);
            return item != nullptr && item->is_array();
        }

        bool string_field(
            const json_value& value,
            const std::string& name,
            std::string* output = nullptr)
        {
            const json_value* item = field(value, name);
            if (item == nullptr || !item->is_string() || item->as_string().empty())
            {
                return false;
            }
            if (output != nullptr) *output = item->as_string();
            return true;
        }

        bool number_field(
            const json_value& value,
            const std::string& name,
            double minimum,
            double maximum,
            double* output = nullptr)
        {
            const json_value* item = field(value, name);
            if (item == nullptr || !item->is_number())
            {
                return false;
            }
            const double number = item->as_number();
            if (!std::isfinite(number) || number < minimum || number > maximum)
            {
                return false;
            }
            if (output != nullptr) *output = number;
            return true;
        }

        bool integer_field(
            const json_value& value,
            const std::string& name,
            uint64_t minimum,
            uint64_t maximum)
        {
            double number = 0.0;
            if (!number_field(value, name, static_cast<double>(minimum),
                static_cast<double>(maximum), &number))
            {
                return false;
            }
            return std::floor(number) == number;
        }

        bool vector3_field(const json_value& value, const std::string& name)
        {
            const json_value* item = field(value, name);
            if (item == nullptr || !item->is_array() || item->as_array().size() != 3)
            {
                return false;
            }
            for (const json_value& component : item->as_array())
            {
                if (!component.is_number() || !std::isfinite(component.as_number()))
                {
                    return false;
                }
            }
            return true;
        }

        bool validate_scenario_document(const json_value& document, std::string& diagnostic)
        {
            if (!document.is_object())
            {
                diagnostic = "Scenario root must be a JSON object.";
                return false;
            }

            if (!integer_field(document, "schema_version", 1, 1))
            {
                diagnostic = "Scenario schema_version is missing or unsupported.";
                return false;
            }
            if (!string_field(document, "asset_id") ||
                !integer_field(document, "asset_cache_version", 1,
                    std::numeric_limits<uint32_t>::max()))
            {
                diagnostic = "Scenario asset_id or asset_cache_version is invalid.";
                return false;
            }

            if (!object_field(document, "asset_generation_settings"))
            {
                diagnostic = "Scenario asset_generation_settings must be an object.";
                return false;
            }
            const json_value& generation = *field(document, "asset_generation_settings");
            const bool has_edge_length = field(generation, "target_tet_edge_length") != nullptr;
            const bool has_tet_count = field(generation, "target_tet_count") != nullptr;
            if (has_edge_length == has_tet_count)
            {
                diagnostic = "Exactly one of target_tet_edge_length or target_tet_count is required.";
                return false;
            }
            if (has_edge_length &&
                !number_field(generation, "target_tet_edge_length",
                    std::numeric_limits<double>::min(),
                    std::numeric_limits<double>::max()))
            {
                diagnostic = "target_tet_edge_length must be finite and positive.";
                return false;
            }
            if (has_tet_count &&
                !integer_field(generation, "target_tet_count", 1,
                    std::numeric_limits<uint32_t>::max()))
            {
                diagnostic = "target_tet_count must be a positive integer.";
                return false;
            }
            if (!integer_field(generation, "max_tet_count", 1,
                std::numeric_limits<uint32_t>::max()))
            {
                diagnostic = "max_tet_count must be a positive integer.";
                return false;
            }
            if (has_tet_count)
            {
                const double target = field(generation, "target_tet_count")->as_number();
                const double maximum = field(generation, "max_tet_count")->as_number();
                if (target > maximum)
                {
                    diagnostic = "target_tet_count exceeds max_tet_count.";
                    return false;
                }
            }

            if (!object_field(document, "solver_settings"))
            {
                diagnostic = "Scenario solver_settings must be an object.";
                return false;
            }
            const json_value& solver = *field(document, "solver_settings");
            if (!number_field(solver, "fixed_delta_time",
                std::numeric_limits<double>::min(), 1.0) ||
                !number_field(solver, "time_scale", 0.0, 1000.0) ||
                !integer_field(solver, "substeps", 1, 1024) ||
                !integer_field(solver, "iterations", 1, 100000))
            {
                diagnostic = "Scenario solver_settings contains an invalid value.";
                return false;
            }

            if (!object_field(document, "initial_state") ||
                !array_field(*field(document, "initial_state"), "positions"))
            {
                diagnostic = "Scenario initial_state.positions is required.";
                return false;
            }
            const json_value& positions = *field(*field(document, "initial_state"), "positions");
            if (positions.as_array().empty())
            {
                diagnostic = "Scenario initial_state.positions must not be empty.";
                return false;
            }
            for (const json_value& position : positions.as_array())
            {
                if (!position.is_array() || position.as_array().size() != 3)
                {
                    diagnostic = "Every initial position must contain three components.";
                    return false;
                }
                for (const json_value& component : position.as_array())
                {
                    if (!component.is_number() || !std::isfinite(component.as_number()))
                    {
                        diagnostic = "Initial positions must be finite.";
                        return false;
                    }
                }
            }

            if (!object_field(document, "animation"))
            {
                diagnostic = "Scenario animation must be an object.";
                return false;
            }
            const json_value& animation = *field(document, "animation");
            if (!string_field(animation, "clip") ||
                !number_field(animation, "time", 0.0, std::numeric_limits<double>::max()) ||
                !number_field(animation, "playback_speed",
                    -1000.0, 1000.0) ||
                !string_field(animation, "discontinuity_policy"))
            {
                diagnostic = "Scenario animation fields are invalid.";
                return false;
            }
            if (!vector3_field(document, "gravity"))
            {
                diagnostic = "Scenario gravity must be a finite three-component vector.";
                return false;
            }

            if (!array_field(document, "forces"))
            {
                diagnostic = "Scenario forces must be an array.";
                return false;
            }
            for (const json_value& force : field(document, "forces")->as_array())
            {
                std::string kind;
                if (!string_field(force, "kind", &kind) ||
                    (kind != "impulse" && kind != "force") ||
                    !vector3_field(force, "position") ||
                    !vector3_field(force, "direction") ||
                    !number_field(force, "strength", 0.0, std::numeric_limits<double>::max()) ||
                    !number_field(force, "radius", 0.0, std::numeric_limits<double>::max()) ||
                    !number_field(force, "duration", 0.0, std::numeric_limits<double>::max()) ||
                    !integer_field(force, "start_step", 0, std::numeric_limits<uint32_t>::max()))
                {
                    diagnostic = "Scenario force entry contains an invalid field.";
                    return false;
                }
                const json_value& direction = *field(force, "direction");
                const double x = direction.as_array()[0].as_number();
                const double y = direction.as_array()[1].as_number();
                const double z = direction.as_array()[2].as_number();
                if (x * x + y * y + z * z <= 1.0e-20)
                {
                    diagnostic = "Scenario force direction must be non-zero.";
                    return false;
                }
                const double duration = field(force, "duration")->as_number();
                if ((kind == "impulse" && duration != 0.0) ||
                    (kind == "force" && duration <= 0.0))
                {
                    diagnostic = "Impulse duration must be zero and force duration must be positive.";
                    return false;
                }
            }

            if (!object_field(document, "collisions"))
            {
                diagnostic = "Scenario collisions must be an object.";
                return false;
            }
            const json_value& collisions = *field(document, "collisions");
            if (!number_field(collisions, "thickness", 0.0, 1000.0) ||
                !number_field(collisions, "friction", 0.0, 1.0) ||
                !number_field(collisions, "restitution", 0.0, 1.0) ||
                !array_field(collisions, "excluded_mesh_pairs"))
            {
                diagnostic = "Scenario collision settings are invalid.";
                return false;
            }
            for (const json_value& pair : field(collisions, "excluded_mesh_pairs")->as_array())
            {
                if (!pair.is_array() || pair.as_array().size() != 2 ||
                    !pair.as_array()[0].is_string() || !pair.as_array()[1].is_string())
                {
                    diagnostic = "Every excluded mesh pair must contain two mesh identifiers.";
                    return false;
                }
            }

            if (!object_field(document, "materials"))
            {
                diagnostic = "Scenario materials must be an object.";
                return false;
            }
            const json_value& materials = *field(document, "materials");
            if (!number_field(materials, "density", std::numeric_limits<double>::min(),
                    std::numeric_limits<double>::max()) ||
                !number_field(materials, "mu", std::numeric_limits<double>::min(),
                    std::numeric_limits<double>::max()) ||
                !number_field(materials, "K", std::numeric_limits<double>::min(),
                    std::numeric_limits<double>::max()) ||
                !number_field(materials, "shape_recovery_scale", 0.0, 1.0e6) ||
                !number_field(materials, "volume_retention_scale", 0.0, 1.0e6))
            {
                diagnostic = "Scenario material parameters are invalid.";
                return false;
            }

            if (!array_field(document, "vertex_weights") ||
                !object_field(document, "test_sequence") ||
                !integer_field(document, "random_seed", 0,
                    std::numeric_limits<uint32_t>::max()))
            {
                diagnostic = "Scenario vertex_weights, test_sequence or random_seed is invalid.";
                return false;
            }
            const json_value& sequence = *field(document, "test_sequence");
            if (!integer_field(sequence, "start_step", 0, std::numeric_limits<uint32_t>::max()) ||
                !integer_field(sequence, "end_step", 0, std::numeric_limits<uint32_t>::max()) ||
                field(sequence, "end_step")->as_number() < field(sequence, "start_step")->as_number())
            {
                diagnostic = "Scenario test_sequence range is invalid.";
                return false;
            }

            diagnostic.clear();
            return true;
        }

        bool validate_run_result_document(
            const json_value& document,
            std::string& diagnostic)
        {
            if (!document.is_object())
            {
                diagnostic = "Run Result root must be a JSON object.";
                return false;
            }

            if (!string_field(document, "scenario_id") ||
                !string_field(document, "scenario_hash") ||
                !string_field(document, "asset_id") ||
                !integer_field(document, "cache_version", 1,
                    std::numeric_limits<uint32_t>::max()) ||
                !string_field(document, "build_id") ||
                !object_field(document, "gpu") ||
                !array_field(document, "passes") ||
                !integer_field(document, "library_allocated_bytes", 0,
                    std::numeric_limits<uint64_t>::max()) ||
                !integer_field(document, "candidate_count", 0,
                    std::numeric_limits<uint64_t>::max()) ||
                !integer_field(document, "contact_count", 0,
                    std::numeric_limits<uint64_t>::max()) ||
                !integer_field(document, "overflow_count", 0,
                    std::numeric_limits<uint64_t>::max()) ||
                !integer_field(document, "ccd_count", 0,
                    std::numeric_limits<uint64_t>::max()) ||
                !number_field(document, "maximum_volume_error", 0.0,
                    std::numeric_limits<double>::max()) ||
                !number_field(document, "maximum_displacement", 0.0,
                    std::numeric_limits<double>::max()))
            {
                diagnostic = "Run Result is missing a required field or contains an invalid value.";
                return false;
            }

            const json_value& gpu = *field(document, "gpu");
            const json_value* available = field(gpu, "available");
            if (available == nullptr || !available->is_bool())
            {
                diagnostic = "Run Result gpu.available must be boolean.";
                return false;
            }
            if (!available->as_bool() &&
                !string_field(gpu, "reason"))
            {
                diagnostic = "Unavailable GPU measurements must include a reason.";
                return false;
            }

            for (const json_value& pass : field(document, "passes")->as_array())
            {
                if (!string_field(pass, "name") ||
                    !number_field(pass, "cpu_time_ms", 0.0,
                        std::numeric_limits<double>::max()))
                {
                    diagnostic = "Run Result pass name or CPU time is invalid.";
                    return false;
                }
                const json_value* gpu_time = field(pass, "gpu_time_ms");
                if (gpu_time == nullptr ||
                    (!gpu_time->is_null() &&
                        (!gpu_time->is_number() || !std::isfinite(gpu_time->as_number()) ||
                            gpu_time->as_number() < 0.0)))
                {
                    diagnostic = "Run Result GPU time must be non-negative or null.";
                    return false;
                }
                if (!available->as_bool() && !gpu_time->is_null())
                {
                    diagnostic = "GPU-unavailable runs must store null GPU pass timings.";
                    return false;
                }
            }

            const char* optional_gpu_metrics[] = {
                "dxgi_current_usage_bytes", "dxgi_budget_bytes"
            };
            for (const char* name : optional_gpu_metrics)
            {
                const json_value* item = field(document, name);
                if (item == nullptr ||
                    (!item->is_null() &&
                        (!item->is_number() || !std::isfinite(item->as_number()) ||
                            item->as_number() < 0.0 ||
                            std::floor(item->as_number()) != item->as_number())))
                {
                    diagnostic = std::string("Run Result ") + name +
                        " must be a non-negative integer or null.";
                    return false;
                }
            }
            const bool usage_available =
                !field(document, "dxgi_current_usage_bytes")->is_null();
            const bool budget_available =
                !field(document, "dxgi_budget_bytes")->is_null();
            if (usage_available != budget_available)
            {
                diagnostic = "DXGI current usage and budget must both be available or both be null.";
                return false;
            }

            diagnostic.clear();
            return true;
        }
    }

    scenario_validation_result validate_scenario(const serialization::json_value& document)
    {
        scenario_validation_result result;
        result.document = document;
        result.success = validate_scenario_document(result.document, result.diagnostic);
        return result;
    }

    scenario_validation_result parse_scenario(const std::string& text)
    {
        scenario_validation_result result;
        const serialization::json_parse_result parsed = serialization::parse_json(text);
        if (!parsed.success)
        {
            result.diagnostic = parsed.error_message;
            return result;
        }
        return validate_scenario(parsed.value);
    }

    scenario_validation_result load_scenario(const std::filesystem::path& path)
    {
        std::string contents;
        scenario_validation_result result;
        if (!serialization::read_text_file(path, contents, result.diagnostic))
        {
            return result;
        }
        return parse_scenario(contents);
    }

    bool save_scenario(
        const std::filesystem::path& path,
        const serialization::json_value& document,
        std::string& diagnostic)
    {
        const scenario_validation_result validated = validate_scenario(document);
        if (!validated.success)
        {
            diagnostic = validated.diagnostic;
            return false;
        }
        const serialization::json_write_result serialized =
            serialization::write_json(validated.document);
        if (!serialized.success)
        {
            diagnostic = serialized.error_message;
            return false;
        }
        return serialization::write_text_file(path, serialized.text, diagnostic);
    }

    run_result_validation_result validate_run_result(const serialization::json_value& document)
    {
        run_result_validation_result result;
        result.document = document;
        result.success = validate_run_result_document(result.document, result.diagnostic);
        return result;
    }

    run_result_validation_result parse_run_result(const std::string& text)
    {
        run_result_validation_result result;
        const serialization::json_parse_result parsed = serialization::parse_json(text);
        if (!parsed.success)
        {
            result.diagnostic = parsed.error_message;
            return result;
        }
        return validate_run_result(parsed.value);
    }

    run_result_validation_result load_run_result(const std::filesystem::path& path)
    {
        std::string contents;
        run_result_validation_result result;
        if (!serialization::read_text_file(path, contents, result.diagnostic))
        {
            return result;
        }
        return parse_run_result(contents);
    }

    bool save_run_result(
        const std::filesystem::path& path,
        const serialization::json_value& document,
        std::string& diagnostic)
    {
        const run_result_validation_result validated = validate_run_result(document);
        if (!validated.success)
        {
            diagnostic = validated.diagnostic;
            return false;
        }
        const serialization::json_write_result serialized =
            serialization::write_json(validated.document);
        if (!serialized.success)
        {
            diagnostic = serialized.error_message;
            return false;
        }
        return serialization::write_text_file(path, serialized.text, diagnostic);
    }

    bool run_results_comparable(
        const serialization::json_value& left,
        const serialization::json_value& right)
    {
        const run_result_validation_result validated_left = validate_run_result(left);
        const run_result_validation_result validated_right = validate_run_result(right);
        if (!validated_left.success || !validated_right.success)
        {
            return false;
        }

        return field(left, "scenario_hash")->as_string() ==
                field(right, "scenario_hash")->as_string() &&
            field(left, "asset_id")->as_string() ==
                field(right, "asset_id")->as_string() &&
            field(left, "cache_version")->as_number() ==
                field(right, "cache_version")->as_number();
    }
}
