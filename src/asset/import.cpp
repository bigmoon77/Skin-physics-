#include "skin_physics/asset/importer.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <string>

#include "importer_internal.hpp"

namespace skin_physics::asset
{
    import_result import_model(const std::filesystem::path& path, const import_options& options)
    {
        const std::string extension = path.extension().string();
        std::string normalized_extension = extension;
        std::transform(
            normalized_extension.begin(),
            normalized_extension.end(),
            normalized_extension.begin(),
            [](unsigned char value) { return static_cast<char>(std::tolower(value)); });

        import_result result;

        try
        {
            if (normalized_extension == ".obj")
            {
                result = detail::import_obj(path, options);
            }
            else if (normalized_extension == ".gltf" || normalized_extension == ".glb")
            {
                result = detail::import_gltf(path, options);
            }
            else
            {
                detail::append_diagnostic(
                    result,
                    diagnostic_severity::error,
                    diagnostic_code::unsupported_format,
                    "Supported input formats are .obj, .gltf and .glb.");
                return result;
            }
        }
        catch (const std::bad_alloc&)
        {
            detail::append_diagnostic(
                result,
                diagnostic_severity::error,
                diagnostic_code::out_of_memory,
                "Memory allocation failed while importing the asset.");
            return result;
        }
        catch (const std::exception& exception)
        {
            detail::append_diagnostic(
                result,
                diagnostic_severity::error,
                diagnostic_code::internal_error,
                std::string("Unexpected importer error: ") + exception.what());
            return result;
        }
        catch (...)
        {
            detail::append_diagnostic(
                result,
                diagnostic_severity::error,
                diagnostic_code::internal_error,
                "Unknown importer error.");
            return result;
        }

        if (result.success)
        {
            std::vector<diagnostic> validation_diagnostics = validate_model(result.asset);
            for (const diagnostic& item : validation_diagnostics)
            {
                result.diagnostics.push_back(item);
                if (item.severity == diagnostic_severity::error)
                {
                    result.success = false;
                }
            }
        }

        return result;
    }
}
