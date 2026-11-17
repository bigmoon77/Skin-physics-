#include "skin_physics/physics/ftetwild_runner.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include <locale>

#include "skin_physics/physics/msh_reader.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace skin_physics::physics
{
    namespace
    {
        void add_diagnostic(
            tetrahedralizer_result& result,
            tetrahedralizer_error code,
            const std::string& message,
            const std::filesystem::path& retained_directory = {})
        {
            tetrahedralizer_diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.message = message;
            diagnostic.retained_directory = retained_directory;
            result.diagnostics.push_back(std::move(diagnostic));
        }

#ifdef _WIN32
        std::wstring quote_windows_argument(const std::wstring& value)
        {
            std::wstring result;
            result.push_back(L'"');

            std::size_t backslash_count = 0;
            for (wchar_t character : value)
            {
                if (character == L'\\')
                {
                    ++backslash_count;
                }
                else if (character == L'"')
                {
                    result.append(backslash_count * 2 + 1, L'\\');
                    result.push_back(L'"');
                    backslash_count = 0;
                }
                else
                {
                    result.append(backslash_count, L'\\');
                    backslash_count = 0;
                    result.push_back(character);
                }
            }

            result.append(backslash_count * 2, L'\\');
            result.push_back(L'"');
            return result;
        }

        bool write_surface_obj(const asset::mesh& surface_mesh, const std::filesystem::path& path)
        {
            std::ofstream file(path);
            file.imbue(std::locale::classic());
            if (!file)
            {
                return false;
            }

            file << std::setprecision(std::numeric_limits<float>::max_digits10);
            file << "o SkinPhysicsSurface\n";
            for (const asset::vertex& vertex : surface_mesh.vertices)
            {
                if (!std::isfinite(vertex.position.x) ||
                    !std::isfinite(vertex.position.y) ||
                    !std::isfinite(vertex.position.z))
                {
                    return false;
                }
                file << "v " << vertex.position.x << " "
                     << vertex.position.y << " " << vertex.position.z << "\n";
            }

            if (surface_mesh.indices.size() % 3 != 0)
            {
                return false;
            }

            for (std::size_t index = 0; index < surface_mesh.indices.size(); index += 3)
            {
                const uint32_t a = surface_mesh.indices[index];
                const uint32_t b = surface_mesh.indices[index + 1];
                const uint32_t c = surface_mesh.indices[index + 2];
                if (a >= surface_mesh.vertices.size() ||
                    b >= surface_mesh.vertices.size() ||
                    c >= surface_mesh.vertices.size())
                {
                    return false;
                }
                file << "f " << (a + 1) << " " << (b + 1) << " " << (c + 1) << "\n";
            }

            file.flush();
            return static_cast<bool>(file);
        }

        bool run_process(
            const std::filesystem::path& executable_path,
            const std::wstring& arguments,
            uint32_t timeout_milliseconds,
            uint32_t& exit_code,
            bool& timed_out)
        {
            timed_out = false;
            std::wstring command_line = quote_windows_argument(executable_path.wstring());
            if (!arguments.empty())
            {
                command_line.push_back(L' ');
                command_line += arguments;
            }

            STARTUPINFOW startup_info = {};
            startup_info.cb = sizeof(startup_info);
            PROCESS_INFORMATION process_info = {};

            std::vector<wchar_t> mutable_command_line(
                command_line.begin(), command_line.end());
            mutable_command_line.push_back(L'\0');

            const std::filesystem::path working_directory = executable_path.parent_path();
            const BOOL started = CreateProcessW(
                executable_path.c_str(),
                mutable_command_line.data(),
                nullptr,
                nullptr,
                FALSE,
                CREATE_NO_WINDOW,
                nullptr,
                working_directory.empty() ? nullptr : working_directory.c_str(),
                &startup_info,
                &process_info);

            if (started == FALSE)
            {
                return false;
            }

            const DWORD wait_result = WaitForSingleObject(process_info.hProcess, timeout_milliseconds);
            if (wait_result == WAIT_TIMEOUT)
            {
                timed_out = true;
                TerminateProcess(process_info.hProcess, 1);
                WaitForSingleObject(process_info.hProcess, INFINITE);
                CloseHandle(process_info.hThread);
                CloseHandle(process_info.hProcess);
                return false;
            }

            if (wait_result != WAIT_OBJECT_0)
            {
                TerminateProcess(process_info.hProcess, 1);
                WaitForSingleObject(process_info.hProcess, INFINITE);
                CloseHandle(process_info.hThread);
                CloseHandle(process_info.hProcess);
                return false;
            }

            DWORD process_exit_code = 0;
            const BOOL exit_code_read = GetExitCodeProcess(process_info.hProcess, &process_exit_code);
            CloseHandle(process_info.hThread);
            CloseHandle(process_info.hProcess);

            if (exit_code_read == FALSE)
            {
                return false;
            }

            exit_code = static_cast<uint32_t>(process_exit_code);
            return true;
        }
#endif
    }

    ftetwild_resolution_result resolve_ftetwild_settings(
        const asset::mesh& surface_mesh,
        const ftetwild_settings& settings)
    {
        ftetwild_resolution_result result;
        const auto fail = [&result](tetrahedralizer_error error, const std::string& message)
        {
            tetrahedralizer_diagnostic diagnostic;
            diagnostic.code = error;
            diagnostic.message = message;
            result.diagnostics.push_back(std::move(diagnostic));
        };

        if (surface_mesh.vertices.empty() || surface_mesh.indices.empty() ||
            surface_mesh.indices.size() % 3 != 0)
        {
            fail(tetrahedralizer_error::invalid_input_mesh,
                "Resolution estimation requires a non-empty triangle-list surface mesh.");
            return result;
        }
        if (settings.maximum_tetrahedra == 0 ||
            settings.timeout_milliseconds == 0 ||
            !std::isfinite(settings.surface_approximation_tolerance) ||
            settings.surface_approximation_tolerance <= 0.0f ||
            !std::isfinite(settings.tetra_quality_threshold) ||
            settings.tetra_quality_threshold < 0.0f ||
            settings.generator_id.empty() || settings.generator_version.empty())
        {
            fail(tetrahedralizer_error::invalid_settings,
                "Maximum count, timeout, approximation tolerance, quality threshold and generator identity are invalid.");
            return result;
        }

        const bool edge_length_selected =
            std::isfinite(settings.target_edge_length) && settings.target_edge_length > 0.0f;
        const bool count_selected = settings.target_tetrahedron_count > 0;
        const bool count_mode_edge_is_zero = settings.target_edge_length == 0.0f;
        if ((edge_length_selected && count_selected) ||
            (!edge_length_selected && !(count_selected && count_mode_edge_is_zero)) ||
            (count_selected && settings.target_tetrahedron_count > settings.maximum_tetrahedra))
        {
            fail(tetrahedralizer_error::invalid_settings,
                "Select exactly one resolution mode. target_tetrahedron_count must not exceed maximum_tetrahedra.");
            return result;
        }

        asset::vec3 minimum = surface_mesh.vertices.front().position;
        asset::vec3 maximum = minimum;
        for (const asset::vertex& vertex : surface_mesh.vertices)
        {
            const asset::vec3& position = vertex.position;
            if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
                !std::isfinite(position.z))
            {
                fail(tetrahedralizer_error::invalid_input_mesh,
                    "Surface mesh contains a non-finite vertex position.");
                return result;
            }
            minimum.x = std::min(minimum.x, position.x);
            minimum.y = std::min(minimum.y, position.y);
            minimum.z = std::min(minimum.z, position.z);
            maximum.x = std::max(maximum.x, position.x);
            maximum.y = std::max(maximum.y, position.y);
            maximum.z = std::max(maximum.z, position.z);
        }
        for (uint32_t index : surface_mesh.indices)
        {
            if (index >= surface_mesh.vertices.size())
            {
                fail(tetrahedralizer_error::invalid_input_mesh,
                    "Surface mesh contains an out-of-range triangle index.");
                return result;
            }
        }

        const double dx = static_cast<double>(maximum.x) - minimum.x;
        const double dy = static_cast<double>(maximum.y) - minimum.y;
        const double dz = static_cast<double>(maximum.z) - minimum.z;
        const double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!std::isfinite(diagonal) || diagonal <= 1.0e-12)
        {
            fail(tetrahedralizer_error::invalid_input_mesh,
                "Surface mesh bounding-box diagonal must be positive and finite.");
            return result;
        }

        result.value.surface_bbox_diagonal = static_cast<float>(diagonal);
        result.value.maximum_tetrahedra = settings.maximum_tetrahedra;
        if (count_selected)
        {
            result.value.resolution_mode = ftetwild_resolution_mode::target_tetrahedron_count;
            result.value.requested_tetrahedron_count = settings.target_tetrahedron_count;
            result.value.edge_length_is_estimated = true;

            // fTetWild exposes target edge length, not a strict element-count switch.
            // This deterministic characteristic spacing is verified against the actual output.
            const double edge_length = diagonal /
                std::cbrt(static_cast<double>(settings.target_tetrahedron_count));
            if (!std::isfinite(edge_length) || edge_length <= 0.0 ||
                edge_length > std::numeric_limits<float>::max())
            {
                fail(tetrahedralizer_error::invalid_settings,
                    "Could not convert the requested tetrahedron count into a finite target edge length.");
                result.value = {};
                return result;
            }
            result.value.target_edge_length = static_cast<float>(edge_length);
        }
        else
        {
            result.value.resolution_mode = ftetwild_resolution_mode::target_edge_length;
            result.value.target_edge_length = settings.target_edge_length;
            result.value.edge_length_is_estimated = false;
        }

        result.success = true;
        return result;
    }

    std::string make_ftetwild_cache_options(
        const ftetwild_settings& settings,
        const resolved_ftetwild_settings& resolved)
    {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::setprecision(std::numeric_limits<float>::max_digits10);
        stream << "resolution_mode="
            << (resolved.resolution_mode == ftetwild_resolution_mode::target_edge_length
                ? "target_edge_length" : "target_tetrahedron_count")
            << ";resolved_target_edge_length=" << resolved.target_edge_length
            << ";requested_target_tetrahedron_count=" << resolved.requested_tetrahedron_count
            << ";maximum_tetrahedra=" << resolved.maximum_tetrahedra
            << ";surface_bbox_diagonal=" << resolved.surface_bbox_diagonal
            << ";edge_length_is_estimated=" << (resolved.edge_length_is_estimated ? 1 : 0)
            << ";surface_approximation_tolerance=" << settings.surface_approximation_tolerance
            << ";tetra_quality_threshold=" << settings.tetra_quality_threshold
            << ";resolution_estimator=bbox_diagonal_over_cuberoot_v1";
        return stream.str();
    }

    bool apply_ftetwild_cache_identity(
        const ftetwild_settings& settings,
        const resolved_ftetwild_settings& resolved,
        physics_asset_cache_identity& identity)
    {
        if (settings.generator_id.empty() || settings.generator_version.empty() ||
            !std::isfinite(resolved.target_edge_length) || resolved.target_edge_length <= 0.0f ||
            resolved.maximum_tetrahedra == 0)
        {
            return false;
        }
        identity.generator_id = settings.generator_id;
        identity.generator_version = settings.generator_version;
        identity.generator_options = make_ftetwild_cache_options(settings, resolved);
        return true;
    }

    bool measure_tetrahedralization_output(
        const raw_tet_mesh& mesh,
        const resolved_ftetwild_settings& resolved,
        tetrahedralization_report& report,
        std::string& error_message)
    {
        if (mesh.positions.empty() || mesh.tetrahedra.empty() ||
            resolved.maximum_tetrahedra == 0 ||
            !std::isfinite(resolved.target_edge_length) ||
            resolved.target_edge_length <= 0.0f)
        {
            error_message = "Tetrahedralization output or resolved generation settings are empty.";
            return false;
        }

        for (const asset::vec3& position : mesh.positions)
        {
            if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
                !std::isfinite(position.z))
            {
                error_message = "Tetrahedralization output contains a non-finite vertex.";
                return false;
            }
        }

        double edge_length_sum = 0.0;
        double minimum_edge = std::numeric_limits<double>::max();
        double maximum_edge = 0.0;
        uint64_t edge_count = 0;
        static constexpr std::array<std::array<uint32_t, 2>, 6> edge_indices = {{
            {{ 0, 1 }}, {{ 0, 2 }}, {{ 0, 3 }},
            {{ 1, 2 }}, {{ 1, 3 }}, {{ 2, 3 }}
        }};

        for (const std::array<uint32_t, 4>& tetrahedron : mesh.tetrahedra)
        {
            for (uint32_t local_index = 0; local_index < 4; ++local_index)
            {
                if (tetrahedron[local_index] >= mesh.positions.size())
                {
                    error_message = "Tetrahedralization output contains an out-of-range vertex index.";
                    return false;
                }
                for (uint32_t previous_index = 0; previous_index < local_index; ++previous_index)
                {
                    if (tetrahedron[local_index] == tetrahedron[previous_index])
                    {
                        error_message = "Tetrahedralization output contains a repeated tetrahedron vertex.";
                        return false;
                    }
                }
            }

            const asset::vec3& p0 = mesh.positions[tetrahedron[0]];
            const asset::vec3& p1 = mesh.positions[tetrahedron[1]];
            const asset::vec3& p2 = mesh.positions[tetrahedron[2]];
            const asset::vec3& p3 = mesh.positions[tetrahedron[3]];
            const double e1x = static_cast<double>(p1.x) - p0.x;
            const double e1y = static_cast<double>(p1.y) - p0.y;
            const double e1z = static_cast<double>(p1.z) - p0.z;
            const double e2x = static_cast<double>(p2.x) - p0.x;
            const double e2y = static_cast<double>(p2.y) - p0.y;
            const double e2z = static_cast<double>(p2.z) - p0.z;
            const double e3x = static_cast<double>(p3.x) - p0.x;
            const double e3y = static_cast<double>(p3.y) - p0.y;
            const double e3z = static_cast<double>(p3.z) - p0.z;
            const double volume_six =
                e1x * (e2y * e3z - e2z * e3y) -
                e1y * (e2x * e3z - e2z * e3x) +
                e1z * (e2x * e3y - e2y * e3x);
            if (!std::isfinite(volume_six) || std::abs(volume_six) <= 1.0e-18)
            {
                error_message = "Tetrahedralization output contains a degenerate tetrahedron.";
                return false;
            }

            for (const auto& edge : edge_indices)
            {
                const asset::vec3& a = mesh.positions[tetrahedron[edge[0]]];
                const asset::vec3& b = mesh.positions[tetrahedron[edge[1]]];
                const double dx = static_cast<double>(b.x) - a.x;
                const double dy = static_cast<double>(b.y) - a.y;
                const double dz = static_cast<double>(b.z) - a.z;
                const double edge_length = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (!std::isfinite(edge_length) || edge_length <= 0.0)
                {
                    error_message = "Tetrahedralization output contains an invalid edge length.";
                    return false;
                }
                edge_length_sum += edge_length;
                minimum_edge = std::min(minimum_edge, edge_length);
                maximum_edge = std::max(maximum_edge, edge_length);
                ++edge_count;
            }
        }

        report.resolution = resolved;
        report.actual_vertex_count = mesh.positions.size();
        report.actual_tetrahedron_count = mesh.tetrahedra.size();
        report.minimum_edge_length = static_cast<float>(minimum_edge);
        report.average_edge_length = static_cast<float>(edge_length_sum /
            static_cast<double>(edge_count));
        report.maximum_edge_length = static_cast<float>(maximum_edge);
        report.has_edge_statistics = edge_count > 0;
        if (resolved.resolution_mode == ftetwild_resolution_mode::target_tetrahedron_count &&
            resolved.requested_tetrahedron_count > 0)
        {
            report.target_tetrahedron_count_relative_error =
                std::abs(static_cast<float>(mesh.tetrahedra.size()) -
                    static_cast<float>(resolved.requested_tetrahedron_count)) /
                static_cast<float>(resolved.requested_tetrahedron_count);
        }
        return true;
    }

    tetrahedralizer_result run_ftetwild(
        const asset::mesh& surface_mesh,
        const ftetwild_settings& settings)
    {
        tetrahedralizer_result result;

#ifndef _WIN32
        (void)surface_mesh;
        (void)settings;
        add_diagnostic(result, tetrahedralizer_error::unsupported_platform,
            "The fTetWild process adapter is currently implemented for Windows only.");
        return result;
#else
        if (settings.executable_path.empty() ||
            !std::filesystem::is_regular_file(settings.executable_path) ||
            !settings.executable_path.is_absolute())
        {
            add_diagnostic(result, tetrahedralizer_error::executable_not_found,
                "Set ftetwild_settings::executable_path to an absolute path to an installed FloatTetwild_bin.exe.");
            return result;
        }

        const ftetwild_resolution_result resolution =
            resolve_ftetwild_settings(surface_mesh, settings);
        if (!resolution.success)
        {
            if (!resolution.diagnostics.empty())
            {
                add_diagnostic(result, resolution.diagnostics[0].code,
                    resolution.diagnostics[0].message);
            }
            return result;
        }
        result.report.resolution = resolution.value;

        static std::atomic<uint64_t> directory_sequence{ 0 };
        std::filesystem::path temporary_directory;
        try
        {
            const std::filesystem::path base_directory = std::filesystem::temp_directory_path();
            const std::wstring directory_name =
                L"SkinPhysics_fTetWild_" +
                std::to_wstring(GetCurrentProcessId()) + L"_" +
                std::to_wstring(directory_sequence.fetch_add(1));
            temporary_directory = base_directory / directory_name;
            std::filesystem::create_directories(temporary_directory);
        }
        catch (const std::exception& exception)
        {
            add_diagnostic(result, tetrahedralizer_error::temporary_directory_error, exception.what());
            return result;
        }

        const std::filesystem::path input_path = temporary_directory / L"surface.obj";
        const std::filesystem::path output_path = temporary_directory / L"volume.msh";
        const std::filesystem::path log_path = temporary_directory / L"ftetwild.log";

        if (!write_surface_obj(surface_mesh, input_path))
        {
            add_diagnostic(result, tetrahedralizer_error::input_write_error,
                "Could not write the temporary OBJ surface mesh.",
                temporary_directory);
            return result;
        }

        std::wostringstream argument_stream;
        argument_stream.imbue(std::locale::classic());
        argument_stream << L"-i " << quote_windows_argument(input_path.wstring())
            << L" -o " << quote_windows_argument(output_path.wstring())
            << L" --la " << std::setprecision(9) << resolution.value.target_edge_length
            << L" --epsr " << settings.surface_approximation_tolerance
            << L" --no-binary --is-quiet --log "
            << quote_windows_argument(log_path.wstring());

        uint32_t exit_code = 0;
        bool timed_out = false;
        const bool process_completed = run_process(
            settings.executable_path,
            argument_stream.str(),
            settings.timeout_milliseconds,
            exit_code,
            timed_out);

        if (!process_completed)
        {
            const std::filesystem::path retained_directory =
                settings.keep_temporary_files_on_failure ? temporary_directory : std::filesystem::path{};
            add_diagnostic(
                result,
                timed_out ? tetrahedralizer_error::process_timeout : tetrahedralizer_error::process_start_error,
                timed_out
                    ? "fTetWild did not complete before the configured timeout."
                    : "Could not start fTetWild or query its process exit code.",
                retained_directory);
            if (!settings.keep_temporary_files_on_failure)
            {
                std::error_code ignored_error;
                std::filesystem::remove_all(temporary_directory, ignored_error);
            }
            return result;
        }

        if (exit_code != 0)
        {
            const std::filesystem::path retained_directory =
                settings.keep_temporary_files_on_failure ? temporary_directory : std::filesystem::path{};
            add_diagnostic(result, tetrahedralizer_error::process_failed,
                "fTetWild returned a non-zero exit code: " + std::to_string(exit_code),
                retained_directory);
            if (!settings.keep_temporary_files_on_failure)
            {
                std::error_code ignored_error;
                std::filesystem::remove_all(temporary_directory, ignored_error);
            }
            return result;
        }

        const msh_read_result read_result = read_gmsh_2_2_ascii(output_path);
        if (!read_result.success)
        {
            std::string message = "Could not read fTetWild output.";
            if (!read_result.diagnostics.empty())
            {
                message += " " + read_result.diagnostics.front().message;
            }
            add_diagnostic(result, tetrahedralizer_error::output_read_error,
                message, temporary_directory);
            return result;
        }

        if (read_result.mesh.tetrahedra.size() > settings.maximum_tetrahedra)
        {
            const std::filesystem::path retained_directory =
                settings.keep_temporary_files_on_failure ? temporary_directory : std::filesystem::path{};
            add_diagnostic(result, tetrahedralizer_error::capacity_exceeded,
                "fTetWild output exceeds maximum_tetrahedra; the generated mesh was rejected.",
                retained_directory);
            if (!settings.keep_temporary_files_on_failure)
            {
                std::error_code ignored_error;
                std::filesystem::remove_all(temporary_directory, ignored_error);
            }
            return result;
        }

        std::string output_error;
        if (!measure_tetrahedralization_output(
            read_result.mesh, resolution.value, result.report, output_error))
        {
            add_diagnostic(result, tetrahedralizer_error::invalid_output_mesh,
                output_error, temporary_directory);
            return result;
        }

        result.mesh = read_result.mesh;
        result.success = true;

        std::error_code ignored_error;
        std::filesystem::remove_all(temporary_directory, ignored_error);
        return result;
#endif
    }
}
