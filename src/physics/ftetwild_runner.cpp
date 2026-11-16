#include "skin_physics/physics/ftetwild_runner.hpp"

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

        if (!std::isfinite(settings.target_edge_length) || settings.target_edge_length <= 0.0f ||
            !std::isfinite(settings.relative_envelope) || settings.relative_envelope <= 0.0f ||
            settings.timeout_milliseconds == 0)
        {
            add_diagnostic(result, tetrahedralizer_error::invalid_settings,
                "fTetWild target edge length, envelope and timeout must be positive and finite.");
            return result;
        }

        if (surface_mesh.vertices.empty() || surface_mesh.indices.empty() ||
            surface_mesh.indices.size() % 3 != 0)
        {
            add_diagnostic(result, tetrahedralizer_error::invalid_input_mesh,
                "The input surface mesh must contain triangle geometry.");
            return result;
        }

        for (uint32_t index : surface_mesh.indices)
        {
            if (index >= surface_mesh.vertices.size())
            {
                add_diagnostic(result, tetrahedralizer_error::invalid_input_mesh,
                    "The surface mesh contains an out-of-range vertex index.");
                return result;
            }
        }

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
            << L" --la " << std::setprecision(9) << settings.target_edge_length
            << L" --epsr " << settings.relative_envelope
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

        result.mesh = read_result.mesh;
        result.success = true;

        std::error_code ignored_error;
        std::filesystem::remove_all(temporary_directory, ignored_error);
        return result;
#endif
    }
}
