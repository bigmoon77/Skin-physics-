#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace test_support
{
    inline std::filesystem::path fixture_directory()
    {
        // MSVC /FC expands __FILE__ to the full source path, so tests work
        // regardless of the current working directory (IDE, CI or command line).
        return std::filesystem::path(__FILE__).parent_path() / "fixtures";
    }

    inline std::filesystem::path fixture_path(const std::string& relative_path)
    {
        const std::filesystem::path path = fixture_directory() / relative_path;
        if (!std::filesystem::exists(path))
        {
            throw std::runtime_error("Required golden fixture does not exist: " + path.string());
        }
        return path;
    }
}
