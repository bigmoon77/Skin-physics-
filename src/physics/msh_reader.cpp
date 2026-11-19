#include "skin_physics/physics/msh_reader.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace skin_physics::physics
{
    namespace
    {
        void add_error(msh_read_result& result, uint32_t line, const std::string& message)
        {
            msh_read_diagnostic diagnostic;
            diagnostic.line = line;
            diagnostic.message = message;
            result.diagnostics.push_back(std::move(diagnostic));
        }

        bool read_count(std::ifstream& file, uint64_t& count, uint32_t& line_number)
        {
            std::string line;
            if (!std::getline(file, line))
            {
                return false;
            }
            ++line_number;
            std::istringstream stream(line);
            if (!(stream >> count))
            {
                return false;
            }
            stream >> std::ws;
            return stream.eof();
        }

        bool expect_end_section(
            std::ifstream& file,
            const std::string& section_name,
            uint32_t& line_number)
        {
            std::string line;
            if (!std::getline(file, line))
            {
                return false;
            }
            ++line_number;
            return line == "$End" + section_name;
        }
    }

    msh_read_result read_gmsh_2_2_ascii(const std::filesystem::path& path)
    {
        msh_read_result result;
        std::ifstream file(path);
        file.imbue(std::locale::classic());
        if (!file)
        {
            add_error(result, 0, "Could not open the Gmsh mesh file: " + path.string());
            return result;
        }

        std::unordered_map<uint64_t, uint32_t> node_id_to_index;
        std::string line;
        uint32_t line_number = 0;
        bool has_mesh_format = false;
        bool has_nodes = false;
        bool has_elements = false;

        while (std::getline(file, line))
        {
            ++line_number;
            if (line == "$MeshFormat")
            {
                std::string format_line;
                if (!std::getline(file, format_line))
                {
                    add_error(result, line_number, "Missing Gmsh mesh format header.");
                    return result;
                }
                ++line_number;

                double version = 0.0;
                int file_type = -1;
                int data_size = 0;
                std::istringstream format_stream(format_line);
                format_stream.imbue(std::locale::classic());
                if (!(format_stream >> version >> file_type >> data_size) ||
                    std::abs(version - 2.2) > 1.0e-6 || file_type != 0 ||
                    (data_size != 4 && data_size != 8))
                {
                    add_error(result, line_number,
                        "Only ASCII Gmsh MSH 2.2 files with 4- or 8-byte floating point data are supported.");
                    return result;
                }

                if (!expect_end_section(file, "MeshFormat", line_number))
                {
                    add_error(result, line_number, "Missing $EndMeshFormat.");
                    return result;
                }
                has_mesh_format = true;
            }
            else if (line == "$Nodes")
            {
                uint64_t node_count = 0;
                if (!read_count(file, node_count, line_number) ||
                    node_count > std::numeric_limits<uint32_t>::max())
                {
                    add_error(result, line_number, "Invalid or excessive Gmsh node count.");
                    return result;
                }

                result.mesh.positions.resize(static_cast<std::size_t>(node_count));
                node_id_to_index.reserve(static_cast<std::size_t>(node_count));

                for (uint64_t node_number = 0; node_number < node_count; ++node_number)
                {
                    if (!std::getline(file, line))
                    {
                        add_error(result, line_number, "Unexpected end of file in Gmsh node section.");
                        return result;
                    }
                    ++line_number;

                    uint64_t node_id = 0;
                    asset::vec3 position;
                    std::istringstream node_stream(line);
                    node_stream.imbue(std::locale::classic());
                    if (!(node_stream >> node_id >> position.x >> position.y >> position.z) ||
                        !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z))
                    {
                        add_error(result, line_number, "Invalid Gmsh node record.");
                        return result;
                    }

                    const uint32_t vertex_index = static_cast<uint32_t>(node_number);
                    const auto insertion = node_id_to_index.emplace(node_id, vertex_index);
                    if (!insertion.second)
                    {
                        add_error(result, line_number, "Duplicate Gmsh node identifier.");
                        return result;
                    }
                    result.mesh.positions[vertex_index] = position;
                }

                if (!expect_end_section(file, "Nodes", line_number))
                {
                    add_error(result, line_number, "Missing $EndNodes.");
                    return result;
                }
                has_nodes = true;
            }
            else if (line == "$Elements")
            {
                uint64_t element_count = 0;
                if (!read_count(file, element_count, line_number) ||
                    element_count > std::numeric_limits<uint32_t>::max())
                {
                    add_error(result, line_number, "Invalid or excessive Gmsh element count.");
                    return result;
                }

                for (uint64_t element_number = 0; element_number < element_count; ++element_number)
                {
                    if (!std::getline(file, line))
                    {
                        add_error(result, line_number, "Unexpected end of file in Gmsh element section.");
                        return result;
                    }
                    ++line_number;

                    std::istringstream element_stream(line);
                    element_stream.imbue(std::locale::classic());
                    uint64_t element_id = 0;
                    uint32_t element_type = 0;
                    uint32_t tag_count = 0;
                    if (!(element_stream >> element_id >> element_type >> tag_count))
                    {
                        add_error(result, line_number, "Invalid Gmsh element record.");
                        return result;
                    }

                    for (uint32_t tag_index = 0; tag_index < tag_count; ++tag_index)
                    {
                        int32_t ignored_tag = 0;
                        if (!(element_stream >> ignored_tag))
                        {
                            add_error(result, line_number, "Invalid Gmsh element tag list.");
                            return result;
                        }
                    }

                    if (element_type != 4)
                    {
                        continue;
                    }

                    std::array<uint32_t, 4> tetrahedron = {};
                    for (uint32_t local_index = 0; local_index < 4; ++local_index)
                    {
                        uint64_t node_id = 0;
                        if (!(element_stream >> node_id))
                        {
                            add_error(result, line_number, "Gmsh tetrahedron does not contain four node identifiers.");
                            return result;
                        }

                        const auto node_iterator = node_id_to_index.find(node_id);
                        if (node_iterator == node_id_to_index.end())
                        {
                            add_error(result, line_number, "Gmsh tetrahedron references an unknown node identifier.");
                            return result;
                        }
                        tetrahedron[local_index] = node_iterator->second;
                    }

                    result.mesh.tetrahedra.push_back(tetrahedron);
                }

                if (!expect_end_section(file, "Elements", line_number))
                {
                    add_error(result, line_number, "Missing $EndElements.");
                    return result;
                }
                has_elements = true;
            }
        }

        if (file.bad())
        {
            add_error(result, line_number, "I/O error while reading Gmsh mesh file.");
            return result;
        }

        if (!has_mesh_format || !has_nodes || !has_elements)
        {
            add_error(result, line_number, "Gmsh file is missing MeshFormat, Nodes, or Elements section.");
            return result;
        }

        if (result.mesh.positions.empty() || result.mesh.tetrahedra.empty())
        {
            add_error(result, line_number, "Gmsh file contains no tetrahedral volume elements.");
            return result;
        }

        result.success = true;
        return result;
    }
}
