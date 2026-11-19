#include "skin_physics/physics/asset_cache.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

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
        constexpr std::array<uint8_t, 8> cache_magic = {
            'S', 'P', 'H', 'Y', 'S', 'C', 'A', '1'
        };
        constexpr uint32_t cache_file_version = 1;
        constexpr uint64_t fnv_offset_basis = 14695981039346656037ull;
        constexpr uint64_t fnv_prime = 1099511628211ull;
        constexpr uint32_t maximum_string_bytes = 1024u * 1024u;
        constexpr uint64_t maximum_element_count = 100'000'000ull;

        struct stream_writer
        {
            explicit stream_writer(std::ostream& output_stream)
                : stream(output_stream)
            {
            }

            bool write_bytes(const void* data, std::size_t size)
            {
                if (size == 0) return true;
                stream.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
                if (!stream) return false;

                const uint8_t* bytes = static_cast<const uint8_t*>(data);
                for (std::size_t index = 0; index < size; ++index)
                {
                    hash ^= bytes[index];
                    hash *= fnv_prime;
                }
                return true;
            }

            bool write_u8(uint8_t value)
            {
                return write_bytes(&value, sizeof(value));
            }

            bool write_u16(uint16_t value)
            {
                const std::array<uint8_t, 2> bytes = {
                    static_cast<uint8_t>(value & 0xffu),
                    static_cast<uint8_t>((value >> 8) & 0xffu)
                };
                return write_bytes(bytes.data(), bytes.size());
            }

            bool write_u32(uint32_t value)
            {
                const std::array<uint8_t, 4> bytes = {
                    static_cast<uint8_t>(value & 0xffu),
                    static_cast<uint8_t>((value >> 8) & 0xffu),
                    static_cast<uint8_t>((value >> 16) & 0xffu),
                    static_cast<uint8_t>((value >> 24) & 0xffu)
                };
                return write_bytes(bytes.data(), bytes.size());
            }

            bool write_u64(uint64_t value)
            {
                const std::array<uint8_t, 8> bytes = {
                    static_cast<uint8_t>(value & 0xffull),
                    static_cast<uint8_t>((value >> 8) & 0xffull),
                    static_cast<uint8_t>((value >> 16) & 0xffull),
                    static_cast<uint8_t>((value >> 24) & 0xffull),
                    static_cast<uint8_t>((value >> 32) & 0xffull),
                    static_cast<uint8_t>((value >> 40) & 0xffull),
                    static_cast<uint8_t>((value >> 48) & 0xffull),
                    static_cast<uint8_t>((value >> 56) & 0xffull)
                };
                return write_bytes(bytes.data(), bytes.size());
            }

            bool write_f32(float value)
            {
                uint32_t bits = 0;
                static_assert(sizeof(bits) == sizeof(value), "Unexpected float size.");
                std::memcpy(&bits, &value, sizeof(bits));
                return write_u32(bits);
            }

            bool write_string(const std::string& value)
            {
                if (value.size() > maximum_string_bytes ||
                    value.size() > std::numeric_limits<uint32_t>::max())
                {
                    return false;
                }
                return write_u32(static_cast<uint32_t>(value.size())) &&
                    write_bytes(value.data(), value.size());
            }

            std::ostream& stream;
            uint64_t hash = fnv_offset_basis;
        };

        struct stream_reader
        {
            explicit stream_reader(std::istream& input_stream)
                : stream(input_stream)
            {
            }

            bool read_bytes(void* destination, std::size_t size)
            {
                if (size == 0) return true;
                stream.read(static_cast<char*>(destination), static_cast<std::streamsize>(size));
                if (!stream) return false;

                const uint8_t* bytes = static_cast<const uint8_t*>(destination);
                for (std::size_t index = 0; index < size; ++index)
                {
                    hash ^= bytes[index];
                    hash *= fnv_prime;
                }
                return true;
            }

            bool read_u8(uint8_t& value)
            {
                return read_bytes(&value, sizeof(value));
            }

            bool read_u16(uint16_t& value)
            {
                std::array<uint8_t, 2> bytes = {};
                if (!read_bytes(bytes.data(), bytes.size())) return false;
                value = static_cast<uint16_t>(bytes[0]) |
                    static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8);
                return true;
            }

            bool read_u32(uint32_t& value)
            {
                std::array<uint8_t, 4> bytes = {};
                if (!read_bytes(bytes.data(), bytes.size())) return false;
                value = static_cast<uint32_t>(bytes[0]) |
                    (static_cast<uint32_t>(bytes[1]) << 8) |
                    (static_cast<uint32_t>(bytes[2]) << 16) |
                    (static_cast<uint32_t>(bytes[3]) << 24);
                return true;
            }

            bool read_u64(uint64_t& value)
            {
                std::array<uint8_t, 8> bytes = {};
                if (!read_bytes(bytes.data(), bytes.size())) return false;
                value = static_cast<uint64_t>(bytes[0]) |
                    (static_cast<uint64_t>(bytes[1]) << 8) |
                    (static_cast<uint64_t>(bytes[2]) << 16) |
                    (static_cast<uint64_t>(bytes[3]) << 24) |
                    (static_cast<uint64_t>(bytes[4]) << 32) |
                    (static_cast<uint64_t>(bytes[5]) << 40) |
                    (static_cast<uint64_t>(bytes[6]) << 48) |
                    (static_cast<uint64_t>(bytes[7]) << 56);
                return true;
            }

            bool read_f32(float& value)
            {
                uint32_t bits = 0;
                if (!read_u32(bits)) return false;
                std::memcpy(&value, &bits, sizeof(value));
                return true;
            }

            bool read_string(std::string& value)
            {
                uint32_t length = 0;
                if (!read_u32(length) || length > maximum_string_bytes)
                {
                    return false;
                }
                value.resize(length);
                return read_bytes(value.data(), length);
            }

            std::istream& stream;
            uint64_t hash = fnv_offset_basis;
        };

        void add_diagnostic(
            std::vector<physics_asset_cache_diagnostic>& diagnostics,
            physics_asset_cache_error code,
            const std::string& message)
        {
            physics_asset_cache_diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.message = message;
            diagnostics.push_back(std::move(diagnostic));
        }

        bool valid_identity(const physics_asset_cache_identity& identity)
        {
            return identity.schema_version != 0 &&
                !identity.source_asset_id.empty() &&
                !identity.source_asset_hash.empty() &&
                !identity.generator_id.empty() &&
                !identity.generator_version.empty() &&
                identity.source_asset_id.size() <= maximum_string_bytes &&
                identity.source_asset_hash.size() <= maximum_string_bytes &&
                identity.generator_id.size() <= maximum_string_bytes &&
                identity.generator_version.size() <= maximum_string_bytes &&
                identity.generator_options.size() <= maximum_string_bytes &&
                identity.coordinate_system_version != 0 &&
                identity.material_layout_version != 0;
        }

        bool equal_identity(
            const physics_asset_cache_identity& left,
            const physics_asset_cache_identity& right)
        {
            return left.schema_version == right.schema_version &&
                left.source_asset_id == right.source_asset_id &&
                left.source_asset_hash == right.source_asset_hash &&
                left.generator_id == right.generator_id &&
                left.generator_version == right.generator_version &&
                left.generator_options == right.generator_options &&
                left.coordinate_system_version == right.coordinate_system_version &&
                left.material_layout_version == right.material_layout_version;
        }

        bool finite_vec3(const asset::vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool validate_mesh(const physics_mesh& value, std::string& error)
        {
            const std::size_t vertex_count = value.rest_positions.size();
            const std::size_t tetrahedron_count = value.tetrahedra.size();

            if (vertex_count == 0 || tetrahedron_count == 0 || value.boundary_triangles.empty())
            {
                error = "Physics Mesh is empty or has no boundary surface.";
                return false;
            }
            if (value.mass.size() != vertex_count || value.inverse_mass.size() != vertex_count)
            {
                error = "Physics Mesh mass arrays do not match the vertex count.";
                return false;
            }
            if (value.vertex_tetrahedron_offsets.size() != vertex_count + 1 ||
                value.vertex_tetrahedron_offsets.empty() ||
                value.vertex_tetrahedron_offsets.front() != 0 ||
                value.vertex_tetrahedron_offsets.back() != value.vertex_tetrahedron_ids.size() ||
                value.vertex_tetrahedron_ids.size() != tetrahedron_count * 4ull)
            {
                error = "Physics Mesh vertex-to-tetrahedron CSR is invalid.";
                return false;
            }
            if (value.has_skin_weights && value.skin_weights.size() != vertex_count)
            {
                error = "Physics Mesh skin weight count does not match the vertex count.";
                return false;
            }
            if (!value.has_skin_weights && !value.skin_weights.empty())
            {
                error = "Physics Mesh contains skin weights while has_skin_weights is false.";
                return false;
            }

            for (std::size_t index = 0; index < vertex_count; ++index)
            {
                if (!finite_vec3(value.rest_positions[index]) ||
                    !std::isfinite(value.mass[index]) || value.mass[index] <= 0.0f ||
                    !std::isfinite(value.inverse_mass[index]) || value.inverse_mass[index] <= 0.0f)
                {
                    error = "Physics Mesh contains a non-finite position or invalid mass.";
                    return false;
                }
                if (value.vertex_tetrahedron_offsets[index] > value.vertex_tetrahedron_offsets[index + 1])
                {
                    error = "Physics Mesh CSR offsets are not monotonic.";
                    return false;
                }

                if (value.has_skin_weights)
                {
                    float weight_sum = 0.0f;
                    for (float weight : value.skin_weights[index].weights)
                    {
                        if (!std::isfinite(weight) || weight < 0.0f)
                        {
                            error = "Physics Mesh contains an invalid skin weight.";
                            return false;
                        }
                        weight_sum += weight;
                    }
                    if (!std::isfinite(weight_sum) || std::abs(weight_sum - 1.0f) > 1.0e-3f)
                    {
                        error = "Physics Mesh skin weights are not normalized.";
                        return false;
                    }
                }
            }

            for (uint32_t tetrahedron_index : value.vertex_tetrahedron_ids)
            {
                if (tetrahedron_index >= tetrahedron_count)
                {
                    error = "Physics Mesh CSR references an invalid tetrahedron.";
                    return false;
                }
            }

            for (const tetrahedron& tet : value.tetrahedra)
            {
                if (!std::isfinite(tet.rest_volume) || tet.rest_volume <= 0.0f ||
                    !std::isfinite(tet.quality) || !std::isfinite(tet.minimum_dihedral_degrees))
                {
                    error = "Physics Mesh contains invalid tetrahedron rest data.";
                    return false;
                }
                for (uint32_t vertex_index : tet.vertices)
                {
                    if (vertex_index >= vertex_count)
                    {
                        error = "Physics Mesh tetrahedron references an invalid vertex.";
                        return false;
                    }
                }
                for (float element : tet.rest_inverse)
                {
                    if (!std::isfinite(element))
                    {
                        error = "Physics Mesh rest inverse contains a non-finite value.";
                        return false;
                    }
                }
                for (const asset::vec3& gradient : tet.rest_shape_gradients)
                {
                    if (!finite_vec3(gradient))
                    {
                        error = "Physics Mesh rest shape gradient contains a non-finite value.";
                        return false;
                    }
                }
            }

            for (const boundary_triangle& triangle : value.boundary_triangles)
            {
                if (triangle.source_tetrahedron >= tetrahedron_count)
                {
                    error = "Physics boundary face references an invalid source tetrahedron.";
                    return false;
                }
                for (uint32_t vertex_index : triangle.vertices)
                {
                    if (vertex_index >= vertex_count)
                    {
                        error = "Physics boundary face references an invalid vertex.";
                        return false;
                    }
                }
            }
            return true;
        }

        bool write_identity(stream_writer& writer, const physics_asset_cache_identity& identity)
        {
            return writer.write_string(identity.source_asset_id) &&
                writer.write_string(identity.source_asset_hash) &&
                writer.write_string(identity.generator_id) &&
                writer.write_string(identity.generator_version) &&
                writer.write_string(identity.generator_options) &&
                writer.write_u32(identity.schema_version) &&
                writer.write_u32(identity.coordinate_system_version) &&
                writer.write_u32(identity.material_layout_version);
        }

        bool read_identity(stream_reader& reader, physics_asset_cache_identity& identity)
        {
            return reader.read_string(identity.source_asset_id) &&
                reader.read_string(identity.source_asset_hash) &&
                reader.read_string(identity.generator_id) &&
                reader.read_string(identity.generator_version) &&
                reader.read_string(identity.generator_options) &&
                reader.read_u32(identity.schema_version) &&
                reader.read_u32(identity.coordinate_system_version) &&
                reader.read_u32(identity.material_layout_version);
        }

        bool write_mesh(stream_writer& writer, const physics_mesh& value)
        {
            if (!writer.write_u64(value.rest_positions.size()) ||
                !writer.write_u64(value.tetrahedra.size()) ||
                !writer.write_u64(value.boundary_triangles.size()) ||
                !writer.write_u64(value.vertex_tetrahedron_offsets.size()) ||
                !writer.write_u64(value.vertex_tetrahedron_ids.size()) ||
                !writer.write_u8(value.has_skin_weights ? 1u : 0u) ||
                !writer.write_u64(value.skin_weights.size()))
            {
                return false;
            }

            for (const asset::vec3& position : value.rest_positions)
            {
                if (!writer.write_f32(position.x) || !writer.write_f32(position.y) || !writer.write_f32(position.z))
                    return false;
            }

            for (const tetrahedron& tet : value.tetrahedra)
            {
                for (uint32_t vertex_index : tet.vertices)
                    if (!writer.write_u32(vertex_index)) return false;
                for (float element : tet.rest_inverse)
                    if (!writer.write_f32(element)) return false;
                for (const asset::vec3& gradient : tet.rest_shape_gradients)
                {
                    if (!writer.write_f32(gradient.x) ||
                        !writer.write_f32(gradient.y) ||
                        !writer.write_f32(gradient.z))
                    {
                        return false;
                    }
                }
                if (!writer.write_f32(tet.rest_volume) ||
                    !writer.write_f32(tet.quality) ||
                    !writer.write_f32(tet.minimum_dihedral_degrees))
                {
                    return false;
                }
            }

            for (const boundary_triangle& triangle : value.boundary_triangles)
            {
                for (uint32_t vertex_index : triangle.vertices)
                    if (!writer.write_u32(vertex_index)) return false;
                if (!writer.write_u32(triangle.source_tetrahedron)) return false;
            }
            for (uint32_t offset : value.vertex_tetrahedron_offsets)
                if (!writer.write_u32(offset)) return false;
            for (uint32_t tetrahedron_index : value.vertex_tetrahedron_ids)
                if (!writer.write_u32(tetrahedron_index)) return false;
            for (float mass : value.mass)
                if (!writer.write_f32(mass)) return false;
            for (float inverse_mass : value.inverse_mass)
                if (!writer.write_f32(inverse_mass)) return false;

            for (const physics_skin_weight& weight_set : value.skin_weights)
            {
                for (uint16_t joint : weight_set.joints)
                    if (!writer.write_u16(joint)) return false;
                for (float weight : weight_set.weights)
                    if (!writer.write_f32(weight)) return false;
            }
            return true;
        }

        bool read_count(stream_reader& reader, uint64_t& count, uint64_t file_size)
        {
            if (!reader.read_u64(count)) return false;
            return count <= maximum_element_count && count <= file_size / sizeof(uint32_t);
        }

        bool read_mesh(stream_reader& reader, physics_mesh& value, uint64_t file_size)
        {
            uint64_t vertex_count = 0;
            uint64_t tetrahedron_count = 0;
            uint64_t boundary_count = 0;
            uint64_t offset_count = 0;
            uint64_t id_count = 0;
            uint64_t skin_weight_count = 0;
            uint8_t has_skin_weights = 0;

            if (!read_count(reader, vertex_count, file_size) ||
                !read_count(reader, tetrahedron_count, file_size) ||
                !read_count(reader, boundary_count, file_size) ||
                !read_count(reader, offset_count, file_size) ||
                !read_count(reader, id_count, file_size) ||
                !reader.read_u8(has_skin_weights) ||
                !read_count(reader, skin_weight_count, file_size))
            {
                return false;
            }

            if (vertex_count == 0 || tetrahedron_count == 0 || boundary_count == 0 ||
                vertex_count >= std::numeric_limits<uint32_t>::max() ||
                tetrahedron_count >= std::numeric_limits<uint32_t>::max() ||
                offset_count != vertex_count + 1 ||
                id_count != tetrahedron_count * 4 ||
                has_skin_weights > 1 ||
                (has_skin_weights != 0 && skin_weight_count != vertex_count) ||
                (has_skin_weights == 0 && skin_weight_count != 0))
            {
                return false;
            }

            value.rest_positions.resize(static_cast<std::size_t>(vertex_count));
            value.tetrahedra.resize(static_cast<std::size_t>(tetrahedron_count));
            value.boundary_triangles.resize(static_cast<std::size_t>(boundary_count));
            value.vertex_tetrahedron_offsets.resize(static_cast<std::size_t>(offset_count));
            value.vertex_tetrahedron_ids.resize(static_cast<std::size_t>(id_count));
            value.mass.resize(static_cast<std::size_t>(vertex_count));
            value.inverse_mass.resize(static_cast<std::size_t>(vertex_count));
            value.has_skin_weights = has_skin_weights != 0;
            value.skin_weights.resize(static_cast<std::size_t>(skin_weight_count));

            for (asset::vec3& position : value.rest_positions)
            {
                if (!reader.read_f32(position.x) ||
                    !reader.read_f32(position.y) ||
                    !reader.read_f32(position.z))
                {
                    return false;
                }
            }

            for (tetrahedron& tet : value.tetrahedra)
            {
                for (uint32_t& vertex_index : tet.vertices)
                    if (!reader.read_u32(vertex_index)) return false;
                for (float& element : tet.rest_inverse)
                    if (!reader.read_f32(element)) return false;
                for (asset::vec3& gradient : tet.rest_shape_gradients)
                {
                    if (!reader.read_f32(gradient.x) ||
                        !reader.read_f32(gradient.y) ||
                        !reader.read_f32(gradient.z))
                    {
                        return false;
                    }
                }
                if (!reader.read_f32(tet.rest_volume) ||
                    !reader.read_f32(tet.quality) ||
                    !reader.read_f32(tet.minimum_dihedral_degrees))
                {
                    return false;
                }
            }

            for (boundary_triangle& triangle : value.boundary_triangles)
            {
                for (uint32_t& vertex_index : triangle.vertices)
                    if (!reader.read_u32(vertex_index)) return false;
                if (!reader.read_u32(triangle.source_tetrahedron)) return false;
            }
            for (uint32_t& offset : value.vertex_tetrahedron_offsets)
                if (!reader.read_u32(offset)) return false;
            for (uint32_t& tetrahedron_index : value.vertex_tetrahedron_ids)
                if (!reader.read_u32(tetrahedron_index)) return false;
            for (float& mass : value.mass)
                if (!reader.read_f32(mass)) return false;
            for (float& inverse_mass : value.inverse_mass)
                if (!reader.read_f32(inverse_mass)) return false;

            for (physics_skin_weight& weight_set : value.skin_weights)
            {
                for (uint16_t& joint : weight_set.joints)
                    if (!reader.read_u16(joint)) return false;
                for (float& weight : weight_set.weights)
                    if (!reader.read_f32(weight)) return false;
            }
            return true;
        }

        bool write_file_header(std::ostream& stream)
        {
            stream.write(reinterpret_cast<const char*>(cache_magic.data()),
                static_cast<std::streamsize>(cache_magic.size()));
            const std::array<uint8_t, 4> version_bytes = {
                static_cast<uint8_t>(cache_file_version & 0xffu),
                static_cast<uint8_t>((cache_file_version >> 8) & 0xffu),
                static_cast<uint8_t>((cache_file_version >> 16) & 0xffu),
                static_cast<uint8_t>((cache_file_version >> 24) & 0xffu)
            };
            stream.write(reinterpret_cast<const char*>(version_bytes.data()),
                static_cast<std::streamsize>(version_bytes.size()));
            return static_cast<bool>(stream);
        }

        bool read_file_header(std::istream& stream, uint32_t& version)
        {
            std::array<uint8_t, 8> magic = {};
            std::array<uint8_t, 4> version_bytes = {};
            stream.read(reinterpret_cast<char*>(magic.data()), static_cast<std::streamsize>(magic.size()));
            stream.read(reinterpret_cast<char*>(version_bytes.data()),
                static_cast<std::streamsize>(version_bytes.size()));
            if (!stream || magic != cache_magic)
            {
                return false;
            }
            version = static_cast<uint32_t>(version_bytes[0]) |
                (static_cast<uint32_t>(version_bytes[1]) << 8) |
                (static_cast<uint32_t>(version_bytes[2]) << 16) |
                (static_cast<uint32_t>(version_bytes[3]) << 24);
            return true;
        }

        bool replace_file(
            const std::filesystem::path& temporary_path,
            const std::filesystem::path& destination_path,
            std::string& error)
        {
#ifdef _WIN32
            if (MoveFileExW(
                temporary_path.c_str(),
                destination_path.c_str(),
                MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == FALSE)
            {
                error = "MoveFileExW failed while replacing the cache file. Win32 error: " +
                    std::to_string(GetLastError());
                return false;
            }
#else
            std::error_code filesystem_error;
            std::filesystem::rename(temporary_path, destination_path, filesystem_error);
            if (filesystem_error)
            {
                error = filesystem_error.message();
                return false;
            }
#endif
            return true;
        }
    }

    physics_asset_cache_write_result save_physics_asset_cache(
        const std::filesystem::path& path,
        const physics_mesh& value,
        const physics_asset_cache_identity& identity)
    {
        physics_asset_cache_write_result result;

        if (path.empty() || !valid_identity(identity))
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::invalid_identity,
                "Cache path and all required cache identity fields must be valid.");
            return result;
        }

        std::string validation_error;
        if (!validate_mesh(value, validation_error))
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::invalid_mesh, validation_error);
            return result;
        }

        std::error_code filesystem_error;
        if (!path.parent_path().empty())
        {
            std::filesystem::create_directories(path.parent_path(), filesystem_error);
            if (filesystem_error)
            {
                add_diagnostic(result.diagnostics, physics_asset_cache_error::io_error,
                    "Could not create cache directory: " + filesystem_error.message());
                return result;
            }
        }

        std::filesystem::path temporary_path = path;
        temporary_path += ".tmp";

        {
            std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
            if (!stream)
            {
                add_diagnostic(result.diagnostics, physics_asset_cache_error::io_error,
                    "Could not create the temporary cache file.");
                return result;
            }
            if (!write_file_header(stream))
            {
                stream.close();
                std::filesystem::remove(temporary_path, filesystem_error);
                add_diagnostic(result.diagnostics, physics_asset_cache_error::io_error,
                    "Could not write the temporary cache file header.");
                return result;
            }

            stream_writer writer(stream);
            if (!write_identity(writer, identity) || !write_mesh(writer, value))
            {
                stream.close();
                std::filesystem::remove(temporary_path, filesystem_error);
                add_diagnostic(result.diagnostics, physics_asset_cache_error::io_error,
                    "Failed while writing Physics Asset Cache payload.");
                return result;
            }

            const uint64_t checksum = writer.hash;
            const std::array<uint8_t, 8> checksum_bytes = {
                static_cast<uint8_t>(checksum & 0xffull),
                static_cast<uint8_t>((checksum >> 8) & 0xffull),
                static_cast<uint8_t>((checksum >> 16) & 0xffull),
                static_cast<uint8_t>((checksum >> 24) & 0xffull),
                static_cast<uint8_t>((checksum >> 32) & 0xffull),
                static_cast<uint8_t>((checksum >> 40) & 0xffull),
                static_cast<uint8_t>((checksum >> 48) & 0xffull),
                static_cast<uint8_t>((checksum >> 56) & 0xffull)
            };
            stream.write(reinterpret_cast<const char*>(checksum_bytes.data()),
                static_cast<std::streamsize>(checksum_bytes.size()));
            stream.flush();
            if (!stream)
            {
                stream.close();
                std::filesystem::remove(temporary_path, filesystem_error);
                add_diagnostic(result.diagnostics, physics_asset_cache_error::io_error,
                    "Failed to flush Physics Asset Cache.");
                return result;
            }
        }

        std::string replace_error;
        if (!replace_file(temporary_path, path, replace_error))
        {
            std::filesystem::remove(temporary_path, filesystem_error);
            add_diagnostic(result.diagnostics, physics_asset_cache_error::io_error,
                "Could not atomically replace the cache file: " + replace_error);
            return result;
        }

        result.success = true;
        return result;
    }

    physics_asset_cache_read_result load_physics_asset_cache(
        const std::filesystem::path& path,
        const physics_asset_cache_identity& expected_identity,
        uint64_t maximum_cache_bytes)
    {
        physics_asset_cache_read_result result;

        if (path.empty() || !valid_identity(expected_identity))
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::invalid_identity,
                "Cache path and expected cache identity must be valid.");
            return result;
        }

        std::error_code filesystem_error;
        const uint64_t file_size = std::filesystem::file_size(path, filesystem_error);
        if (filesystem_error || file_size < 8 + 4 + 8)
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::io_error,
                "Cache file is missing, too small or unreadable.");
            result.needs_rebuild = true;
            return result;
        }
        if (file_size > maximum_cache_bytes)
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::capacity_exceeded,
                "Cache file exceeds maximum_cache_bytes.");
            result.needs_rebuild = true;
            return result;
        }

        std::ifstream stream(path, std::ios::binary);
        if (!stream)
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::io_error,
                "Could not open Physics Asset Cache.");
            result.needs_rebuild = true;
            return result;
        }

        uint32_t file_version = 0;
        if (!read_file_header(stream, file_version))
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::corrupt_data,
                "Physics Asset Cache magic is invalid or its header is truncated.");
            result.needs_rebuild = true;
            return result;
        }
        if (file_version != cache_file_version)
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::unsupported_version,
                "Physics Asset Cache file version is unsupported.");
            result.needs_rebuild = true;
            return result;
        }

        stream_reader reader(stream);
        if (!read_identity(reader, result.identity) || !valid_identity(result.identity))
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::corrupt_data,
                "Physics Asset Cache identity is truncated or invalid.");
            result.needs_rebuild = true;
            return result;
        }

        if (!equal_identity(result.identity, expected_identity))
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::incompatible_identity,
                "Physics Asset Cache identity does not match the current source asset or generation settings.");
            result.needs_rebuild = true;
            return result;
        }

        if (!read_mesh(reader, result.value, file_size))
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::corrupt_data,
                "Physics Asset Cache payload is truncated or contains invalid array sizes.");
            result.needs_rebuild = true;
            return result;
        }

        uint64_t stored_checksum = 0;
        std::array<uint8_t, 8> checksum_bytes = {};
        stream.read(reinterpret_cast<char*>(checksum_bytes.data()),
            static_cast<std::streamsize>(checksum_bytes.size()));
        if (!stream)
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::corrupt_data,
                "Physics Asset Cache checksum is missing.");
            result.needs_rebuild = true;
            return result;
        }
        stored_checksum = static_cast<uint64_t>(checksum_bytes[0]) |
            (static_cast<uint64_t>(checksum_bytes[1]) << 8) |
            (static_cast<uint64_t>(checksum_bytes[2]) << 16) |
            (static_cast<uint64_t>(checksum_bytes[3]) << 24) |
            (static_cast<uint64_t>(checksum_bytes[4]) << 32) |
            (static_cast<uint64_t>(checksum_bytes[5]) << 40) |
            (static_cast<uint64_t>(checksum_bytes[6]) << 48) |
            (static_cast<uint64_t>(checksum_bytes[7]) << 56);

        if (stored_checksum != reader.hash || stream.peek() != std::char_traits<char>::eof())
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::corrupt_data,
                "Physics Asset Cache checksum or file length is invalid.");
            result.needs_rebuild = true;
            return result;
        }

        std::string validation_error;
        if (!validate_mesh(result.value, validation_error))
        {
            add_diagnostic(result.diagnostics, physics_asset_cache_error::corrupt_data, validation_error);
            result.value = {};
            result.needs_rebuild = true;
            return result;
        }

        result.success = true;
        return result;
    }
}
