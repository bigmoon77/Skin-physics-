#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace skin_physics::asset
{
    constexpr uint32_t invalid_index = 0xffffffffu;
    constexpr std::size_t max_skin_influences = 8;

    struct vec2
    {
        float x = 0.0f;
        float y = 0.0f;
    };

    struct vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct vec4
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 0.0f;
    };

    // Column-major matrix. The storage matches glTF MAT4 accessor ordering.
    struct mat4
    {
        std::array<float, 16> value = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };
    };

    enum class physics_mode : uint8_t
    {
        none,
        soft_body,
        cloth
    };

    enum vertex_attribute : uint32_t
    {
        attribute_none = 0,
        attribute_normal = 1u << 0,
        attribute_tangent = 1u << 1,
        attribute_texcoord_0 = 1u << 2,
        attribute_color_0 = 1u << 3,
        attribute_skinning = 1u << 4
    };

    enum class physics_attribute_channel : uint8_t
    {
        stretch,
        shear,
        bend,
        volume,
        shape_recovery,
        damping,
        animation_tracking,
        count
    };

    struct vertex
    {
        vec3 position;
        vec3 normal;
        vec4 tangent;
        vec2 texcoord_0;
        vec4 color = { 1.0f, 1.0f, 1.0f, 1.0f };
        std::array<uint16_t, max_skin_influences> joints = {};
        std::array<float, max_skin_influences> weights = {};
        std::array<float, static_cast<std::size_t>(physics_attribute_channel::count)> physics_weights = {
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
        };
        uint32_t attributes = attribute_none;
    };

    struct submesh
    {
        std::string name;
        uint32_t first_index = 0;
        uint32_t index_count = 0;
        uint32_t material_index = invalid_index;
    };

    struct vertex_group
    {
        std::string name;
        std::vector<float> weights;
    };

    struct mesh
    {
        std::string name;
        std::vector<vertex> vertices;
        std::vector<vertex_group> vertex_groups;
        std::vector<uint32_t> indices;
        std::vector<submesh> submeshes;
        physics_mode mode = physics_mode::none;
        bool collision_enabled = true;
        bool self_collision_enabled = true;
    };

    struct material
    {
        std::string name;
        vec4 base_color_factor = { 1.0f, 1.0f, 1.0f, 1.0f };
        vec3 emissive_factor;
        float metallic_factor = 1.0f;
        float roughness_factor = 1.0f;
        float alpha_cutoff = 0.5f;
        bool double_sided = false;
        bool alpha_blend = false;
        bool alpha_mask = false;
        uint32_t base_color_texture = invalid_index;
        uint32_t metallic_roughness_texture = invalid_index;
        uint32_t normal_texture = invalid_index;
        uint32_t emissive_texture = invalid_index;
    };

    struct image
    {
        std::string name;
        std::string uri;
        std::string mime_type;
        std::vector<uint8_t> encoded_data;
    };

    struct sampler
    {
        int32_t mag_filter = -1;
        int32_t min_filter = -1;
        int32_t wrap_s = 10497;
        int32_t wrap_t = 10497;
    };

    struct texture
    {
        std::string name;
        uint32_t image_index = invalid_index;
        uint32_t sampler_index = invalid_index;
    };

    struct node
    {
        std::string name;
        uint32_t parent_index = invalid_index;
        std::vector<uint32_t> children;
        uint32_t mesh_index = invalid_index;
        uint32_t skin_index = invalid_index;
        mat4 local_matrix;
        vec3 translation;
        vec4 rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
        vec3 scale = { 1.0f, 1.0f, 1.0f };
        bool uses_matrix = false;
    };

    struct skin
    {
        std::string name;
        uint32_t skeleton_node_index = invalid_index;
        std::vector<uint32_t> joints;
        std::vector<mat4> inverse_bind_matrices;
    };

    enum class animation_path : uint8_t
    {
        translation,
        rotation,
        scale
    };

    enum class interpolation : uint8_t
    {
        step,
        linear,
        cubic_spline
    };

    struct animation_key
    {
        float time = 0.0f;
        std::vector<float> in_tangent;
        std::vector<float> value;
        std::vector<float> out_tangent;
    };

    struct animation_track
    {
        uint32_t node_index = invalid_index;
        animation_path path = animation_path::translation;
        interpolation mode = interpolation::linear;
        uint32_t component_count = 0;
        std::vector<animation_key> keys;
    };

    struct animation_clip
    {
        std::string name;
        float duration = 0.0f;
        std::vector<animation_track> tracks;
    };

    struct model
    {
        std::string source_path;
        std::vector<mesh> meshes;
        std::vector<material> materials;
        std::vector<image> images;
        std::vector<sampler> samplers;
        std::vector<texture> textures;
        std::vector<node> nodes;
        std::vector<skin> skins;
        std::vector<animation_clip> animations;
        std::vector<uint32_t> scene_roots;
    };

    enum class diagnostic_severity : uint8_t
    {
        info,
        warning,
        error
    };

    enum class diagnostic_code : uint16_t
    {
        none,
        file_not_found,
        unsupported_format,
        unsupported_feature,
        invalid_text,
        invalid_index,
        invalid_attribute,
        invalid_topology,
        invalid_material,
        invalid_skin,
        invalid_animation,
        invalid_transform,
        invalid_unit_scale,
        io_error,
        out_of_memory,
        validation_failed,
        internal_error
    };

    struct diagnostic
    {
        diagnostic_severity severity = diagnostic_severity::info;
        diagnostic_code code = diagnostic_code::none;
        std::string message;
        uint32_t line = 0;
    };

    struct import_options
    {
        // OBJ has no declared unit. Set this to an explicit positive meter scale before importing OBJ.
        bool has_obj_unit_scale = false;
        float obj_unit_scale = 1.0f;
    };

    struct import_result
    {
        model asset;
        std::vector<diagnostic> diagnostics;
        bool success = false;
    };
}
