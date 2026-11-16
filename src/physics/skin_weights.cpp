#include "skin_physics/physics/skin_weights.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace skin_physics::physics
{
    namespace
    {
        using asset::vec3;

        constexpr float minimum_edge_length_squared = 1.0e-20f;
        constexpr float minimum_triangle_area_squared = 1.0e-24f;
        constexpr float minimum_weight_sum = 1.0e-8f;

        struct aabb
        {
            vec3 minimum = {};
            vec3 maximum = {};
        };

        struct triangle_primitive
        {
            uint32_t triangle_index = 0;
            aabb bounds;
            vec3 centroid;
        };

        struct bvh_node
        {
            aabb bounds;
            uint32_t begin = 0;
            uint32_t count = 0;
            int32_t left = -1;
            int32_t right = -1;
        };

        struct closest_triangle_result
        {
            std::array<float, 3> barycentric = {};
            float distance_squared = std::numeric_limits<float>::max();
            uint32_t triangle_index = asset::invalid_index;
            bool found = false;
        };

        struct weighted_joint
        {
            uint16_t joint = 0;
            float weight = 0.0f;
        };

        using top_influences = std::array<weighted_joint, asset::max_skin_influences>;

        vec3 add(const vec3& left, const vec3& right)
        {
            return { left.x + right.x, left.y + right.y, left.z + right.z };
        }

        vec3 subtract(const vec3& left, const vec3& right)
        {
            return { left.x - right.x, left.y - right.y, left.z - right.z };
        }

        vec3 multiply(const vec3& value, float scale)
        {
            return { value.x * scale, value.y * scale, value.z * scale };
        }

        float dot(const vec3& left, const vec3& right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }

        vec3 cross(const vec3& left, const vec3& right)
        {
            return {
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x
            };
        }

        float length_squared(const vec3& value)
        {
            return dot(value, value);
        }

        bool finite(const vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        vec3 component_min(const vec3& left, const vec3& right)
        {
            return {
                std::min(left.x, right.x),
                std::min(left.y, right.y),
                std::min(left.z, right.z)
            };
        }

        vec3 component_max(const vec3& left, const vec3& right)
        {
            return {
                std::max(left.x, right.x),
                std::max(left.y, right.y),
                std::max(left.z, right.z)
            };
        }

        float distance_squared_to_aabb(const vec3& point, const aabb& bounds)
        {
            float distance_squared = 0.0f;
            const std::array<float, 3> point_values = { point.x, point.y, point.z };
            const std::array<float, 3> minimum_values = {
                bounds.minimum.x, bounds.minimum.y, bounds.minimum.z
            };
            const std::array<float, 3> maximum_values = {
                bounds.maximum.x, bounds.maximum.y, bounds.maximum.z
            };

            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                float difference = 0.0f;
                if (point_values[axis] < minimum_values[axis])
                {
                    difference = minimum_values[axis] - point_values[axis];
                }
                else if (point_values[axis] > maximum_values[axis])
                {
                    difference = point_values[axis] - maximum_values[axis];
                }
                distance_squared += difference * difference;
            }
            return distance_squared;
        }

        bool closest_point_on_triangle(
            const vec3& point,
            const vec3& a,
            const vec3& b,
            const vec3& c,
            std::array<float, 3>& barycentric,
            float& output_distance_squared)
        {
            const vec3 ab = subtract(b, a);
            const vec3 ac = subtract(c, a);
            if (length_squared(cross(ab, ac)) <= minimum_triangle_area_squared)
            {
                return false;
            }

            const vec3 ap = subtract(point, a);
            const float d1 = dot(ab, ap);
            const float d2 = dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                barycentric = { 1.0f, 0.0f, 0.0f };
                output_distance_squared = length_squared(ap);
                return true;
            }

            const vec3 bp = subtract(point, b);
            const float d3 = dot(ab, bp);
            const float d4 = dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3)
            {
                barycentric = { 0.0f, 1.0f, 0.0f };
                output_distance_squared = length_squared(bp);
                return true;
            }

            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                const float denominator = d1 - d3;
                if (std::abs(denominator) <= 1.0e-20f)
                {
                    return false;
                }
                const float v = d1 / denominator;
                barycentric = { 1.0f - v, v, 0.0f };
                const vec3 closest = add(a, multiply(ab, v));
                output_distance_squared = length_squared(subtract(point, closest));
                return true;
            }

            const vec3 cp = subtract(point, c);
            const float d5 = dot(ab, cp);
            const float d6 = dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6)
            {
                barycentric = { 0.0f, 0.0f, 1.0f };
                output_distance_squared = length_squared(cp);
                return true;
            }

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                const float denominator = d2 - d6;
                if (std::abs(denominator) <= 1.0e-20f)
                {
                    return false;
                }
                const float w = d2 / denominator;
                barycentric = { 1.0f - w, 0.0f, w };
                const vec3 closest = add(a, multiply(ac, w));
                output_distance_squared = length_squared(subtract(point, closest));
                return true;
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                const float denominator = (d4 - d3) + (d5 - d6);
                if (std::abs(denominator) <= 1.0e-20f)
                {
                    return false;
                }
                const float w = (d4 - d3) / denominator;
                barycentric = { 0.0f, 1.0f - w, w };
                const vec3 closest = add(b, multiply(subtract(c, b), w));
                output_distance_squared = length_squared(subtract(point, closest));
                return true;
            }

            const float denominator = va + vb + vc;
            if (!std::isfinite(denominator) || std::abs(denominator) <= 1.0e-20f)
            {
                return false;
            }

            const float reciprocal = 1.0f / denominator;
            const float v = vb * reciprocal;
            const float w = vc * reciprocal;
            const float u = 1.0f - v - w;
            barycentric = { u, v, w };
            const vec3 closest = add(add(multiply(a, u), multiply(b, v)), multiply(c, w));
            output_distance_squared = length_squared(subtract(point, closest));
            return std::isfinite(output_distance_squared);
        }

        class surface_bvh
        {
        public:
            bool build(const asset::mesh& surface_mesh, std::string& error)
            {
                primitives_.clear();
                nodes_.clear();

                if (surface_mesh.indices.empty() || surface_mesh.indices.size() % 3 != 0)
                {
                    error = "Render Mesh must contain triangle indices.";
                    return false;
                }

                for (uint32_t index : surface_mesh.indices)
                {
                    if (index >= surface_mesh.vertices.size())
                    {
                        error = "Render Mesh contains an out-of-range triangle index.";
                        return false;
                    }
                }

                for (const asset::vertex& vertex : surface_mesh.vertices)
                {
                    if (!finite(vertex.position))
                    {
                        error = "Render Mesh contains a non-finite position.";
                        return false;
                    }
                }

                primitives_.reserve(surface_mesh.indices.size() / 3);
                for (std::size_t index = 0; index < surface_mesh.indices.size(); index += 3)
                {
                    const vec3& a = surface_mesh.vertices[surface_mesh.indices[index]].position;
                    const vec3& b = surface_mesh.vertices[surface_mesh.indices[index + 1]].position;
                    const vec3& c = surface_mesh.vertices[surface_mesh.indices[index + 2]].position;
                    if (length_squared(cross(subtract(b, a), subtract(c, a))) <= minimum_triangle_area_squared)
                    {
                        continue;
                    }

                    triangle_primitive primitive;
                    primitive.triangle_index = static_cast<uint32_t>(index / 3);
                    primitive.bounds.minimum = component_min(a, component_min(b, c));
                    primitive.bounds.maximum = component_max(a, component_max(b, c));
                    primitive.centroid = multiply(add(add(a, b), c), 1.0f / 3.0f);
                    primitives_.push_back(primitive);
                }

                if (primitives_.empty())
                {
                    error = "Render Mesh contains no non-degenerate triangles.";
                    return false;
                }

                build_node(0, static_cast<uint32_t>(primitives_.size()));
                return true;
            }

            closest_triangle_result find_closest(
                const vec3& point,
                const asset::mesh& surface_mesh) const
            {
                closest_triangle_result result;
                if (!nodes_.empty())
                {
                    query_node(0, point, surface_mesh, result);
                }
                return result;
            }

        private:
            int32_t build_node(uint32_t begin, uint32_t end)
            {
                bvh_node node;
                node.begin = begin;
                node.count = end - begin;

                node.bounds.minimum = {
                    std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max()
                };
                node.bounds.maximum = {
                    -std::numeric_limits<float>::max(),
                    -std::numeric_limits<float>::max(),
                    -std::numeric_limits<float>::max()
                };

                for (uint32_t index = begin; index < end; ++index)
                {
                    node.bounds.minimum = component_min(node.bounds.minimum, primitives_[index].bounds.minimum);
                    node.bounds.maximum = component_max(node.bounds.maximum, primitives_[index].bounds.maximum);
                }

                const int32_t node_index = static_cast<int32_t>(nodes_.size());
                nodes_.push_back(node);

                constexpr uint32_t leaf_size = 8;
                if (end - begin <= leaf_size)
                {
                    return node_index;
                }

                const vec3 extent = subtract(node.bounds.maximum, node.bounds.minimum);
                uint32_t split_axis = 0;
                if (extent.y > extent.x && extent.y >= extent.z)
                {
                    split_axis = 1;
                }
                else if (extent.z > extent.x && extent.z > extent.y)
                {
                    split_axis = 2;
                }

                const uint32_t middle = begin + (end - begin) / 2;
                std::nth_element(
                    primitives_.begin() + begin,
                    primitives_.begin() + middle,
                    primitives_.begin() + end,
                    [split_axis](const triangle_primitive& left, const triangle_primitive& right)
                    {
                        if (split_axis == 0) return left.centroid.x < right.centroid.x;
                        if (split_axis == 1) return left.centroid.y < right.centroid.y;
                        return left.centroid.z < right.centroid.z;
                    });

                const int32_t left = build_node(begin, middle);
                const int32_t right = build_node(middle, end);
                nodes_[static_cast<std::size_t>(node_index)].left = left;
                nodes_[static_cast<std::size_t>(node_index)].right = right;
                nodes_[static_cast<std::size_t>(node_index)].count = 0;
                return node_index;
            }

            void query_node(
                int32_t node_index,
                const vec3& point,
                const asset::mesh& surface_mesh,
                closest_triangle_result& result) const
            {
                const bvh_node& node = nodes_[static_cast<std::size_t>(node_index)];
                if (distance_squared_to_aabb(point, node.bounds) > result.distance_squared)
                {
                    return;
                }

                if (node.left < 0 && node.right < 0)
                {
                    for (uint32_t index = node.begin; index < node.begin + node.count; ++index)
                    {
                        const triangle_primitive& primitive = primitives_[index];
                        const std::size_t base = static_cast<std::size_t>(primitive.triangle_index) * 3;
                        const vec3& a = surface_mesh.vertices[surface_mesh.indices[base]].position;
                        const vec3& b = surface_mesh.vertices[surface_mesh.indices[base + 1]].position;
                        const vec3& c = surface_mesh.vertices[surface_mesh.indices[base + 2]].position;

                        std::array<float, 3> barycentric = {};
                        float distance_squared = 0.0f;
                        if (closest_point_on_triangle(
                            point, a, b, c, barycentric, distance_squared) &&
                            (distance_squared < result.distance_squared ||
                             (distance_squared == result.distance_squared &&
                              primitive.triangle_index < result.triangle_index)))
                        {
                            result.distance_squared = distance_squared;
                            result.barycentric = barycentric;
                            result.triangle_index = primitive.triangle_index;
                            result.found = true;
                        }
                    }
                    return;
                }

                const float left_distance = node.left >= 0
                    ? distance_squared_to_aabb(point, nodes_[static_cast<std::size_t>(node.left)].bounds)
                    : std::numeric_limits<float>::max();
                const float right_distance = node.right >= 0
                    ? distance_squared_to_aabb(point, nodes_[static_cast<std::size_t>(node.right)].bounds)
                    : std::numeric_limits<float>::max();

                if (left_distance <= right_distance)
                {
                    if (node.left >= 0) query_node(node.left, point, surface_mesh, result);
                    if (node.right >= 0) query_node(node.right, point, surface_mesh, result);
                }
                else
                {
                    if (node.right >= 0) query_node(node.right, point, surface_mesh, result);
                    if (node.left >= 0) query_node(node.left, point, surface_mesh, result);
                }
            }

            std::vector<triangle_primitive> primitives_;
            std::vector<bvh_node> nodes_;
        };

        void sort_influences(top_influences& influences, uint32_t count)
        {
            std::sort(
                influences.begin(),
                influences.begin() + count,
                [](const weighted_joint& left, const weighted_joint& right)
                {
                    if (left.weight != right.weight) return left.weight > right.weight;
                    return left.joint < right.joint;
                });
        }

        void insert_influence(
            top_influences& influences,
            uint32_t& count,
            uint16_t joint,
            float weight)
        {
            if (weight <= 0.0f || !std::isfinite(weight))
            {
                return;
            }

            for (uint32_t index = 0; index < count; ++index)
            {
                if (influences[index].joint == joint)
                {
                    influences[index].weight += weight;
                    sort_influences(influences, count);
                    return;
                }
            }

            if (count < asset::max_skin_influences)
            {
                influences[count++] = { joint, weight };
                sort_influences(influences, count);
                return;
            }

            if (weight > influences[count - 1].weight)
            {
                influences[count - 1] = { joint, weight };
                sort_influences(influences, count);
            }
        }

        float find_weight(const std::map<uint16_t, float>& weights, uint16_t joint)
        {
            const auto iterator = weights.find(joint);
            return iterator == weights.end() ? 0.0f : iterator->second;
        }

        float vector_dot(const std::vector<float>& left, const std::vector<float>& right)
        {
            float value = 0.0f;
            for (std::size_t index = 0; index < left.size(); ++index)
            {
                value += left[index] * right[index];
            }
            return value;
        }

        void add_diagnostic(
            skin_weight_transfer_result& result,
            skin_weight_error code,
            uint32_t element_index,
            const std::string& message)
        {
            skin_weight_diagnostic diagnostic;
            diagnostic.code = code;
            diagnostic.element_index = element_index;
            diagnostic.message = message;
            result.diagnostics.push_back(std::move(diagnostic));
        }
    }

    skin_weight_transfer_result transfer_skin_weights(
        const asset::mesh& render_mesh,
        const asset::skin& render_skin,
        physics_mesh& target_mesh,
        const skin_weight_transfer_settings& settings)
    {
        skin_weight_transfer_result result;

        if (!std::isfinite(settings.maximum_surface_distance) ||
            settings.maximum_surface_distance < 0.0f ||
            settings.maximum_iterations == 0 ||
            !std::isfinite(settings.relative_residual_tolerance) ||
            settings.relative_residual_tolerance <= 0.0f ||
            settings.relative_residual_tolerance >= 1.0f)
        {
            add_diagnostic(result, skin_weight_error::invalid_result, asset::invalid_index,
                "Skin weight transfer settings are invalid.");
            return result;
        }

        if (render_mesh.vertices.empty() || render_mesh.indices.empty() ||
            render_mesh.indices.size() % 3 != 0)
        {
            add_diagnostic(result, skin_weight_error::invalid_surface_mesh, asset::invalid_index,
                "Render Mesh must contain indexed triangles.");
            return result;
        }

        if (render_skin.joints.empty() ||
            render_skin.joints.size() > static_cast<std::size_t>(std::numeric_limits<uint16_t>::max()) + 1)
        {
            add_diagnostic(result, skin_weight_error::invalid_skin, asset::invalid_index,
                "Skin must contain between 1 and 65536 joints.");
            return result;
        }

        if (target_mesh.rest_positions.empty() || target_mesh.tetrahedra.empty() ||
            target_mesh.boundary_triangles.empty())
        {
            add_diagnostic(result, skin_weight_error::invalid_physics_mesh, asset::invalid_index,
                "Physics Mesh must contain tetrahedra and an extracted boundary surface.");
            return result;
        }

        std::string bvh_error;
        surface_bvh bvh;
        if (!bvh.build(render_mesh, bvh_error))
        {
            add_diagnostic(result, skin_weight_error::no_valid_surface_triangles, asset::invalid_index, bvh_error);
            return result;
        }

        for (std::size_t vertex_index = 0; vertex_index < render_mesh.vertices.size(); ++vertex_index)
        {
            const asset::vertex& vertex = render_mesh.vertices[vertex_index];
            if ((vertex.attributes & asset::attribute_skinning) == 0)
            {
                add_diagnostic(result, skin_weight_error::invalid_surface_mesh,
                    static_cast<uint32_t>(vertex_index),
                    "Every Render Mesh vertex must contain skinning attributes.");
                return result;
            }

            float weight_sum = 0.0f;
            for (std::size_t influence_index = 0; influence_index < asset::max_skin_influences; ++influence_index)
            {
                const float weight = vertex.weights[influence_index];
                if (!std::isfinite(weight) || weight < 0.0f ||
                    (weight > 0.0f && vertex.joints[influence_index] >= render_skin.joints.size()))
                {
                    add_diagnostic(result, skin_weight_error::invalid_skin,
                        static_cast<uint32_t>(vertex_index),
                        "Render Mesh contains an invalid joint index or skin weight.");
                    return result;
                }
                weight_sum += weight;
            }

            if (!std::isfinite(weight_sum) || std::abs(weight_sum - 1.0f) > 1.0e-3f)
            {
                add_diagnostic(result, skin_weight_error::invalid_skin,
                    static_cast<uint32_t>(vertex_index),
                    "Render Mesh skin weights must be normalized.");
                return result;
            }
        }

        const std::size_t vertex_count = target_mesh.rest_positions.size();
        std::vector<bool> is_boundary(vertex_count, false);
        for (const boundary_triangle& triangle : target_mesh.boundary_triangles)
        {
            for (uint32_t vertex_index : triangle.vertices)
            {
                if (vertex_index >= vertex_count)
                {
                    add_diagnostic(result, skin_weight_error::invalid_physics_mesh, vertex_index,
                        "Physics boundary triangle contains an out-of-range vertex.");
                    return result;
                }
                is_boundary[vertex_index] = true;
            }
        }

        std::vector<std::map<uint16_t, float>> boundary_weights(vertex_count);
        std::set<uint16_t> active_joints;
        const float maximum_distance_squared =
            settings.maximum_surface_distance * settings.maximum_surface_distance;

        for (std::size_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            if (!finite(target_mesh.rest_positions[vertex_index]))
            {
                add_diagnostic(result, skin_weight_error::invalid_physics_mesh,
                    static_cast<uint32_t>(vertex_index),
                    "Physics Mesh contains a non-finite rest position.");
                return result;
            }

            if (!is_boundary[vertex_index])
            {
                continue;
            }

            const closest_triangle_result closest =
                bvh.find_closest(target_mesh.rest_positions[vertex_index], render_mesh);
            if (!closest.found || closest.distance_squared > maximum_distance_squared)
            {
                add_diagnostic(result, skin_weight_error::boundary_match_too_far,
                    static_cast<uint32_t>(vertex_index),
                    "Physics boundary vertex is farther from the Render Mesh than maximum_surface_distance.");
                return result;
            }

            const std::size_t triangle_base = static_cast<std::size_t>(closest.triangle_index) * 3;
            const std::array<uint32_t, 3> surface_vertices = {
                render_mesh.indices[triangle_base],
                render_mesh.indices[triangle_base + 1],
                render_mesh.indices[triangle_base + 2]
            };

            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                const asset::vertex& source_vertex = render_mesh.vertices[surface_vertices[corner]];
                const float barycentric_weight = closest.barycentric[corner];
                for (std::size_t influence_index = 0; influence_index < asset::max_skin_influences; ++influence_index)
                {
                    const float source_weight = source_vertex.weights[influence_index] * barycentric_weight;
                    if (source_weight > 0.0f)
                    {
                        const uint16_t joint = source_vertex.joints[influence_index];
                        boundary_weights[vertex_index][joint] += source_weight;
                        active_joints.insert(joint);
                    }
                }
            }

            float sum = 0.0f;
            for (const auto& influence : boundary_weights[vertex_index])
            {
                sum += influence.second;
            }
            if (!std::isfinite(sum) || sum <= minimum_weight_sum)
            {
                add_diagnostic(result, skin_weight_error::invalid_result,
                    static_cast<uint32_t>(vertex_index),
                    "Interpolated boundary skin weights have no positive influence.");
                return result;
            }
            for (auto& influence : boundary_weights[vertex_index])
            {
                influence.second /= sum;
            }
        }

        if (active_joints.empty())
        {
            add_diagnostic(result, skin_weight_error::invalid_skin, asset::invalid_index,
                "No positive skin influences were found on the Physics boundary.");
            return result;
        }

        using edge_key = std::pair<uint32_t, uint32_t>;
        std::map<edge_key, float> unique_edges;
        for (std::size_t tet_index = 0; tet_index < target_mesh.tetrahedra.size(); ++tet_index)
        {
            const tetrahedron& tet = target_mesh.tetrahedra[tet_index];
            for (uint32_t vertex_index : tet.vertices)
            {
                if (vertex_index >= vertex_count)
                {
                    add_diagnostic(result, skin_weight_error::invalid_physics_mesh,
                        static_cast<uint32_t>(tet_index),
                        "Physics tetrahedron contains an out-of-range vertex.");
                    return result;
                }
            }

            for (uint32_t first = 0; first < 4; ++first)
            {
                for (uint32_t second = first + 1; second < 4; ++second)
                {
                    uint32_t a = tet.vertices[first];
                    uint32_t b = tet.vertices[second];
                    if (a > b) std::swap(a, b);

                    const float edge_length_squared =
                        length_squared(subtract(target_mesh.rest_positions[a], target_mesh.rest_positions[b]));
                    if (!std::isfinite(edge_length_squared) || edge_length_squared <= minimum_edge_length_squared)
                    {
                        add_diagnostic(result, skin_weight_error::invalid_physics_mesh,
                            static_cast<uint32_t>(tet_index),
                            "Physics Mesh contains a degenerate or non-finite edge.");
                        return result;
                    }

                    const float edge_weight = 1.0f / edge_length_squared;
                    if (!std::isfinite(edge_weight) || edge_weight <= 0.0f)
                    {
                        add_diagnostic(result, skin_weight_error::invalid_physics_mesh,
                            static_cast<uint32_t>(tet_index),
                            "Physics Mesh edge weight is not finite and positive.");
                        return result;
                    }
                    unique_edges.emplace(edge_key{ a, b }, edge_weight);
                }
            }
        }

        struct graph_edge
        {
            uint32_t vertex = 0;
            float weight = 0.0f;
        };

        std::vector<std::vector<graph_edge>> adjacency(vertex_count);
        for (const auto& edge : unique_edges)
        {
            adjacency[edge.first.first].push_back({ edge.first.second, edge.second });
            adjacency[edge.first.second].push_back({ edge.first.first, edge.second });
        }

        std::vector<bool> visited(vertex_count, false);
        for (std::size_t start = 0; start < vertex_count; ++start)
        {
            if (visited[start]) continue;

            bool has_boundary_anchor = false;
            std::queue<uint32_t> pending;
            pending.push(static_cast<uint32_t>(start));
            visited[start] = true;

            while (!pending.empty())
            {
                const uint32_t current = pending.front();
                pending.pop();
                has_boundary_anchor = has_boundary_anchor || is_boundary[current];

                for (const graph_edge& edge : adjacency[current])
                {
                    if (!visited[edge.vertex])
                    {
                        visited[edge.vertex] = true;
                        pending.push(edge.vertex);
                    }
                }
            }

            if (!has_boundary_anchor)
            {
                add_diagnostic(result, skin_weight_error::disconnected_interior_component,
                    static_cast<uint32_t>(start),
                    "A Physics Mesh connected component has no Render boundary anchor.");
                return result;
            }
        }

        std::vector<int32_t> interior_index(vertex_count, -1);
        std::vector<uint32_t> interior_vertices;
        interior_vertices.reserve(vertex_count);
        for (std::size_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            if (!is_boundary[vertex_index])
            {
                interior_index[vertex_index] = static_cast<int32_t>(interior_vertices.size());
                interior_vertices.push_back(static_cast<uint32_t>(vertex_index));
            }
        }

        std::vector<float> diagonal(interior_vertices.size(), 0.0f);
        std::vector<float> right_hand_side(interior_vertices.size(), 0.0f);
        std::vector<float> solution(interior_vertices.size(), 0.0f);
        std::vector<float> residual(interior_vertices.size(), 0.0f);
        std::vector<float> direction(interior_vertices.size(), 0.0f);
        std::vector<float> matrix_direction(interior_vertices.size(), 0.0f);
        std::vector<float> preconditioned(interior_vertices.size(), 0.0f);
        std::vector<top_influences> output_influences(vertex_count);
        std::vector<uint32_t> output_influence_counts(vertex_count, 0);

        for (std::size_t interior = 0; interior < interior_vertices.size(); ++interior)
        {
            const uint32_t vertex_index = interior_vertices[interior];
            for (const graph_edge& edge : adjacency[vertex_index])
            {
                diagonal[interior] += edge.weight;
            }
            if (!std::isfinite(diagonal[interior]) || diagonal[interior] <= 0.0f)
            {
                add_diagnostic(result, skin_weight_error::disconnected_interior_component,
                    vertex_index, "Interior vertex has no positive graph Laplacian diagonal.");
                return result;
            }
        }

        const auto multiply_laplacian = [&](const std::vector<float>& input, std::vector<float>& output)
        {
            for (std::size_t interior = 0; interior < interior_vertices.size(); ++interior)
            {
                const uint32_t vertex_index = interior_vertices[interior];
                float value = diagonal[interior] * input[interior];

                for (const graph_edge& edge : adjacency[vertex_index])
                {
                    const int32_t neighbor_interior = interior_index[edge.vertex];
                    if (neighbor_interior >= 0)
                    {
                        value -= edge.weight * input[static_cast<std::size_t>(neighbor_interior)];
                    }
                }
                output[interior] = value;
            }
        };

        for (const auto& joint_entry : active_joints)
        {
            const uint16_t joint = joint_entry;
            std::fill(right_hand_side.begin(), right_hand_side.end(), 0.0f);
            std::fill(solution.begin(), solution.end(), 0.0f);

            for (std::size_t interior = 0; interior < interior_vertices.size(); ++interior)
            {
                const uint32_t vertex_index = interior_vertices[interior];
                float value = 0.0f;
                for (const graph_edge& edge : adjacency[vertex_index])
                {
                    if (is_boundary[edge.vertex])
                    {
                        value += edge.weight * find_weight(boundary_weights[edge.vertex], joint);
                    }
                }
                right_hand_side[interior] = value;
            }

            const float right_hand_side_norm = std::sqrt(std::max(0.0f,
                vector_dot(right_hand_side, right_hand_side)));
            if (right_hand_side_norm <= 1.0e-20f)
            {
                continue;
            }

            multiply_laplacian(solution, matrix_direction);
            for (std::size_t index = 0; index < residual.size(); ++index)
            {
                residual[index] = right_hand_side[index] - matrix_direction[index];
                preconditioned[index] = residual[index] / diagonal[index];
                direction[index] = preconditioned[index];
            }

            float residual_preconditioned = vector_dot(residual, preconditioned);
            bool converged = false;
            uint32_t iterations_used = 0;

            for (uint32_t iteration = 0; iteration < settings.maximum_iterations; ++iteration)
            {
                multiply_laplacian(direction, matrix_direction);
                const float denominator = vector_dot(direction, matrix_direction);
                if (!std::isfinite(denominator) || denominator <= 1.0e-30f ||
                    !std::isfinite(residual_preconditioned) || residual_preconditioned < 0.0f)
                {
                    break;
                }

                const float alpha = residual_preconditioned / denominator;
                if (!std::isfinite(alpha))
                {
                    break;
                }

                for (std::size_t index = 0; index < solution.size(); ++index)
                {
                    solution[index] += alpha * direction[index];
                    residual[index] -= alpha * matrix_direction[index];
                }

                iterations_used = iteration + 1;
                const float relative_residual = std::sqrt(std::max(0.0f,
                    vector_dot(residual, residual))) / right_hand_side_norm;
                if (std::isfinite(relative_residual) &&
                    relative_residual <= settings.relative_residual_tolerance)
                {
                    result.maximum_relative_residual =
                        std::max(result.maximum_relative_residual, relative_residual);
                    converged = true;
                    break;
                }

                for (std::size_t index = 0; index < residual.size(); ++index)
                {
                    preconditioned[index] = residual[index] / diagonal[index];
                }

                const float next_residual_preconditioned = vector_dot(residual, preconditioned);
                if (!std::isfinite(next_residual_preconditioned) ||
                    next_residual_preconditioned < 0.0f ||
                    residual_preconditioned <= 1.0e-30f)
                {
                    break;
                }

                const float beta = next_residual_preconditioned / residual_preconditioned;
                for (std::size_t index = 0; index < direction.size(); ++index)
                {
                    direction[index] = preconditioned[index] + beta * direction[index];
                }
                residual_preconditioned = next_residual_preconditioned;
            }

            result.maximum_iterations_used =
                std::max(result.maximum_iterations_used, iterations_used);

            if (!converged)
            {
                const float final_relative_residual = right_hand_side_norm > 0.0f
                    ? std::sqrt(std::max(0.0f, vector_dot(residual, residual))) / right_hand_side_norm
                    : 0.0f;
                result.maximum_relative_residual =
                    std::max(result.maximum_relative_residual, final_relative_residual);
                add_diagnostic(result, skin_weight_error::solver_not_converged, joint,
                    "Conjugate-gradient harmonic extension did not reach the configured relative residual.");
                return result;
            }

            for (std::size_t interior = 0; interior < interior_vertices.size(); ++interior)
            {
                float weight = solution[interior];
                if (!std::isfinite(weight))
                {
                    add_diagnostic(result, skin_weight_error::invalid_result,
                        interior_vertices[interior], "Harmonic skin weight is not finite.");
                    return result;
                }
                weight = std::max(0.0f, weight);
                insert_influence(
                    output_influences[interior_vertices[interior]],
                    output_influence_counts[interior_vertices[interior]],
                    joint,
                    weight);
            }
        }

        std::vector<physics_skin_weight> transferred_weights;
        transferred_weights.reserve(vertex_count);

        for (std::size_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            if (is_boundary[vertex_index])
            {
                for (const auto& influence : boundary_weights[vertex_index])
                {
                    insert_influence(
                        output_influences[vertex_index],
                        output_influence_counts[vertex_index],
                        influence.first,
                        influence.second);
                }
            }

            const uint32_t influence_count = output_influence_counts[vertex_index];
            if (influence_count == 0)
            {
                add_diagnostic(result, skin_weight_error::invalid_result,
                    static_cast<uint32_t>(vertex_index),
                    "Physics vertex has no positive skin influence after transfer.");
                return result;
            }

            float weight_sum = 0.0f;
            for (uint32_t influence_index = 0; influence_index < influence_count; ++influence_index)
            {
                weight_sum += output_influences[vertex_index][influence_index].weight;
            }
            if (!std::isfinite(weight_sum) || weight_sum <= minimum_weight_sum)
            {
                add_diagnostic(result, skin_weight_error::invalid_result,
                    static_cast<uint32_t>(vertex_index),
                    "Physics vertex skin weights cannot be normalized.");
                return result;
            }

            transferred_weights.emplace_back();
            physics_skin_weight& destination = transferred_weights.back();
            for (uint32_t influence_index = 0; influence_index < influence_count; ++influence_index)
            {
                destination.joints[influence_index] = output_influences[vertex_index][influence_index].joint;
                destination.weights[influence_index] =
                    output_influences[vertex_index][influence_index].weight / weight_sum;
            }
        }

        if (transferred_weights.size() != vertex_count)
        {
            add_diagnostic(result, skin_weight_error::invalid_result, asset::invalid_index,
                "Transferred skin weight count does not match Physics Mesh vertex count.");
            return result;
        }

        target_mesh.skin_weights = std::move(transferred_weights);
        target_mesh.has_skin_weights = true;
        result.success = true;
        return result;
    }
}
