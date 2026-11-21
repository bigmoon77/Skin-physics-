#pragma once

#include <cstdint>
#include <string>

#include <d3d11.h>
#include <wrl/client.h>

#include "skin_physics/physics/animation_track.hpp"

namespace skin_physics::physics
{
    struct animation_trajectory_gpu_buffer_counts
    {
        uint32_t spans = 0u;
        uint32_t tracks = 0u;
        uint32_t channels = 0u;
        uint32_t blends = 0u;
        uint32_t inverse_bind_matrices = 0u;
        uint32_t static_node_matrices = 0u;
        uint32_t skins = 0u;
        uint32_t skin_joints = 0u;
        uint32_t colliders = 0u;
    };

    // Owns one immutable Animation Trajectory Packet on the GPU and one
    // independent mutable Segment Cursor for a Model within its World. All
    // Skinned Mesh consumers of that Model share this Cursor for each Segment.
    //
    // The immutable StructuredBuffers use D3D11_USAGE_IMMUTABLE. The Cursor
    // uses D3D11_USAGE_DEFAULT and has both SRV / UAV views so compute passes
    // can advance it once after all Model Mesh consumers without rewriting
    // the Packet.
    class animation_trajectory_gpu_resources
    {
    public:
        animation_trajectory_gpu_resources() = default;
        animation_trajectory_gpu_resources(
            const animation_trajectory_gpu_resources&) = delete;
        animation_trajectory_gpu_resources& operator=(
            const animation_trajectory_gpu_resources&) = delete;
        animation_trajectory_gpu_resources(
            animation_trajectory_gpu_resources&&) noexcept = default;
        animation_trajectory_gpu_resources& operator=(
            animation_trajectory_gpu_resources&&) noexcept = default;

        bool valid() const noexcept;
        uint64_t animation_input_version() const noexcept;
        const animation_trajectory_gpu_buffer_counts& counts() const noexcept;

        ID3D11Buffer* span_buffer() const noexcept;
        ID3D11Buffer* cursor_buffer() const noexcept;
        ID3D11Buffer* track_buffer() const noexcept;
        ID3D11Buffer* channel_buffer() const noexcept;
        ID3D11Buffer* blend_buffer() const noexcept;
        ID3D11Buffer* inverse_bind_matrix_buffer() const noexcept;
        ID3D11Buffer* static_node_matrix_buffer() const noexcept;
        ID3D11Buffer* skin_buffer() const noexcept;
        ID3D11Buffer* skin_joint_buffer() const noexcept;
        ID3D11Buffer* collider_buffer() const noexcept;

        ID3D11ShaderResourceView* span_srv() const noexcept;
        ID3D11ShaderResourceView* cursor_srv() const noexcept;
        ID3D11ShaderResourceView* track_srv() const noexcept;
        ID3D11ShaderResourceView* channel_srv() const noexcept;
        ID3D11ShaderResourceView* blend_srv() const noexcept;
        ID3D11ShaderResourceView* inverse_bind_matrix_srv() const noexcept;
        ID3D11ShaderResourceView* static_node_matrix_srv() const noexcept;
        ID3D11ShaderResourceView* skin_srv() const noexcept;
        ID3D11ShaderResourceView* skin_joint_srv() const noexcept;
        ID3D11ShaderResourceView* collider_srv() const noexcept;

        ID3D11UnorderedAccessView* cursor_uav() const noexcept;

        void reset() noexcept;

    private:
        friend bool upload_animation_trajectory_packet(
            ID3D11Device*,
            const animation_trajectory_packet&,
            const animation_span_cursor_gpu_record&,
            animation_trajectory_gpu_resources&,
            std::string&);

        uint64_t _animation_input_version = 0u;
        animation_trajectory_gpu_buffer_counts _counts;

        Microsoft::WRL::ComPtr<ID3D11Buffer> _span_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _cursor_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _track_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _channel_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _blend_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _inverse_bind_matrix_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _static_node_matrix_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _skin_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _skin_joint_buffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _collider_buffer;

        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _span_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _cursor_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _track_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _channel_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _blend_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _inverse_bind_matrix_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _static_node_matrix_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _skin_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _skin_joint_srv;
        Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> _collider_srv;
        Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> _cursor_uav;
    };

    // Upload is transactional: every immutable buffer and the mutable Cursor
    // are created in a temporary bundle. out_resources is replaced only after
    // all D3D11 resources and views have been created successfully.
    bool upload_animation_trajectory_packet(
        ID3D11Device* device,
        const animation_trajectory_packet& packet,
        const animation_span_cursor_gpu_record& initial_cursor,
        animation_trajectory_gpu_resources& out_resources,
        std::string& diagnostic);
}
