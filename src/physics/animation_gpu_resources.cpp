#include "skin_physics/physics/animation_gpu_resources.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace skin_physics::physics
{
    namespace
    {
        template<typename T>
        bool valid_structured_buffer_size(
            const std::vector<T>& values,
            const char* label,
            std::string& diagnostic)
        {
            if (values.empty())
            {
                return true;
            }

            if (sizeof(T) == 0u ||
                sizeof(T) > static_cast<std::size_t>(
                    std::numeric_limits<UINT>::max()) ||
                values.size() > static_cast<std::size_t>(
                    std::numeric_limits<UINT>::max()) / sizeof(T))
            {
                diagnostic = std::string("Animation Packet ") + label +
                    " StructuredBuffer exceeds the D3D11 UINT ByteWidth limit.";
                return false;
            }

            return true;
        }

        template<typename T>
        bool create_immutable_structured_buffer(
            ID3D11Device* device,
            const std::vector<T>& values,
            const char* label,
            Microsoft::WRL::ComPtr<ID3D11Buffer>& out_buffer,
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>& out_srv,
            std::string& diagnostic)
        {
            out_buffer.Reset();
            out_srv.Reset();
            if (values.empty())
            {
                return true;
            }

            if (!valid_structured_buffer_size(values, label, diagnostic))
            {
                return false;
            }

            D3D11_BUFFER_DESC buffer_description = {};
            buffer_description.ByteWidth =
                static_cast<UINT>(values.size() * sizeof(T));
            buffer_description.Usage = D3D11_USAGE_IMMUTABLE;
            buffer_description.BindFlags = D3D11_BIND_SHADER_RESOURCE;
            buffer_description.MiscFlags =
                D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
            buffer_description.StructureByteStride =
                static_cast<UINT>(sizeof(T));

            D3D11_SUBRESOURCE_DATA initial_data = {};
            initial_data.pSysMem = values.data();

            HRESULT result = device->CreateBuffer(
                &buffer_description,
                &initial_data,
                out_buffer.GetAddressOf());
            if (FAILED(result))
            {
                diagnostic = std::string("D3D11 CreateBuffer failed for immutable Animation Packet ") +
                    label + " (HRESULT=" + std::to_string(
                        static_cast<uint32_t>(result)) + ").";
                return false;
            }

            D3D11_SHADER_RESOURCE_VIEW_DESC srv_description = {};
            srv_description.Format = DXGI_FORMAT_UNKNOWN;
            srv_description.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
            srv_description.Buffer.FirstElement = 0u;
            srv_description.Buffer.NumElements =
                static_cast<UINT>(values.size());

            result = device->CreateShaderResourceView(
                out_buffer.Get(),
                &srv_description,
                out_srv.GetAddressOf());
            if (FAILED(result))
            {
                diagnostic = std::string("D3D11 CreateShaderResourceView failed for immutable Animation Packet ") +
                    label + " (HRESULT=" + std::to_string(
                        static_cast<uint32_t>(result)) + ").";
                out_buffer.Reset();
                return false;
            }

            return true;
        }

        bool create_mutable_cursor_buffer(
            ID3D11Device* device,
            const animation_span_cursor_gpu_record& initial_cursor,
            Microsoft::WRL::ComPtr<ID3D11Buffer>& out_buffer,
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>& out_srv,
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView>& out_uav,
            std::string& diagnostic)
        {
            out_buffer.Reset();
            out_srv.Reset();
            out_uav.Reset();

            D3D11_BUFFER_DESC buffer_description = {};
            buffer_description.ByteWidth =
                static_cast<UINT>(sizeof(animation_span_cursor_gpu_record));
            buffer_description.Usage = D3D11_USAGE_DEFAULT;
            buffer_description.BindFlags =
                D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
            buffer_description.MiscFlags =
                D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
            buffer_description.StructureByteStride =
                static_cast<UINT>(sizeof(animation_span_cursor_gpu_record));

            D3D11_SUBRESOURCE_DATA initial_data = {};
            initial_data.pSysMem = &initial_cursor;

            HRESULT result = device->CreateBuffer(
                &buffer_description,
                &initial_data,
                out_buffer.GetAddressOf());
            if (FAILED(result))
            {
                diagnostic =
                    "D3D11 CreateBuffer failed for the mutable Animation Span Cursor (HRESULT=" +
                    std::to_string(static_cast<uint32_t>(result)) + ").";
                return false;
            }

            D3D11_SHADER_RESOURCE_VIEW_DESC srv_description = {};
            srv_description.Format = DXGI_FORMAT_UNKNOWN;
            srv_description.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
            srv_description.Buffer.FirstElement = 0u;
            srv_description.Buffer.NumElements = 1u;
            result = device->CreateShaderResourceView(
                out_buffer.Get(),
                &srv_description,
                out_srv.GetAddressOf());
            if (FAILED(result))
            {
                diagnostic =
                    "D3D11 CreateShaderResourceView failed for the mutable Animation Span Cursor (HRESULT=" +
                    std::to_string(static_cast<uint32_t>(result)) + ").";
                out_buffer.Reset();
                return false;
            }

            D3D11_UNORDERED_ACCESS_VIEW_DESC uav_description = {};
            uav_description.Format = DXGI_FORMAT_UNKNOWN;
            uav_description.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
            uav_description.Buffer.FirstElement = 0u;
            uav_description.Buffer.NumElements = 1u;
            result = device->CreateUnorderedAccessView(
                out_buffer.Get(),
                &uav_description,
                out_uav.GetAddressOf());
            if (FAILED(result))
            {
                diagnostic =
                    "D3D11 CreateUnorderedAccessView failed for the mutable Animation Span Cursor (HRESULT=" +
                    std::to_string(static_cast<uint32_t>(result)) + ").";
                out_buffer.Reset();
                out_srv.Reset();
                return false;
            }

            return true;
        }

        bool valid_record_range(
            uint32_t first,
            uint32_t count,
            std::size_t total)
        {
            return static_cast<std::size_t>(first) <= total &&
                static_cast<std::size_t>(count) <=
                    total - static_cast<std::size_t>(first);
        }

        bool validate_animation_packet(
            const animation_trajectory_packet& packet,
            const animation_span_cursor_gpu_record& cursor,
            std::string& diagnostic)
        {
            diagnostic.clear();
            if (packet.animation_input_version == 0u)
            {
                diagnostic =
                    "Animation Trajectory Packet Upload requires a non-zero immutable Input Version.";
                return false;
            }
            if (packet.span_records.empty() ||
                packet.span_world_time_starts.size() != packet.span_records.size() ||
                packet.span_records.size() >
                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()))
            {
                diagnostic =
                    "Animation Trajectory Packet must contain a non-empty Span table with matching CPU World-Time metadata.";
                return false;
            }

            const uint32_t version_low = static_cast<uint32_t>(
                packet.animation_input_version & 0xffffffffull);
            const uint32_t version_high = static_cast<uint32_t>(
                packet.animation_input_version >> 32u);

            std::unordered_set<uint32_t> span_ids;
            span_ids.reserve(packet.span_records.size());
            for (std::size_t span_index = 0u;
                span_index < packet.span_records.size(); ++span_index)
            {
                const animation_span_gpu_record& span =
                    packet.span_records[span_index];
                const double world_time_start =
                    packet.span_world_time_starts[span_index];

                if (span.animation_input_version_low != version_low ||
                    span.animation_input_version_high != version_high ||
                    !span_ids.insert(span.span_id).second ||
                    !std::isfinite(world_time_start) ||
                    !std::isfinite(span.span_duration_seconds) ||
                    span.span_duration_seconds <= 0.0f ||
                    !std::isfinite(span.inverse_span_duration) ||
                    span.inverse_span_duration <= 0.0f ||
                    !std::isfinite(span.speed_bound) ||
                    span.speed_bound < 0.0f ||
                    !std::isfinite(span.acceleration_bound) ||
                    span.acceleration_bound < 0.0f ||
                    !std::isfinite(span.chord_deviation_bound) ||
                    span.chord_deviation_bound < 0.0f ||
                    !std::isfinite(span.quaternion_normalization_lower_bound) ||
                    span.quaternion_normalization_lower_bound <= 0.0f ||
                    !valid_record_range(
                        span.first_track_record,
                        span.track_record_count,
                        packet.track_records.size()) ||
                    !valid_record_range(
                        span.first_channel_record,
                        span.channel_record_count,
                        packet.channel_records.size()) ||
                    !valid_record_range(
                        span.first_blend_record,
                        span.blend_record_count,
                        packet.blend_records.size()) ||
                    !valid_record_range(
                        span.first_collider_record,
                        span.collider_record_count,
                        packet.collider_records.size()))
                {
                    diagnostic =
                        "Animation Trajectory Packet contains an invalid Span Version, ID, Bound or absolute Buffer Range.";
                    return false;
                }

                for (uint32_t local_blend_index = 0u;
                    local_blend_index < span.blend_record_count;
                    ++local_blend_index)
                {
                    const animation_blend_gpu_record& blend =
                        packet.blend_records[
                            static_cast<std::size_t>(span.first_blend_record) +
                            local_blend_index];
                    if (blend.source_track_index >= span.track_record_count ||
                        blend.target_track_index >= span.track_record_count ||
                        blend.source_track_index == blend.target_track_index)
                    {
                        diagnostic =
                            "Animation Trajectory Packet Blend Record references a Track outside its owning Span.";
                        return false;
                    }
                }
            }

            for (const animation_track_gpu_record& track : packet.track_records)
            {
                if (!valid_record_range(
                        track.first_channel_record,
                        track.channel_record_count,
                        packet.channel_records.size()) ||
                    !std::isfinite(track.clip_time_start) ||
                    !std::isfinite(track.clip_time_delta) ||
                    !std::isfinite(track.playback_speed) ||
                    !std::isfinite(
                        static_cast<double>(track.clip_time_start) +
                        track.clip_time_delta))
                {
                    diagnostic =
                        "Animation Trajectory Packet contains an invalid Track Channel Range or non-finite Clip-Time field.";
                    return false;
                }
            }

            for (const animation_channel_gpu_record& channel :
                packet.channel_records)
            {
                if (channel.node_index == asset::invalid_index ||
                    (channel.parent_index != asset::invalid_index &&
                        channel.parent_index >= packet.channel_records.size()))
                {
                    diagnostic =
                        "Animation Trajectory Packet contains an invalid Node or Parent Channel index.";
                    return false;
                }
            }

            for (const animation_blend_gpu_record& blend :
                packet.blend_records)
            {
                if (blend.source_track_index == blend.target_track_index ||
                    !std::isfinite(blend.weight_coefficients.x) ||
                    !std::isfinite(blend.weight_coefficients.y) ||
                    !std::isfinite(blend.weight_coefficients.z) ||
                    !std::isfinite(blend.weight_coefficients.w) ||
                    !std::isfinite(blend.duration_seconds) ||
                    blend.duration_seconds <= 0.0f ||
                    !std::isfinite(blend.inverse_duration) ||
                    blend.inverse_duration <= 0.0f ||
                    !std::isfinite(blend.blend_start_offset_seconds))
                {
                    diagnostic =
                        "Animation Trajectory Packet contains an invalid Blend Curve Record.";
                    return false;
                }
            }

            for (const animation_skin_gpu_record& skin : packet.skin_records)
            {
                if (!valid_record_range(
                        skin.first_joint_record,
                        skin.joint_count,
                        packet.skin_joint_records.size()) ||
                    skin.mesh_node_channel_offset == asset::invalid_index)
                {
                    diagnostic =
                        "Animation Trajectory Packet contains an invalid Skin-to-Joint Record Range.";
                    return false;
                }
            }

            for (const animation_skin_joint_gpu_record& joint :
                packet.skin_joint_records)
            {
                if (joint.joint_channel_offset == asset::invalid_index ||
                    joint.inverse_bind_record_index >=
                        packet.inverse_bind_matrices.size() ||
                    joint.joint_node_index == asset::invalid_index)
                {
                    diagnostic =
                        "Animation Trajectory Packet contains an invalid Skin Joint Channel or Inverse Bind reference.";
                    return false;
                }
            }

            for (const kinematic_collider_gpu_record& collider :
                packet.collider_records)
            {
                if (!std::isfinite(collider.duration_seconds) ||
                    collider.duration_seconds <= 0.0f ||
                    !std::isfinite(collider.inverse_duration) ||
                    collider.inverse_duration <= 0.0f ||
                    !std::isfinite(collider.linear_speed_bound) ||
                    collider.linear_speed_bound < 0.0f ||
                    !std::isfinite(collider.angular_speed_bound) ||
                    collider.angular_speed_bound < 0.0f ||
                    !std::isfinite(collider.maximum_local_radius) ||
                    collider.maximum_local_radius < 0.0f ||
                    !std::isfinite(collider.sagitta_bound) ||
                    collider.sagitta_bound < 0.0f)
                {
                    diagnostic =
                        "Animation Trajectory Packet contains an invalid Kinematic Collider duration or certified Bound.";
                    return false;
                }
            }

            if (cursor.span_record_index >= packet.span_records.size() ||
                cursor.animation_input_version_low != version_low ||
                cursor.animation_input_version_high != version_high ||
                !std::isfinite(cursor.u_segment_start) ||
                cursor.u_segment_start < 0.0f ||
                cursor.u_segment_start > 1.0f)
            {
                diagnostic =
                    "Initial Animation Span Cursor must reference this Packet Version and one valid normalized Span coordinate.";
                return false;
            }

            return true;
        }

        template<typename T>
        void assign_buffer_count(
            const std::vector<T>& values,
            uint32_t& out_count)
        {
            out_count = static_cast<uint32_t>(values.size());
        }
    }

    bool animation_trajectory_gpu_resources::valid() const noexcept
    {
        return _animation_input_version != 0u &&
            _span_buffer != nullptr &&
            _span_srv != nullptr &&
            _cursor_buffer != nullptr &&
            _cursor_srv != nullptr &&
            _cursor_uav != nullptr &&
            _counts.spans != 0u;
    }

    uint64_t animation_trajectory_gpu_resources::animation_input_version() const noexcept
    {
        return _animation_input_version;
    }

    const animation_trajectory_gpu_buffer_counts&
        animation_trajectory_gpu_resources::counts() const noexcept
    {
        return _counts;
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::span_buffer() const noexcept
    {
        return _span_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::cursor_buffer() const noexcept
    {
        return _cursor_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::track_buffer() const noexcept
    {
        return _track_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::channel_buffer() const noexcept
    {
        return _channel_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::blend_buffer() const noexcept
    {
        return _blend_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::inverse_bind_matrix_buffer() const noexcept
    {
        return _inverse_bind_matrix_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::static_node_matrix_buffer() const noexcept
    {
        return _static_node_matrix_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::skin_buffer() const noexcept
    {
        return _skin_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::skin_joint_buffer() const noexcept
    {
        return _skin_joint_buffer.Get();
    }

    ID3D11Buffer* animation_trajectory_gpu_resources::collider_buffer() const noexcept
    {
        return _collider_buffer.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::span_srv() const noexcept
    {
        return _span_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::cursor_srv() const noexcept
    {
        return _cursor_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::track_srv() const noexcept
    {
        return _track_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::channel_srv() const noexcept
    {
        return _channel_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::blend_srv() const noexcept
    {
        return _blend_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::inverse_bind_matrix_srv() const noexcept
    {
        return _inverse_bind_matrix_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::static_node_matrix_srv() const noexcept
    {
        return _static_node_matrix_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::skin_srv() const noexcept
    {
        return _skin_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::skin_joint_srv() const noexcept
    {
        return _skin_joint_srv.Get();
    }

    ID3D11ShaderResourceView* animation_trajectory_gpu_resources::collider_srv() const noexcept
    {
        return _collider_srv.Get();
    }

    ID3D11UnorderedAccessView*
        animation_trajectory_gpu_resources::cursor_uav() const noexcept
    {
        return _cursor_uav.Get();
    }

    void animation_trajectory_gpu_resources::reset() noexcept
    {
        _animation_input_version = 0u;
        _counts = {};

        _span_buffer.Reset();
        _cursor_buffer.Reset();
        _track_buffer.Reset();
        _channel_buffer.Reset();
        _blend_buffer.Reset();
        _inverse_bind_matrix_buffer.Reset();
        _static_node_matrix_buffer.Reset();
        _skin_buffer.Reset();
        _skin_joint_buffer.Reset();
        _collider_buffer.Reset();

        _span_srv.Reset();
        _cursor_srv.Reset();
        _track_srv.Reset();
        _channel_srv.Reset();
        _blend_srv.Reset();
        _inverse_bind_matrix_srv.Reset();
        _static_node_matrix_srv.Reset();
        _skin_srv.Reset();
        _skin_joint_srv.Reset();
        _collider_srv.Reset();
        _cursor_uav.Reset();
    }

    bool upload_animation_trajectory_packet(
        ID3D11Device* device,
        const animation_trajectory_packet& packet,
        const animation_span_cursor_gpu_record& initial_cursor,
        animation_trajectory_gpu_resources& out_resources,
        std::string& diagnostic)
    {
        diagnostic.clear();
        if (device == nullptr)
        {
            diagnostic = "Animation Trajectory Packet Upload requires a valid D3D11 Device.";
            return false;
        }
        if (!validate_animation_packet(packet, initial_cursor, diagnostic))
        {
            return false;
        }

        if (!valid_structured_buffer_size(
                packet.span_records, "Span", diagnostic) ||
            !valid_structured_buffer_size(
                packet.track_records, "Track", diagnostic) ||
            !valid_structured_buffer_size(
                packet.channel_records, "Channel", diagnostic) ||
            !valid_structured_buffer_size(
                packet.blend_records, "Blend", diagnostic) ||
            !valid_structured_buffer_size(
                packet.inverse_bind_matrices, "Inverse Bind Matrix", diagnostic) ||
            !valid_structured_buffer_size(
                packet.static_node_matrices, "Static Node Matrix", diagnostic) ||
            !valid_structured_buffer_size(
                packet.skin_records, "Skin", diagnostic) ||
            !valid_structured_buffer_size(
                packet.skin_joint_records, "Skin Joint", diagnostic) ||
            !valid_structured_buffer_size(
                packet.collider_records, "Collider", diagnostic))
        {
            return false;
        }

        animation_trajectory_gpu_resources candidate;
        candidate._animation_input_version = packet.animation_input_version;
        assign_buffer_count(packet.span_records, candidate._counts.spans);
        assign_buffer_count(packet.track_records, candidate._counts.tracks);
        assign_buffer_count(packet.channel_records, candidate._counts.channels);
        assign_buffer_count(packet.blend_records, candidate._counts.blends);
        assign_buffer_count(
            packet.inverse_bind_matrices,
            candidate._counts.inverse_bind_matrices);
        assign_buffer_count(
            packet.static_node_matrices,
            candidate._counts.static_node_matrices);
        assign_buffer_count(packet.skin_records, candidate._counts.skins);
        assign_buffer_count(
            packet.skin_joint_records,
            candidate._counts.skin_joints);
        assign_buffer_count(packet.collider_records, candidate._counts.colliders);

        if (!create_immutable_structured_buffer(
                device, packet.span_records, "Span",
                candidate._span_buffer, candidate._span_srv, diagnostic) ||
            !create_immutable_structured_buffer(
                device, packet.track_records, "Track",
                candidate._track_buffer, candidate._track_srv, diagnostic) ||
            !create_immutable_structured_buffer(
                device, packet.channel_records, "Channel",
                candidate._channel_buffer, candidate._channel_srv, diagnostic) ||
            !create_immutable_structured_buffer(
                device, packet.blend_records, "Blend",
                candidate._blend_buffer, candidate._blend_srv, diagnostic) ||
            !create_immutable_structured_buffer(
                device, packet.inverse_bind_matrices, "Inverse Bind Matrix",
                candidate._inverse_bind_matrix_buffer,
                candidate._inverse_bind_matrix_srv, diagnostic) ||
            !create_immutable_structured_buffer(
                device, packet.static_node_matrices, "Static Node Matrix",
                candidate._static_node_matrix_buffer,
                candidate._static_node_matrix_srv, diagnostic) ||
            !create_immutable_structured_buffer(
                device, packet.skin_records, "Skin",
                candidate._skin_buffer, candidate._skin_srv, diagnostic) ||
            !create_immutable_structured_buffer(
                device, packet.skin_joint_records, "Skin Joint",
                candidate._skin_joint_buffer, candidate._skin_joint_srv, diagnostic) ||
            !create_immutable_structured_buffer(
                device, packet.collider_records, "Collider",
                candidate._collider_buffer, candidate._collider_srv, diagnostic) ||
            !create_mutable_cursor_buffer(
                device, initial_cursor,
                candidate._cursor_buffer,
                candidate._cursor_srv,
                candidate._cursor_uav,
                diagnostic))
        {
            return false;
        }

        if (!candidate.valid())
        {
            diagnostic =
                "Animation Trajectory Packet Upload produced an incomplete GPU resource bundle.";
            return false;
        }

        out_resources = std::move(candidate);
        return true;
    }
}
