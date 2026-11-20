#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <thread>
#include <vector>

#include <d3d11.h>
#include <wrl/client.h>

namespace skin_physics::simulation
{
    struct d3d11_status_query_key
    {
        uint64_t world_id = 0;
        uint64_t tick_sequence = 0;
        uint64_t dispatch_id = 0;
    };

    struct d3d11_status_readback
    {
        d3d11_status_query_key key;
        std::vector<uint8_t> bytes;
    };

    enum class d3d11_status_ring_error : uint8_t
    {
        none,
        invalid_argument,
        not_initialized,
        submission_thread_violation,
        capacity_full,
        duplicate_query_key,
        invalid_source_buffer,
        device_removed,
        query_failed,
        map_failed
    };

    struct d3d11_status_ring_poll_result
    {
        d3d11_status_ring_error error = d3d11_status_ring_error::none;
        std::vector<d3d11_status_readback> completed;
        std::size_t pending_count = 0;
        std::string diagnostic;

        bool success() const
        {
            return error == d3d11_status_ring_error::none;
        }
    };

    // A FIFO, non-blocking Status Readback ring tied to one D3D11 Immediate Context thread.
    // A slot is not reusable until its Event Query completed and its staging buffer was mapped.
    class d3d11_status_ring
    {
    public:
        d3d11_status_ring() = default;
        d3d11_status_ring(const d3d11_status_ring&) = delete;
        d3d11_status_ring& operator=(const d3d11_status_ring&) = delete;

        bool initialize(
            ID3D11Device* device,
            ID3D11DeviceContext* immediate_context,
            uint32_t slot_count,
            uint32_t status_byte_width,
            std::string& diagnostic);

        d3d11_status_ring_error enqueue(
            const d3d11_status_query_key& key,
            ID3D11Buffer* gpu_status_buffer,
            std::string& diagnostic);

        d3d11_status_ring_poll_result poll();

        std::size_t pending_count() const;
        uint32_t capacity() const;
        uint32_t status_byte_width() const;
        bool valid() const;

    private:
        struct slot
        {
            Microsoft::WRL::ComPtr<ID3D11Buffer> staging_buffer;
            Microsoft::WRL::ComPtr<ID3D11Query> completion_query;
            d3d11_status_query_key key;
            bool pending = false;
        };

        Microsoft::WRL::ComPtr<ID3D11Device> _device;
        Microsoft::WRL::ComPtr<ID3D11DeviceContext> _context;
        std::vector<slot> _slots;
        std::deque<std::size_t> _submission_order;
        std::thread::id _submission_thread;
        uint32_t _status_byte_width = 0;
        bool _valid = false;
        std::string _diagnostic;

        bool on_submission_thread() const;
        bool same_device(ID3D11Device* device) const;
        d3d11_status_ring_poll_result make_poll_failure(
            d3d11_status_ring_error error,
            const std::string& diagnostic) const;
    };
}
