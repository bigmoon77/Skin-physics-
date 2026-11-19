#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <d3d11.h>
#include <dxgi1_4.h>
#include <wrl/client.h>

namespace skin_physics::simulation
{
    struct gpu_pass_timing
    {
        std::string pass_name;
        double milliseconds = 0.0;
    };

    enum class timestamp_collection_status : uint8_t
    {
        not_ready,
        collected,
        invalid_data,
        device_error
    };

    struct timestamp_collection_result
    {
        timestamp_collection_status status = timestamp_collection_status::not_ready;
        std::vector<gpu_pass_timing> pass_timings;
        bool timestamp_disjoint = false;
        std::string diagnostic;
    };

    struct video_memory_info
    {
        uint64_t current_usage_bytes = 0;
        uint64_t budget_bytes = 0;
        bool available = false;
        std::string diagnostic;
    };

    class d3d11_timestamp_profiler
    {
    public:
        d3d11_timestamp_profiler(
            ID3D11Device* device,
            ID3D11DeviceContext* immediate_context,
            uint32_t frame_slot_count = 4);

        bool valid() const;
        const std::string& diagnostic() const;

        bool register_pass(const std::string& pass_name);
        bool begin_frame();
        bool begin_pass(const std::string& pass_name);
        bool end_pass(const std::string& pass_name);
        bool end_frame();

        timestamp_collection_result try_collect();

    private:
        struct pass_query_pair
        {
            Microsoft::WRL::ComPtr<ID3D11Query> begin;
            Microsoft::WRL::ComPtr<ID3D11Query> end;
        };

        struct frame_slot
        {
            Microsoft::WRL::ComPtr<ID3D11Query> disjoint;
            std::map<std::string, pass_query_pair> passes;
            std::vector<std::string> recorded_passes;
            std::string active_pass;
            bool active = false;
            bool pending = false;
        };

        Microsoft::WRL::ComPtr<ID3D11Device> _device;
        Microsoft::WRL::ComPtr<ID3D11DeviceContext> _context;
        std::vector<frame_slot> _slots;
        std::size_t _active_slot = static_cast<std::size_t>(-1);
        std::size_t _next_slot = 0;
        std::string _diagnostic;
        bool _valid = false;

        bool create_query(D3D11_QUERY query_type, ID3D11Query** output);
    };

    video_memory_info query_local_video_memory(IDXGIAdapter* adapter);
}
