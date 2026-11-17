#include "skin_physics/simulation/d3d11_timestamp_profiler.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace skin_physics::simulation
{
    d3d11_timestamp_profiler::d3d11_timestamp_profiler(
        ID3D11Device* device,
        ID3D11DeviceContext* immediate_context,
        uint32_t frame_slot_count)
    {
        if (device == nullptr || immediate_context == nullptr || frame_slot_count < 2)
        {
            _diagnostic = "Timestamp profiler requires a D3D11 device, immediate context and at least two frame slots.";
            return;
        }

        _device = device;
        _context = immediate_context;
        _slots.resize(frame_slot_count);
        for (frame_slot& slot : _slots)
        {
            if (!create_query(D3D11_QUERY_TIMESTAMP_DISJOINT, slot.disjoint.GetAddressOf()))
            {
                _slots.clear();
                return;
            }
        }
        _valid = true;
    }

    bool d3d11_timestamp_profiler::valid() const
    {
        return _valid;
    }

    const std::string& d3d11_timestamp_profiler::diagnostic() const
    {
        return _diagnostic;
    }

    bool d3d11_timestamp_profiler::create_query(
        D3D11_QUERY query_type,
        ID3D11Query** output)
    {
        D3D11_QUERY_DESC description = {};
        description.Query = query_type;
        description.MiscFlags = 0;
        const HRESULT result = _device->CreateQuery(&description, output);
        if (FAILED(result))
        {
            _diagnostic = "D3D11 CreateQuery failed with HRESULT " +
                std::to_string(static_cast<unsigned long>(result)) + ".";
            return false;
        }
        return true;
    }

    bool d3d11_timestamp_profiler::register_pass(const std::string& pass_name)
    {
        if (!_valid || pass_name.empty() || _active_slot != static_cast<std::size_t>(-1))
        {
            _diagnostic = "Pass registration requires a valid idle profiler and a non-empty name.";
            return false;
        }

        for (frame_slot& slot : _slots)
        {
            if (slot.passes.find(pass_name) != slot.passes.end())
            {
                _diagnostic = "Timestamp pass name is already registered.";
                return false;
            }

            pass_query_pair pair;
            if (!create_query(D3D11_QUERY_TIMESTAMP, pair.begin.GetAddressOf()) ||
                !create_query(D3D11_QUERY_TIMESTAMP, pair.end.GetAddressOf()))
            {
                return false;
            }
            slot.passes.emplace(pass_name, std::move(pair));
        }
        return true;
    }

    bool d3d11_timestamp_profiler::begin_frame()
    {
        if (!_valid || _active_slot != static_cast<std::size_t>(-1))
        {
            _diagnostic = "Cannot begin a frame while the profiler is invalid or another frame is active.";
            return false;
        }

        for (std::size_t attempt = 0; attempt < _slots.size(); ++attempt)
        {
            const std::size_t index = (_next_slot + attempt) % _slots.size();
            frame_slot& slot = _slots[index];
            if (slot.pending || slot.active)
            {
                continue;
            }
            slot.recorded_passes.clear();
            slot.active_pass.clear();
            _context->Begin(slot.disjoint.Get());
            slot.active = true;
            _active_slot = index;
            _next_slot = (index + 1) % _slots.size();
            return true;
        }

        _diagnostic = "All timestamp frame slots are pending collection; refusing to overwrite them.";
        return false;
    }

    bool d3d11_timestamp_profiler::begin_pass(const std::string& pass_name)
    {
        if (_active_slot >= _slots.size())
        {
            _diagnostic = "Cannot begin a timestamp pass outside an active frame.";
            return false;
        }
        frame_slot& slot = _slots[_active_slot];
        if (!slot.active || !slot.active_pass.empty())
        {
            _diagnostic = "Timestamp pass nesting is not supported.";
            return false;
        }
        const auto found = slot.passes.find(pass_name);
        if (found == slot.passes.end())
        {
            _diagnostic = "Timestamp pass name has not been registered.";
            return false;
        }
        for (const std::string& recorded : slot.recorded_passes)
        {
            if (recorded == pass_name)
            {
                _diagnostic = "A timestamp pass may execute at most once per frame.";
                return false;
            }
        }
        _context->End(found->second.begin.Get());
        slot.active_pass = pass_name;
        return true;
    }

    bool d3d11_timestamp_profiler::end_pass(const std::string& pass_name)
    {
        if (_active_slot >= _slots.size())
        {
            _diagnostic = "Cannot end a timestamp pass outside an active frame.";
            return false;
        }
        frame_slot& slot = _slots[_active_slot];
        if (!slot.active || slot.active_pass != pass_name)
        {
            _diagnostic = "Timestamp pass end does not match the active pass.";
            return false;
        }
        const auto found = slot.passes.find(pass_name);
        if (found == slot.passes.end())
        {
            _diagnostic = "Active timestamp pass disappeared from registration.";
            return false;
        }
        _context->End(found->second.end.Get());
        slot.recorded_passes.push_back(pass_name);
        slot.active_pass.clear();
        return true;
    }

    bool d3d11_timestamp_profiler::end_frame()
    {
        if (_active_slot >= _slots.size())
        {
            _diagnostic = "Cannot end a timestamp frame when no frame is active.";
            return false;
        }
        frame_slot& slot = _slots[_active_slot];
        if (!slot.active || !slot.active_pass.empty())
        {
            _diagnostic = "All timestamp passes must be ended before ending the frame.";
            return false;
        }
        _context->End(slot.disjoint.Get());
        slot.active = false;
        slot.pending = true;
        _active_slot = static_cast<std::size_t>(-1);
        return true;
    }

    timestamp_collection_result d3d11_timestamp_profiler::try_collect()
    {
        timestamp_collection_result result;
        if (!_valid)
        {
            result.status = timestamp_collection_status::device_error;
            result.diagnostic = _diagnostic;
            return result;
        }

        for (frame_slot& slot : _slots)
        {
            if (!slot.pending)
            {
                continue;
            }

            D3D11_QUERY_DATA_TIMESTAMP_DISJOINT disjoint = {};
            HRESULT get_result = _context->GetData(
                slot.disjoint.Get(), &disjoint, sizeof(disjoint),
                D3D11_ASYNC_GETDATA_DONOTFLUSH);
            if (get_result == S_FALSE)
            {
                continue;
            }
            if (FAILED(get_result))
            {
                result.status = timestamp_collection_status::device_error;
                result.diagnostic = "GetData failed for timestamp disjoint query.";
                return result;
            }

            if (disjoint.Disjoint || disjoint.Frequency == 0)
            {
                slot.pending = false;
                result.status = timestamp_collection_status::invalid_data;
                result.timestamp_disjoint = disjoint.Disjoint != FALSE;
                result.diagnostic = "GPU timestamps were disjoint or had a zero frequency.";
                return result;
            }

            std::vector<gpu_pass_timing> timings;
            timings.reserve(slot.recorded_passes.size());
            bool all_ready = true;
            for (const std::string& pass_name : slot.recorded_passes)
            {
                const pass_query_pair& pair = slot.passes.at(pass_name);
                uint64_t begin_timestamp = 0;
                uint64_t end_timestamp = 0;
                const HRESULT begin_result = _context->GetData(
                    pair.begin.Get(), &begin_timestamp, sizeof(begin_timestamp),
                    D3D11_ASYNC_GETDATA_DONOTFLUSH);
                const HRESULT end_result = _context->GetData(
                    pair.end.Get(), &end_timestamp, sizeof(end_timestamp),
                    D3D11_ASYNC_GETDATA_DONOTFLUSH);
                if (begin_result == S_FALSE || end_result == S_FALSE)
                {
                    all_ready = false;
                    break;
                }
                if (FAILED(begin_result) || FAILED(end_result))
                {
                    result.status = timestamp_collection_status::device_error;
                    result.diagnostic = "GetData failed for a pass timestamp query.";
                    return result;
                }
                if (end_timestamp < begin_timestamp)
                {
                    result.status = timestamp_collection_status::invalid_data;
                    result.diagnostic = "GPU timestamp order is invalid.";
                    return result;
                }

                gpu_pass_timing timing;
                timing.pass_name = pass_name;
                timing.milliseconds =
                    static_cast<double>(end_timestamp - begin_timestamp) * 1000.0 /
                    static_cast<double>(disjoint.Frequency);
                if (!std::isfinite(timing.milliseconds) || timing.milliseconds < 0.0)
                {
                    result.status = timestamp_collection_status::invalid_data;
                    result.diagnostic = "GPU pass timestamp conversion produced an invalid duration.";
                    return result;
                }
                timings.push_back(std::move(timing));
            }

            if (!all_ready)
            {
                continue;
            }

            slot.pending = false;
            result.status = timestamp_collection_status::collected;
            result.pass_timings = std::move(timings);
            return result;
        }

        return result;
    }

    video_memory_info query_local_video_memory(IDXGIAdapter* adapter)
    {
        video_memory_info result;
        if (adapter == nullptr)
        {
            result.diagnostic = "DXGI adapter was not supplied.";
            return result;
        }

        Microsoft::WRL::ComPtr<IDXGIAdapter3> adapter3;
        const HRESULT query_result = adapter->QueryInterface(
            IID_PPV_ARGS(adapter3.GetAddressOf()));
        if (FAILED(query_result))
        {
            result.diagnostic = "IDXGIAdapter3 is unavailable on this adapter.";
            return result;
        }

        DXGI_QUERY_VIDEO_MEMORY_INFO memory_info = {};
        const HRESULT memory_result = adapter3->QueryVideoMemoryInfo(
            0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &memory_info);
        if (FAILED(memory_result))
        {
            result.diagnostic = "QueryVideoMemoryInfo failed for local video memory.";
            return result;
        }

        result.current_usage_bytes = memory_info.CurrentUsage;
        result.budget_bytes = memory_info.Budget;
        result.available = true;
        return result;
    }
}
