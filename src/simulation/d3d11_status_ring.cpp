#include "skin_physics/simulation/d3d11_status_ring.hpp"

#include <cstring>
#include <utility>

namespace skin_physics::simulation
{
    namespace
    {
        bool same_com_identity(IUnknown* left, IUnknown* right)
        {
            if (left == nullptr || right == nullptr)
            {
                return false;
            }

            Microsoft::WRL::ComPtr<IUnknown> left_identity;
            Microsoft::WRL::ComPtr<IUnknown> right_identity;
            if (FAILED(left->QueryInterface(
                    IID_PPV_ARGS(left_identity.GetAddressOf()))) ||
                FAILED(right->QueryInterface(
                    IID_PPV_ARGS(right_identity.GetAddressOf()))))
            {
                return false;
            }
            return left_identity.Get() == right_identity.Get();
        }

        bool same_key(
            const d3d11_status_query_key& left,
            const d3d11_status_query_key& right)
        {
            return left.world_id == right.world_id &&
                left.tick_sequence == right.tick_sequence &&
                left.dispatch_id == right.dispatch_id;
        }

        bool is_device_removed(HRESULT result)
        {
            return result == DXGI_ERROR_DEVICE_REMOVED ||
                result == DXGI_ERROR_DEVICE_RESET;
        }
    }

    bool d3d11_status_ring::initialize(
        ID3D11Device* device,
        ID3D11DeviceContext* immediate_context,
        uint32_t slot_count,
        uint32_t status_byte_width,
        std::string& diagnostic)
    {
        if (_valid || device == nullptr || immediate_context == nullptr ||
            slot_count < 2 || status_byte_width < sizeof(uint32_t) ||
            (status_byte_width & 3u) != 0u)
        {
            diagnostic = "Status Ring requires an unused instance, a D3D11 device, an Immediate Context, at least two slots, and a DWORD-aligned non-empty Status buffer size.";
            return false;
        }

        if (device->GetFeatureLevel() < D3D_FEATURE_LEVEL_11_0)
        {
            diagnostic = "Status Ring requires D3D11 Feature Level 11_0 or newer.";
            return false;
        }
        if (immediate_context->GetType() != D3D11_DEVICE_CONTEXT_IMMEDIATE)
        {
            diagnostic = "Status Ring must use the Host-owned Immediate Context; Deferred Contexts are unsupported.";
            return false;
        }

        Microsoft::WRL::ComPtr<ID3D11Device> context_device;
        immediate_context->GetDevice(context_device.GetAddressOf());
        if (!same_com_identity(device, context_device.Get()))
        {
            diagnostic = "Status Ring Device and Immediate Context belong to different D3D11 Devices.";
            return false;
        }

        const HRESULT removed_reason = device->GetDeviceRemovedReason();
        if (FAILED(removed_reason))
        {
            diagnostic = "D3D11 Device is already removed; Status Ring initialization was rejected.";
            return false;
        }

        _device = device;
        _context = immediate_context;
        _status_byte_width = status_byte_width;
        _submission_thread = std::this_thread::get_id();
        _slots.resize(slot_count);

        D3D11_BUFFER_DESC staging_description = {};
        staging_description.ByteWidth = status_byte_width;
        staging_description.Usage = D3D11_USAGE_STAGING;
        staging_description.BindFlags = 0;
        staging_description.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        staging_description.MiscFlags = 0;
        staging_description.StructureByteStride = 0;

        D3D11_QUERY_DESC query_description = {};
        query_description.Query = D3D11_QUERY_EVENT;
        query_description.MiscFlags = 0;

        for (slot& entry : _slots)
        {
            HRESULT result = _device->CreateBuffer(
                &staging_description,
                nullptr,
                entry.staging_buffer.GetAddressOf());
            if (FAILED(result))
            {
                diagnostic = "Status Ring staging buffer creation failed with HRESULT " +
                    std::to_string(static_cast<unsigned long>(result)) + ".";
                _slots.clear();
                _context.Reset();
                _device.Reset();
                _status_byte_width = 0;
                return false;
            }

            result = _device->CreateQuery(
                &query_description,
                entry.completion_query.GetAddressOf());
            if (FAILED(result))
            {
                diagnostic = "Status Ring Event Query creation failed with HRESULT " +
                    std::to_string(static_cast<unsigned long>(result)) + ".";
                _slots.clear();
                _context.Reset();
                _device.Reset();
                _status_byte_width = 0;
                return false;
            }
        }

        _valid = true;
        _diagnostic.clear();
        diagnostic.clear();
        return true;
    }

    bool d3d11_status_ring::on_submission_thread() const
    {
        return _submission_thread == std::this_thread::get_id();
    }

    bool d3d11_status_ring::same_device(ID3D11Device* device) const
    {
        return same_com_identity(_device.Get(), device);
    }

    d3d11_status_ring_error d3d11_status_ring::enqueue(
        const d3d11_status_query_key& key,
        ID3D11Buffer* gpu_status_buffer,
        std::string& diagnostic)
    {
        diagnostic.clear();
        if (!_valid)
        {
            diagnostic = "Status Ring is not initialized.";
            return d3d11_status_ring_error::not_initialized;
        }
        if (!on_submission_thread())
        {
            diagnostic = "Status Ring enqueue must run on its registered Immediate Context submission thread.";
            return d3d11_status_ring_error::submission_thread_violation;
        }
        if (gpu_status_buffer == nullptr ||
            key.tick_sequence == 0 ||
            key.dispatch_id == 0)
        {
            diagnostic = "Status Ring enqueue requires a GPU Status Buffer and non-zero Tick Sequence / Dispatch ID.";
            return d3d11_status_ring_error::invalid_argument;
        }

        D3D11_BUFFER_DESC source_description = {};
        gpu_status_buffer->GetDesc(&source_description);
        if (source_description.Usage != D3D11_USAGE_DEFAULT ||
            source_description.CPUAccessFlags != 0 ||
            source_description.ByteWidth != _status_byte_width)
        {
            diagnostic = "GPU Status Buffer must be a D3D11_USAGE_DEFAULT buffer with no CPU access and the configured exact ByteWidth.";
            return d3d11_status_ring_error::invalid_source_buffer;
        }

        Microsoft::WRL::ComPtr<ID3D11Device> source_device;
        gpu_status_buffer->GetDevice(source_device.GetAddressOf());
        if (!same_device(source_device.Get()))
        {
            diagnostic = "GPU Status Buffer belongs to a different D3D11 Device.";
            return d3d11_status_ring_error::invalid_source_buffer;
        }

        for (std::size_t slot_index : _submission_order)
        {
            if (same_key(_slots[slot_index].key, key))
            {
                diagnostic = "A Status Query with the same World / Tick / Dispatch key is already pending.";
                return d3d11_status_ring_error::duplicate_query_key;
            }
        }

        if (_submission_order.size() >= _slots.size())
        {
            diagnostic = "All Status Ring slots are awaiting non-blocking completion; no slot was overwritten.";
            return d3d11_status_ring_error::capacity_full;
        }

        std::size_t free_index = _slots.size();
        for (std::size_t index = 0; index < _slots.size(); ++index)
        {
            if (!_slots[index].pending)
            {
                free_index = index;
                break;
            }
        }
        if (free_index >= _slots.size())
        {
            diagnostic = "Status Ring bookkeeping found no free slot despite available queue capacity.";
            return d3d11_status_ring_error::capacity_full;
        }

        slot& entry = _slots[free_index];
        const HRESULT removed_reason = _device->GetDeviceRemovedReason();
        if (FAILED(removed_reason))
        {
            diagnostic = "D3D11 Device was removed before Status Copy / Event Query submission.";
            return d3d11_status_ring_error::device_removed;
        }

        _context->CopyResource(
            entry.staging_buffer.Get(),
            gpu_status_buffer);
        _context->End(entry.completion_query.Get());

        entry.key = key;
        entry.pending = true;
        _submission_order.push_back(free_index);
        return d3d11_status_ring_error::none;
    }

    d3d11_status_ring_poll_result d3d11_status_ring::make_poll_failure(
        d3d11_status_ring_error error,
        const std::string& diagnostic) const
    {
        d3d11_status_ring_poll_result result;
        result.error = error;
        result.pending_count = _submission_order.size();
        result.diagnostic = diagnostic;
        return result;
    }

    d3d11_status_ring_poll_result d3d11_status_ring::poll()
    {
        if (!_valid)
        {
            return make_poll_failure(
                d3d11_status_ring_error::not_initialized,
                "Status Ring is not initialized.");
        }
        if (!on_submission_thread())
        {
            return make_poll_failure(
                d3d11_status_ring_error::submission_thread_violation,
                "Status Ring poll must run on its registered Immediate Context submission thread.");
        }

        const HRESULT removed_reason = _device->GetDeviceRemovedReason();
        if (FAILED(removed_reason))
        {
            return make_poll_failure(
                d3d11_status_ring_error::device_removed,
                "D3D11 Device was removed while polling Status Queries.");
        }

        d3d11_status_ring_poll_result result;
        while (!_submission_order.empty())
        {
            const std::size_t slot_index = _submission_order.front();
            slot& entry = _slots[slot_index];

            BOOL event_complete = FALSE;
            const HRESULT query_result = _context->GetData(
                entry.completion_query.Get(),
                &event_complete,
                sizeof(event_complete),
                D3D11_ASYNC_GETDATA_DONOTFLUSH);

            if (query_result == S_FALSE ||
                (SUCCEEDED(query_result) && event_complete == FALSE))
            {
                break;
            }
            if (FAILED(query_result))
            {
                const HRESULT device_reason = _device->GetDeviceRemovedReason();
                return make_poll_failure(
                    is_device_removed(query_result) || FAILED(device_reason)
                        ? d3d11_status_ring_error::device_removed
                        : d3d11_status_ring_error::query_failed,
                    "D3D11 Event Query GetData failed before Status Readback.");
            }

            D3D11_MAPPED_SUBRESOURCE mapped = {};
            const HRESULT map_result = _context->Map(
                entry.staging_buffer.Get(),
                0,
                D3D11_MAP_READ,
                D3D11_MAP_FLAG_DO_NOT_WAIT,
                &mapped);

            if (map_result == DXGI_ERROR_WAS_STILL_DRAWING)
            {
                break;
            }
            if (FAILED(map_result))
            {
                const HRESULT device_reason = _device->GetDeviceRemovedReason();
                return make_poll_failure(
                    is_device_removed(map_result) || FAILED(device_reason)
                        ? d3d11_status_ring_error::device_removed
                        : d3d11_status_ring_error::map_failed,
                    "D3D11 non-blocking Map failed for a completed Status Event Query.");
            }

            d3d11_status_readback readback;
            readback.key = entry.key;
            readback.bytes.resize(_status_byte_width);
            std::memcpy(
                readback.bytes.data(),
                mapped.pData,
                _status_byte_width);
            _context->Unmap(entry.staging_buffer.Get(), 0);

            result.completed.push_back(std::move(readback));
            entry.pending = false;
            entry.key = {};
            _submission_order.pop_front();
        }

        result.pending_count = _submission_order.size();
        return result;
    }

    std::size_t d3d11_status_ring::pending_count() const
    {
        return _submission_order.size();
    }

    uint32_t d3d11_status_ring::capacity() const
    {
        return static_cast<uint32_t>(_slots.size());
    }

    uint32_t d3d11_status_ring::status_byte_width() const
    {
        return _status_byte_width;
    }

    bool d3d11_status_ring::valid() const
    {
        return _valid;
    }
}
