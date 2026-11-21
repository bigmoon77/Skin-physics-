#include "skin_physics/simulation/d3d11_compute_graph.hpp"

#include <array>
#include <set>
#include <string>
#include <utility>

namespace skin_physics::simulation
{
    namespace
    {
        bool same_resource(ID3D11Resource* left, ID3D11Resource* right)
        {
            if (left == nullptr || right == nullptr)
            {
                return false;
            }

            Microsoft::WRL::ComPtr<IUnknown> left_identity;
            Microsoft::WRL::ComPtr<IUnknown> right_identity;
            if (FAILED(left->QueryInterface(IID_PPV_ARGS(left_identity.GetAddressOf()))) ||
                FAILED(right->QueryInterface(IID_PPV_ARGS(right_identity.GetAddressOf()))))
            {
                return false;
            }
            return left_identity.Get() == right_identity.Get();
        }

        bool same_resource(ID3D11View* left, ID3D11View* right)
        {
            if (left == nullptr || right == nullptr)
            {
                return false;
            }

            Microsoft::WRL::ComPtr<ID3D11Resource> left_resource;
            Microsoft::WRL::ComPtr<ID3D11Resource> right_resource;
            left->GetResource(left_resource.GetAddressOf());
            right->GetResource(right_resource.GetAddressOf());
            return same_resource(left_resource.Get(), right_resource.Get());
        }

        bool same_resource(ID3D11Buffer* buffer, ID3D11View* view)
        {
            if (buffer == nullptr || view == nullptr)
            {
                return false;
            }

            Microsoft::WRL::ComPtr<ID3D11Resource> view_resource;
            view->GetResource(view_resource.GetAddressOf());
            return same_resource(buffer, view_resource.Get());
        }

        void unbind_compute_resources(ID3D11DeviceContext* context)
        {
            std::array<ID3D11ShaderResourceView*,
                D3D11_COMMONSHADER_INPUT_RESOURCE_SLOT_COUNT> null_srvs = {};
            std::array<ID3D11UnorderedAccessView*,
                D3D11_PS_CS_UAV_REGISTER_COUNT> null_uavs = {};
            std::array<ID3D11Buffer*,
                D3D11_COMMONSHADER_CONSTANT_BUFFER_API_SLOT_COUNT> null_buffers = {};

            context->CSSetShaderResources(
                0, static_cast<UINT>(null_srvs.size()), null_srvs.data());
            context->CSSetUnorderedAccessViews(
                0, static_cast<UINT>(null_uavs.size()), null_uavs.data(), nullptr);
            context->CSSetConstantBuffers(
                0, static_cast<UINT>(null_buffers.size()), null_buffers.data());
            context->CSSetShader(nullptr, nullptr, 0);
        }
    }

    bool d3d11_compute_graph::add_pass(
        const compute_pass_desc& description,
        std::string& diagnostic)
    {
        diagnostic.clear();
        if (description.name.empty())
        {
            diagnostic = "Compute pass name must not be empty.";
            return false;
        }
        if (description.shader == nullptr)
        {
            diagnostic = "Compute pass requires a non-null shader.";
            return false;
        }
        if (description.indirect_arguments == nullptr &&
            description.indirect_argument_offset != 0)
        {
            diagnostic = "A DispatchIndirect argument offset requires an argument buffer.";
            return false;
        }
        if (description.indirect_arguments != nullptr)
        {
            D3D11_BUFFER_DESC argument_description = {};
            description.indirect_arguments->GetDesc(&argument_description);
            const UINT argument_size = sizeof(UINT) * 3u;
            if ((argument_description.MiscFlags &
                    D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS) == 0)
            {
                diagnostic = "DispatchIndirect argument buffer must use D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS.";
                return false;
            }
            if ((description.indirect_argument_offset & 3u) != 0u ||
                description.indirect_argument_offset > argument_description.ByteWidth ||
                argument_description.ByteWidth - description.indirect_argument_offset < argument_size)
            {
                diagnostic = "DispatchIndirect requires a DWORD-aligned offset with twelve readable argument bytes.";
                return false;
            }
        }
        if (description.indirect_arguments == nullptr &&
            (description.group_count_x == 0 ||
                description.group_count_y == 0 ||
                description.group_count_z == 0 ||
                description.group_count_x > D3D11_CS_DISPATCH_MAX_THREAD_GROUPS_PER_DIMENSION ||
                description.group_count_y > D3D11_CS_DISPATCH_MAX_THREAD_GROUPS_PER_DIMENSION ||
                description.group_count_z > D3D11_CS_DISPATCH_MAX_THREAD_GROUPS_PER_DIMENSION))
        {
            diagnostic = "Compute dispatch group counts are outside the D3D11 limits.";
            return false;
        }
        for (const stored_pass& pass : _passes)
        {
            if (pass.name == description.name)
            {
                diagnostic = "Compute pass names must be unique within a graph.";
                return false;
            }
        }

        std::set<uint32_t> srv_slots;
        std::set<uint32_t> uav_slots;
        std::set<uint32_t> constant_slots;
        std::vector<ID3D11View*> srv_views;
        std::vector<ID3D11View*> uav_views;
        srv_views.reserve(description.shader_resources.size());
        uav_views.reserve(description.unordered_access_views.size());

        for (const compute_srv_binding& binding : description.shader_resources)
        {
            if (binding.view == nullptr ||
                binding.slot >= D3D11_COMMONSHADER_INPUT_RESOURCE_SLOT_COUNT ||
                !srv_slots.insert(binding.slot).second)
            {
                diagnostic = "Compute pass contains a null SRV, invalid SRV slot or duplicate SRV slot.";
                return false;
            }
            srv_views.push_back(binding.view);
        }

        for (const compute_uav_binding& binding : description.unordered_access_views)
        {
            if (binding.view == nullptr ||
                binding.slot >= D3D11_PS_CS_UAV_REGISTER_COUNT ||
                !uav_slots.insert(binding.slot).second)
            {
                diagnostic = "Compute pass contains a null UAV, invalid UAV slot or duplicate UAV slot.";
                return false;
            }
            uav_views.push_back(binding.view);
        }

        for (std::size_t srv_index = 0; srv_index < srv_views.size(); ++srv_index)
        {
            for (std::size_t uav_index = 0; uav_index < uav_views.size(); ++uav_index)
            {
                if (same_resource(srv_views[srv_index], uav_views[uav_index]))
                {
                    diagnostic = "A compute pass cannot bind the same resource as both SRV and UAV.";
                    return false;
                }
            }
        }

        for (const compute_constant_buffer_binding& binding : description.constant_buffers)
        {
            if (binding.buffer == nullptr ||
                binding.slot >= D3D11_COMMONSHADER_CONSTANT_BUFFER_API_SLOT_COUNT ||
                !constant_slots.insert(binding.slot).second)
            {
                diagnostic = "Compute pass contains a null constant buffer, invalid slot or duplicate constant-buffer slot.";
                return false;
            }
        }

        if (description.indirect_arguments != nullptr)
        {
            for (ID3D11View* view : srv_views)
            {
                if (same_resource(description.indirect_arguments, view))
                {
                    diagnostic = "DispatchIndirect arguments cannot alias a resource bound as SRV in the same pass.";
                    return false;
                }
            }
            for (ID3D11View* view : uav_views)
            {
                if (same_resource(description.indirect_arguments, view))
                {
                    diagnostic = "DispatchIndirect arguments must be unbound from UAV before DispatchIndirect.";
                    return false;
                }
            }
            for (const compute_constant_buffer_binding& binding : description.constant_buffers)
            {
                if (same_resource(description.indirect_arguments,
                        static_cast<ID3D11Resource*>(binding.buffer)))
                {
                    diagnostic = "DispatchIndirect arguments cannot alias a constant buffer in the same pass.";
                    return false;
                }
            }
        }

        stored_pass pass;
        pass.name = description.name;
        pass.shader = description.shader;
        pass.indirect_arguments = description.indirect_arguments;
        pass.indirect_argument_offset = description.indirect_argument_offset;
        pass.group_count_x = description.group_count_x;
        pass.group_count_y = description.group_count_y;
        pass.group_count_z = description.group_count_z;

        for (const compute_srv_binding& binding : description.shader_resources)
        {
            stored_srv_binding stored;
            stored.slot = binding.slot;
            stored.view = binding.view;
            pass.shader_resources.push_back(std::move(stored));
        }
        for (const compute_uav_binding& binding : description.unordered_access_views)
        {
            stored_uav_binding stored;
            stored.slot = binding.slot;
            stored.initial_count = binding.initial_count;
            stored.view = binding.view;
            pass.unordered_access_views.push_back(std::move(stored));
        }
        for (const compute_constant_buffer_binding& binding : description.constant_buffers)
        {
            stored_constant_buffer_binding stored;
            stored.slot = binding.slot;
            stored.buffer = binding.buffer;
            pass.constant_buffers.push_back(std::move(stored));
        }

        _passes.push_back(std::move(pass));
        return true;
    }

    void d3d11_compute_graph::clear()
    {
        _passes.clear();
    }

    std::size_t d3d11_compute_graph::pass_count() const
    {
        return _passes.size();
    }

    compute_graph_result d3d11_compute_graph::execute(
        ID3D11DeviceContext* immediate_context) const
    {
        compute_graph_result result;
        if (immediate_context == nullptr)
        {
            result.diagnostic = "Compute graph requires a non-null immediate context.";
            return result;
        }

        for (const stored_pass& pass : _passes)
        {
            unbind_compute_resources(immediate_context);

            for (const stored_srv_binding& binding : pass.shader_resources)
            {
                ID3D11ShaderResourceView* view = binding.view.Get();
                immediate_context->CSSetShaderResources(binding.slot, 1, &view);
            }
            for (const stored_uav_binding& binding : pass.unordered_access_views)
            {
                ID3D11UnorderedAccessView* view = binding.view.Get();
                const UINT initial_count = binding.initial_count;
                immediate_context->CSSetUnorderedAccessViews(
                    binding.slot, 1, &view, &initial_count);
            }
            for (const stored_constant_buffer_binding& binding : pass.constant_buffers)
            {
                ID3D11Buffer* buffer = binding.buffer.Get();
                immediate_context->CSSetConstantBuffers(binding.slot, 1, &buffer);
            }

            immediate_context->CSSetShader(pass.shader.Get(), nullptr, 0);
            if (pass.indirect_arguments)
            {
                // The pass preamble unbinds every CS SRV / UAV slot, including a
                // DispatchIndirect buffer written by the immediately preceding pass.
                immediate_context->DispatchIndirect(
                    pass.indirect_arguments.Get(),
                    pass.indirect_argument_offset);
            }
            else
            {
                immediate_context->Dispatch(
                    pass.group_count_x, pass.group_count_y, pass.group_count_z);
            }
            ++result.executed_pass_count;
            unbind_compute_resources(immediate_context);
        }

        result.success = true;
        return result;
    }
}
