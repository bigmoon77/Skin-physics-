#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <d3d11.h>
#include <wrl/client.h>

namespace skin_physics::simulation
{
    struct compute_srv_binding
    {
        uint32_t slot = 0;
        ID3D11ShaderResourceView* view = nullptr;
    };

    struct compute_uav_binding
    {
        uint32_t slot = 0;
        ID3D11UnorderedAccessView* view = nullptr;
        uint32_t initial_count = 0xffffffffu;
    };

    struct compute_constant_buffer_binding
    {
        uint32_t slot = 0;
        ID3D11Buffer* buffer = nullptr;
    };

    struct compute_pass_desc
    {
        std::string name;
        ID3D11ComputeShader* shader = nullptr;
        std::vector<compute_srv_binding> shader_resources;
        std::vector<compute_uav_binding> unordered_access_views;
        std::vector<compute_constant_buffer_binding> constant_buffers;
        // When non-null, DispatchIndirect consumes three GPU-generated UINT group counts.
        // The argument buffer must carry D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS.
        ID3D11Buffer* indirect_arguments = nullptr;
        uint32_t indirect_argument_offset = 0;
        uint32_t group_count_x = 1;
        uint32_t group_count_y = 1;
        uint32_t group_count_z = 1;
    };

    struct compute_graph_result
    {
        uint32_t executed_pass_count = 0;
        std::string failed_pass;
        std::string diagnostic;
        bool success = false;
    };

    class d3d11_compute_graph
    {
    public:
        bool add_pass(const compute_pass_desc& description, std::string& diagnostic);
        void clear();
        std::size_t pass_count() const;
        compute_graph_result execute(ID3D11DeviceContext* immediate_context) const;

    private:
        struct stored_srv_binding
        {
            uint32_t slot = 0;
            Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> view;
        };

        struct stored_uav_binding
        {
            uint32_t slot = 0;
            uint32_t initial_count = 0xffffffffu;
            Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> view;
        };

        struct stored_constant_buffer_binding
        {
            uint32_t slot = 0;
            Microsoft::WRL::ComPtr<ID3D11Buffer> buffer;
        };

        struct stored_pass
        {
            std::string name;
            Microsoft::WRL::ComPtr<ID3D11ComputeShader> shader;
            std::vector<stored_srv_binding> shader_resources;
            std::vector<stored_uav_binding> unordered_access_views;
            std::vector<stored_constant_buffer_binding> constant_buffers;
            Microsoft::WRL::ComPtr<ID3D11Buffer> indirect_arguments;
            uint32_t indirect_argument_offset = 0;
            uint32_t group_count_x = 1;
            uint32_t group_count_y = 1;
            uint32_t group_count_z = 1;
        };

        std::vector<stored_pass> _passes;
    };
}
