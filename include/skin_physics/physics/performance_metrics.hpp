#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace skin_physics::physics
{
    struct gpu_pass_timing
    {
        std::string pass_name;
        double milliseconds = 0.0;
    };

    struct performance_counters
    {
        uint64_t tetrahedron_count = 0;
        uint64_t cloth_triangle_count = 0;
        uint64_t candidate_count = 0;
        uint64_t contact_count = 0;
        uint64_t overflow_count = 0;
        uint64_t ccd_iteration_count = 0;
        uint64_t library_allocated_bytes = 0;
        uint64_t dxgi_current_usage_bytes = 0;
        uint64_t dxgi_budget_bytes = 0;
        bool dxgi_memory_available = false;
    };

    struct performance_sample
    {
        std::string scenario_hash;
        double cpu_time_ms = 0.0;
        std::vector<gpu_pass_timing> gpu_pass_timings;
        performance_counters counters;
        bool all_values_finite = true;
        bool all_capacity_overflows_reported = true;
        bool candidate_set_contains_reference = true;
        bool state_committed_only_after_complete_step = true;
    };

    struct metric_distribution
    {
        double median = 0.0;
        double percentile_95 = 0.0;
        double maximum = 0.0;
    };

    struct performance_summary
    {
        std::string scenario_hash;
        metric_distribution cpu_time_ms;
        std::map<std::string, metric_distribution> gpu_pass_time_ms;
        performance_counters maximum_counters;
        std::size_t sample_count = 0;
        bool gpu_timing_available = false;
        bool dxgi_memory_available = false;
        bool success = false;
        std::string diagnostic;
    };

    enum class performance_error : uint8_t
    {
        none,
        empty_scenario_hash,
        invalid_cpu_time,
        invalid_gpu_timing,
        duplicate_gpu_pass,
        correctness_invariant_failed,
        scenario_hash_mismatch,
        insufficient_samples
    };

    class performance_collector
    {
    public:
        bool add_sample(const performance_sample& sample, performance_error& error);
        performance_summary summarize(
            const std::string& expected_scenario_hash,
            std::size_t minimum_sample_count = 1) const;
        void reset();
        std::size_t sample_count() const;

    private:
        std::vector<performance_sample> _samples;
    };

    metric_distribution calculate_metric_distribution(const std::vector<double>& samples);
}
