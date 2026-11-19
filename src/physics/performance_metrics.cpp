#include "skin_physics/physics/performance_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace skin_physics::physics
{
    namespace
    {
        metric_distribution summarize_values(std::vector<double> values)
        {
            metric_distribution result;
            if (values.empty())
            {
                return result;
            }
            std::sort(values.begin(), values.end());
            const std::size_t middle = values.size() / 2;
            if ((values.size() & 1u) != 0)
            {
                result.median = values[middle];
            }
            else
            {
                result.median = (values[middle - 1] + values[middle]) * 0.5;
            }
            const std::size_t percentile_index =
                static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(values.size()))) - 1;
            result.percentile_95 = values[std::min(percentile_index, values.size() - 1)];
            result.maximum = values.back();
            return result;
        }

        bool finite_non_negative(double value)
        {
            return std::isfinite(value) && value >= 0.0;
        }

        void include_maximum_counters(
            performance_counters& destination,
            const performance_counters& source)
        {
            destination.tetrahedron_count =
                std::max(destination.tetrahedron_count, source.tetrahedron_count);
            destination.cloth_triangle_count =
                std::max(destination.cloth_triangle_count, source.cloth_triangle_count);
            destination.candidate_count =
                std::max(destination.candidate_count, source.candidate_count);
            destination.contact_count =
                std::max(destination.contact_count, source.contact_count);
            destination.overflow_count =
                std::max(destination.overflow_count, source.overflow_count);
            destination.ccd_iteration_count =
                std::max(destination.ccd_iteration_count, source.ccd_iteration_count);
            destination.library_allocated_bytes =
                std::max(destination.library_allocated_bytes, source.library_allocated_bytes);
            if (source.dxgi_memory_available)
            {
                destination.dxgi_current_usage_bytes =
                    std::max(destination.dxgi_current_usage_bytes, source.dxgi_current_usage_bytes);
                destination.dxgi_budget_bytes =
                    std::max(destination.dxgi_budget_bytes, source.dxgi_budget_bytes);
                destination.dxgi_memory_available = true;
            }
        }
    }

    metric_distribution calculate_metric_distribution(const std::vector<double>& samples)
    {
        return summarize_values(samples);
    }

    bool performance_collector::add_sample(
        const performance_sample& sample,
        performance_error& error)
    {
        error = performance_error::none;
        if (sample.scenario_hash.empty())
        {
            error = performance_error::empty_scenario_hash;
            return false;
        }
        if (!finite_non_negative(sample.cpu_time_ms))
        {
            error = performance_error::invalid_cpu_time;
            return false;
        }
        if (!sample.all_values_finite ||
            !sample.all_capacity_overflows_reported ||
            !sample.candidate_set_contains_reference ||
            !sample.state_committed_only_after_complete_step)
        {
            error = performance_error::correctness_invariant_failed;
            return false;
        }

        std::set<std::string> pass_names;
        for (const gpu_pass_timing& timing : sample.gpu_pass_timings)
        {
            if (timing.pass_name.empty() || !finite_non_negative(timing.milliseconds))
            {
                error = performance_error::invalid_gpu_timing;
                return false;
            }
            if (!pass_names.insert(timing.pass_name).second)
            {
                error = performance_error::duplicate_gpu_pass;
                return false;
            }
        }

        _samples.push_back(sample);
        return true;
    }

    performance_summary performance_collector::summarize(
        const std::string& expected_scenario_hash,
        std::size_t minimum_sample_count) const
    {
        performance_summary result;
        result.scenario_hash = expected_scenario_hash;
        result.sample_count = _samples.size();
        if (expected_scenario_hash.empty())
        {
            result.diagnostic = "Expected scenario hash must not be empty.";
            return result;
        }
        if (_samples.size() < minimum_sample_count)
        {
            result.diagnostic = "Insufficient repeated samples for the requested profile.";
            return result;
        }

        std::vector<double> cpu_times;
        std::map<std::string, std::vector<double>> gpu_times;
        cpu_times.reserve(_samples.size());
        for (const performance_sample& sample : _samples)
        {
            if (sample.scenario_hash != expected_scenario_hash)
            {
                result.diagnostic = "Samples with different scenario hashes cannot be compared.";
                return result;
            }
            cpu_times.push_back(sample.cpu_time_ms);
            for (const gpu_pass_timing& timing : sample.gpu_pass_timings)
            {
                gpu_times[timing.pass_name].push_back(timing.milliseconds);
            }
            include_maximum_counters(result.maximum_counters, sample.counters);
        }

        result.cpu_time_ms = summarize_values(cpu_times);
        for (const auto& entry : gpu_times)
        {
            result.gpu_pass_time_ms.emplace(entry.first, summarize_values(entry.second));
        }
        result.gpu_timing_available = !gpu_times.empty();
        result.dxgi_memory_available = result.maximum_counters.dxgi_memory_available;
        result.success = true;
        return result;
    }

    void performance_collector::reset()
    {
        _samples.clear();
    }

    std::size_t performance_collector::sample_count() const
    {
        return _samples.size();
    }
}
