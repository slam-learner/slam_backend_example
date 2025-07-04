//
// Created by xiang on 2021/11/17.
//
#include "utils/timer.h"
#include "utils/print.h"

#include <fstream>
#include <numeric>

namespace utils {

std::map<std::string, Timer::TimerRecord> Timer::records_;

void Timer::print_all() {
    utils::print(">>> ===== Printing run time =====");
    for (const auto& r : records_) {
        utils::print_wo_space(
            "> [ ", r.first, " ] average time usage: ",
            std::accumulate(r.second.time_usage_in_ms_.begin(), r.second.time_usage_in_ms_.end(), 0.0) /
                double(r.second.time_usage_in_ms_.size()),
            " ms , called times: ", r.second.time_usage_in_ms_.size());
    }
    utils::print(">>> ===== Printing run time end =====");
}

void Timer::dump_into_file(const std::string& file_name) {
    std::ofstream ofs(file_name, std::ios::out);
    if (!ofs.is_open()) {
        utils::print("Failed to open file:", file_name);
        return;
    }

    utils::print("Dump Time Records into file:", file_name);

    size_t max_length = 0;
    for (const auto& iter : records_) {
        ofs << iter.first << ", ";
        if (iter.second.time_usage_in_ms_.size() > max_length) {
            max_length = iter.second.time_usage_in_ms_.size();
        }
    }
    ofs << std::endl;

    for (size_t i = 0; i < max_length; ++i) {
        for (const auto& iter : records_) {
            if (i < iter.second.time_usage_in_ms_.size()) {
                ofs << iter.second.time_usage_in_ms_[i] << ",";
            } else {
                ofs << ",";
            }
        }
        ofs << std::endl;
    }
    ofs.close();
}

double Timer::get_mean_time(const std::string& func_name) {
    if (records_.find(func_name) == records_.end()) {
        return 0.0;
    }

    auto r = records_[func_name];
    return std::accumulate(r.time_usage_in_ms_.begin(), r.time_usage_in_ms_.end(), 0.0) /
           double(r.time_usage_in_ms_.size());
}

}  // namespace utils