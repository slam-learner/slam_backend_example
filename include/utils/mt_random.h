#pragma once

#include <random>
#include <type_traits>

class Random {
public:
    // 默认构造：使用不可预测的随机种子
    Random() : rng_(std::random_device{}()) {}

    // 使用固定种子构造：可复现
    explicit Random(uint32_t seed) : rng_(seed) {}

    // 模板：生成正态分布随机数（适用于 float, double, long double）
    template<typename RealType = double>
    RealType normal(RealType mean = 0.0, RealType stddev = 1.0) {
        static_assert(std::is_floating_point_v<RealType>,
                      "normal() only supports floating-point types.");
        std::normal_distribution<RealType> dist(mean, stddev);
        return dist(rng_);
    }

private:
    std::mt19937 rng_;
};
