#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "utils/mt_random.h"
#include "utils/print.h"

namespace {

/*******************************************************************
 * TypeDef
 ********************************************************************/
using Point2 = std::array<double, 2>;

/*******************************************************************
 * Global Variable
 ********************************************************************/
constexpr int kIterations = 100;                            // 迭代次数
constexpr int kDataNum = 100;                               // 数据点
constexpr double kSigma = 1.0;                              // 噪声Sigma值
constexpr double kInvSigma = 1.0 / kSigma;                  // 噪声Sigma值倒数
constexpr double kRealA = 1.0, kRealB = 2.0, kRealC = 1.0;  // 真实参数值

/*******************************************************************
 * Function
 ********************************************************************/
std::vector<Point2> prepare_data() {
    Random rdm(42);
    std::vector<Point2> data;
    data.reserve(kDataNum);
    for (int i = 0; i < kDataNum; ++i) {
        double x = i / 100.0;
        double y = std::exp(kRealA * x * x + kRealB * x + kRealC) + rdm.normal(0.0, kSigma);
        data.push_back(Point2{x, y});
    }

    return data;
}

}  // namespace

int main() {
    // y = exp(a * x * x + b * x + c)
    double ae = 2.0, be = -1.0, ce = 5.0;  // 估计参数值

    auto data = prepare_data();

    double cost = 0.;
    double last_cost = 0.;

    for (int i = 0; i < kIterations; ++i) {
        cost = 0.;

        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (const auto& pt : data) {
            const double xr = pt[0];
            const double yr = pt[1];
            const double ye = std::exp(ae * xr * xr + be * xr + ce);
            const double error = (yr - ye);
            cost += error * error;

            Eigen::Vector3d Ji = Eigen::Vector3d::Zero();
            Ji[2] = -ye;             // ∂ei/ ∂ce;
            Ji[1] = -xr * ye;        // ∂ei/ ∂be;
            Ji[0] = -xr * xr * ye;   // ∂ei/ ∂ae;

            H += kInvSigma * kInvSigma * Ji * Ji.transpose();
            b += -kInvSigma * kInvSigma * error * Ji;
        }

        Eigen::Vector3d delta_abc = H.llt().solve(b);

        if (std::isnan(delta_abc[0])) {
            utils::print("Solution for normal equation is illegal!!!");
            break;
        }

        if (i > 0 && cost > last_cost) {
            utils::print("Iterate completed after", i, "iteration");
            break;
        }
        ae += delta_abc[0];
        be += delta_abc[1];
        ce += delta_abc[2];

        last_cost = cost;
    }

    utils::print("real abc:", kRealA, kRealB, kRealC);
    utils::print("estimated abc:", ae, be, ce);

    return 0;
}