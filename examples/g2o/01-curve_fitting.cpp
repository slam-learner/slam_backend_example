#include <absl/algorithm/container.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "utils/mt_random.h"
#include "utils/print.h"
#include "utils/timer.h"

namespace {

/*******************************************************************
 * TypeDef
 ********************************************************************/
using Point2 = std::array<double, 2>;
// 误差项优化变量, 模板参数：优化变量维度、观测量维度
using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
// 线性求解器类型
using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

/*******************************************************************
 * Global Variable
 ********************************************************************/
constexpr int kDataNum = 100;                                         // 数据点
constexpr double kSigma = 1.0;                                        // 噪声Sigma值
constexpr double kInvSigma = 1.0 / kSigma;                            // 噪声Sigma值倒数
constexpr double kRealA = 1.0, kRealB = 2.0, kRealC = 1.0;            // 真实参数值
constexpr double kInitialA = 2.0, kInitialB = -1.0, kInitialC = 5.0;  // 参数初始估计值

/*******************************************************************
 * Class/Struct
 ********************************************************************/
// 模板参数：优化变量的维度、变量类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override { _estimate << 0.0, 0.0, 0.0; }

    void oplusImpl(const double* update) override { _estimate += Eigen::Map<const Eigen::Vector3d>(update); }

    bool read(std::istream& /*in*/) override { return true; }

    bool write(std::ostream& /*out*/) const override { return true; }
};

// 模板参数：观测值维度、观测值类型、连接顶点类型
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x) : x_(x) {}

    // 计算误差
    void computeError() override {
        // 从边所连接的顶点中取出当前的状态值
        const auto* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const auto abc = v->estimate();
        double ye = std::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]);
        _error(0, 0) = _measurement - ye;
    }

    // 计算雅可比
    void linearizeOplus() override {
        const auto* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const auto abc = v->estimate();
        double ye = std::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]);
        _jacobianOplusXi[0] = -x_ * x_ * ye;
        _jacobianOplusXi[1] = -x_ * ye;
        _jacobianOplusXi[2] = -ye;
    }

    bool read(std::istream& /*in*/) override { return true; }

    bool write(std::ostream& /*out*/) const override { return true; }

private:
    double x_;  // x 值，y值为观测值，在添加边时由 setMeasurement 函数设置
};

/*******************************************************************
 * Function
 ********************************************************************/
std::vector<Point2> prepare_data() {
    constexpr int seed = 42;
    Random rdm(seed);
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
    double ae = kInitialA;
    double be = kInitialB;
    double ce = kInitialC;
    auto data = prepare_data();

    // 求解方法，这里选用GN法
    auto* solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    // 整个图模型
    g2o::SparseOptimizer optimizer;
    // 设置求解器
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    auto* v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    for (int i = 0; i < data.size(); ++i) {
        auto* edge = new CurveFittingEdge(data[i][0]);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(data[i][1]);
        // 信息矩阵：协方差矩阵的逆
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * kInvSigma * kInvSigma);
        optimizer.addEdge(edge);
    }

    utils::print("start optimization");
    constexpr int kIterations = 10;
    auto time_used = utils::Timer::evaluate_once([&optimizer]() -> void {
        optimizer.initializeOptimization();
        optimizer.optimize(kIterations);
    });
    utils::print("time cost(ms):", time_used);
    utils::print("estimate value:", v->estimate().transpose());

    return 0;
}