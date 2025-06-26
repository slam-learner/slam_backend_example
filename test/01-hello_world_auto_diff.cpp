#include "ceres/ceres.h"
#include "utils/print.h"

namespace {

struct CostFunctor {
    template <class T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

}  // namespace

int main() {
    double initial_x = 5.0;
    double x = initial_x;

    ceres::Problem problem;

    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x:", initial_x, "->x:", x);

    return 0;
}