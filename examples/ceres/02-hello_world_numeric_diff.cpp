#include "ceres/ceres.h"
#include "utils/print.h"

namespace {

struct NumericCostFunctor {
    template <class T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

}  // namespace

int main() {
    const double initial_x = 5.0;
    double x = initial_x;

    ceres::Problem problem;
    ceres::CostFunction* numeric_cost_function =
        new ceres::NumericDiffCostFunction<NumericCostFunctor, ceres::CENTRAL, 1, 1>(new NumericCostFunctor());
    problem.AddResidualBlock(numeric_cost_function, nullptr, &x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x:", initial_x, "->x:", x);

    return 0;
}