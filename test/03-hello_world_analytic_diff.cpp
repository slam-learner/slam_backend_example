#include "ceres/ceres.h"
#include "utils/print.h"

namespace {

class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
public:
    virtual ~QuadraticCostFunction() {}
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        const double x = parameters[0][0];
        residuals[0] = 10.0 - x;
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            jacobians[0][0] = -1;
        }
        return true;
    }
};

}  // namespace

int main() {
    const double initial_x = 5.0;
    double x = initial_x;

    ceres::Problem problem;
    ceres::CostFunction* auto_cost_function = new QuadraticCostFunction;
    problem.AddResidualBlock(auto_cost_function, nullptr, &x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x:", initial_x, "->x:", x);

    return 0;
}