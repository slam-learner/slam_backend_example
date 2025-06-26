#include "ceres/ceres.h"
#include "utils/print.h"

namespace {

struct FxFunctor {
    template <class T>
    bool operator()(T const* const parameters, T* residuals) const {
        const auto x1 = parameters[0];
        const auto x2 = parameters[1];
        const auto x3 = parameters[2];
        const auto x4 = parameters[3];

        residuals[0] = x1 + 10. * x2;
        residuals[1] = std::sqrt(5.) * (x3 - x4);
        residuals[2] = (x2 - 2. * x3) * (x2 - 2. * x3);
        residuals[3] = std::sqrt(10) * (x1 - x4) * (x1 - x4);

        return true;
    }
};

struct FxFunctorSplit {
    template <class T>
    bool operator()(T const* const x1, T const* const x2, T const* const x3, T const* const x4, T* residuals) const {
        residuals[0] = x1[0] + 10. * x2[0];
        residuals[1] = std::sqrt(5.) * (x3[0] - x4[0]);
        residuals[2] = (x2[0] - 2. * x3[0]) * (x2[0] - 2. * x3[0]);
        residuals[3] = std::sqrt(10) * (x1[0] - x4[0]) * (x1[0] - x4[0]);

        return true;
    }
};

class AnalyticFxCost : public ceres::SizedCostFunction<4, 4> {
public:
    virtual ~AnalyticFxCost() {}
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        const double* x_vec = parameters[0];
        const double x1 = x_vec[0];
        const double x2 = x_vec[1];
        const double x3 = x_vec[2];
        const double x4 = x_vec[3];

        residuals[0] = x1 + 10 * x2;
        residuals[1] = std::sqrt(5) * (x3 - x4);
        residuals[2] = (x2 - 2 * x3) * (x2 - 2 * x3);
        residuals[3] = std::sqrt(10) * (x1 - x4) * (x1 - x4);

        // 行为函数，列为自变量
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            double* jacobian = jacobians[0];

            // residuals[0] = x1 + 10 * x2;
            jacobian[0 * 4 + 0] = 1.;   // ∂f1/∂x1
            jacobian[0 * 4 + 1] = 10.;  // ∂f1/∂x2
            jacobian[0 * 4 + 2] = 0.;   // ∂f1/∂x3
            jacobian[0 * 4 + 3] = 0.;   // ∂f1/∂x4

            // residuals[1] = sqrt(5) * (x3 - x4);
            jacobian[1 * 4 + 0] = 0.;              // ∂f2/∂x1
            jacobian[1 * 4 + 1] = 0.;              // ∂f2/∂x2
            jacobian[1 * 4 + 2] = std::sqrt(5.);   // ∂f2/∂x3
            jacobian[1 * 4 + 3] = -std::sqrt(5.);  // ∂f2/∂x4

            // residuals[2] = (x2 - 2 * x3)^2;
            double t3 = x2 - 2 * x3;
            jacobian[2 * 4 + 0] = 0.;       // ∂f3/∂x1
            jacobian[2 * 4 + 1] = 2 * t3;   // ∂f3/∂x2
            jacobian[2 * 4 + 2] = -4 * t3;  // ∂f3/∂x3
            jacobian[2 * 4 + 3] = 0.;       // ∂f3/∂x4

            // residuals[3] = sqrt(10) * (x1 - x4)^2;
            double t4 = 2 * std::sqrt(10.) * (x1 - x4);
            jacobian[3 * 4 + 0] = t4;   // ∂f4/∂x1
            jacobian[3 * 4 + 1] = 0.;   // ∂f4/∂x2
            jacobian[3 * 4 + 2] = 0.;   // ∂f4/∂x3
            jacobian[3 * 4 + 3] = -t4;  // ∂f4/∂x4
        }

        return true;
    }
};

class AnalyticFxCostSplit : public ceres::SizedCostFunction<4, 1, 1, 1, 1> {
public:
    virtual ~AnalyticFxCostSplit() {}

    // parameters[i] 指第 i 个参数块，对应 xᵢ
    // residuals[j] 是第 j 个残差，对应 fⱼ
    // jacobians[i] 是一块大小为 (num_residuals × parameter_block_size[i]) 的 row-major 连续数组，表示：∂f / ∂xᵢ
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        const double x1 = parameters[0][0];
        const double x2 = parameters[1][0];
        const double x3 = parameters[2][0];
        const double x4 = parameters[3][0];

        // residuals
        residuals[0] = x1 + 10.0 * x2;
        residuals[1] = std::sqrt(5.0) * (x3 - x4);
        double t3 = x2 - 2.0 * x3;
        residuals[2] = t3 * t3;
        double t4 = x1 - x4;
        residuals[3] = std::sqrt(10.0) * t4 * t4;

        // 行优先存储
        // All Jacobians are stored in row-major order, and jacobians[i] contains the Jacobian of the residual block
        // with respect to the i-th parameter block, i.e., ∂f / ∂xᵢ.
        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                jacobians[0][0] = 1.0;                         // ∂f1/∂x1
                jacobians[0][1] = 0.0;                         // ∂f2/∂x1
                jacobians[0][2] = 0.0;                         // ∂f3/∂x1
                jacobians[0][3] = 2.0 * std::sqrt(10.0) * t4;  // ∂f4/∂x1
            }
            if (jacobians[1] != nullptr) {
                jacobians[1][0] = 10.0;      // ∂f1/∂x2
                jacobians[1][1] = 0.0;       // ∂f2/∂x2
                jacobians[1][2] = 2.0 * t3;  // ∂f3/∂x2
                jacobians[1][3] = 0.0;       // ∂f4/∂x2
            }
            if (jacobians[2] != nullptr) {
                jacobians[2][0] = 0.0;             // ∂f1/∂x3
                jacobians[2][1] = std::sqrt(5.0);  // ∂f2/∂x3
                jacobians[2][2] = -4.0 * t3;       // ∂f3/∂x3
                jacobians[2][3] = 0.0;             // ∂f4/∂x3
            }
            if (jacobians[3] != nullptr) {
                jacobians[3][0] = 0.0;                          // ∂f1/∂x4
                jacobians[3][1] = -std::sqrt(5.0);              // ∂f2/∂x4
                jacobians[3][2] = 0.0;                          // ∂f3/∂x4
                jacobians[3][3] = -2.0 * std::sqrt(10.0) * t4;  // ∂f4/∂x4
            }
        }

        return true;
    }
};

struct F1 {
    template <class T>
    bool operator()(T const* const x1, T const* const x2, T* residuals) const {
        residuals[0] = x1[0] + 10. * x2[0];
        return true;
    }
};

struct F2 {
    template <class T>
    bool operator()(T const* const x3, T const* const x4, T* residuals) const {
        residuals[0] = std::sqrt(5.) * (x3[0] - x4[0]);
        return true;
    }
};

struct F3 {
    template <class T>
    bool operator()(T const* const x2, T const* const x3, T* residuals) const {
        residuals[0] = (x2[0] - 2. * x3[0]) * (x2[0] - 2. * x3[0]);
        return true;
    }
};

struct F4 {
    template <class T>
    bool operator()(T const* const x1, T const* const x4, T* residuals) const {
        residuals[0] = std::sqrt(10.) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
        return true;
    }
};

class AnalyticF1Cost : public ceres::SizedCostFunction<1, 1, 1> {
public:
    virtual ~AnalyticF1Cost() {}
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        const double x1 = parameters[0][0];
        const double x2 = parameters[1][0];

        residuals[0] = x1 + 10. * x2;

        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) jacobians[0][0] = 1.;
            if (jacobians[1] != nullptr) jacobians[1][0] = 10.;
        }

        return true;
    }
};

class AnalyticF2Cost : public ceres::SizedCostFunction<1, 1, 1> {
public:
    virtual ~AnalyticF2Cost() {}
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        const double x3 = parameters[0][0];
        const double x4 = parameters[1][0];

        residuals[0] = std::sqrt(5.) * (x3 - x4);

        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) jacobians[0][0] = std::sqrt(5.);
            if (jacobians[1] != nullptr) jacobians[1][0] = -std::sqrt(5.);
        }

        return true;
    }
};

class AnalyticF3Cost : public ceres::SizedCostFunction<1, 1, 1> {
public:
    virtual ~AnalyticF3Cost() {}
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        const double x2 = parameters[0][0];
        const double x3 = parameters[1][0];

        const double t = x2 - 2. * x3;
        residuals[0] = t * t;

        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) jacobians[0][0] = 2. * t;
            if (jacobians[1] != nullptr) jacobians[1][0] = -4. * t;
        }

        return true;
    }
};

class AnalyticF4Cost : public ceres::SizedCostFunction<1, 1, 1> {
public:
    virtual ~AnalyticF4Cost() {}
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        const double x1 = parameters[0][0];
        const double x4 = parameters[1][0];

        const double t = x1 - x4;
        residuals[0] = std::sqrt(10.) * t * t;

        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) jacobians[0][0] = 2. * std::sqrt(10.) * t;
            if (jacobians[1] != nullptr) jacobians[1][0] = -2. * std::sqrt(10.) * t;
        }

        return true;
    }
};

void numeric_diff() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    problem.AddResidualBlock(new ceres::NumericDiffCostFunction<FxFunctor, ceres::CENTRAL, 4, 4>(new FxFunctor),
                             nullptr, x_vec.data());
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

void numeric_diff_split() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    problem.AddResidualBlock(
        new ceres::NumericDiffCostFunction<FxFunctorSplit, ceres::CENTRAL, 4, 1, 1, 1, 1>(new FxFunctorSplit), nullptr,
        x_vec.data(), &x_vec[1], &x_vec[2], &x_vec[3]);
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

void numeric_diff_blocks() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    problem.AddResidualBlock(new ceres::NumericDiffCostFunction<F1, ceres::CENTRAL, 1, 1, 1>(new F1), nullptr,
                             x_vec.data(), &x_vec[1]);
    problem.AddResidualBlock(new ceres::NumericDiffCostFunction<F2, ceres::CENTRAL, 1, 1, 1>(new F2), nullptr,
                             &x_vec[2], &x_vec[3]);
    problem.AddResidualBlock(new ceres::NumericDiffCostFunction<F3, ceres::CENTRAL, 1, 1, 1>(new F3), nullptr,
                             &x_vec[1], &x_vec[2]);
    problem.AddResidualBlock(new ceres::NumericDiffCostFunction<F4, ceres::CENTRAL, 1, 1, 1>(new F4), nullptr,
                             x_vec.data(), &x_vec[3]);
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

void auto_diff() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<FxFunctor, 4, 4>(new FxFunctor), nullptr, x_vec.data());
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

void auto_diff_split() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<FxFunctorSplit, 4, 1, 1, 1, 1>(new FxFunctorSplit),
                             nullptr, x_vec.data(), &x_vec[1], &x_vec[2], &x_vec[3]);
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

void auto_diff_blocks() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F1, 1, 1, 1>(new F1), nullptr, x_vec.data(), &x_vec[1]);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F2, 1, 1, 1>(new F2), nullptr, &x_vec[2], &x_vec[3]);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F3, 1, 1, 1>(new F3), nullptr, &x_vec[1], &x_vec[2]);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F4, 1, 1, 1>(new F4), nullptr, x_vec.data(), &x_vec[3]);
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

void analytic_diff() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    ceres::CostFunction* analytic_cost_function = new AnalyticFxCost;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    problem.AddResidualBlock(analytic_cost_function, nullptr, x_vec.data());
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

void analytic_diff_split() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    problem.AddResidualBlock(new AnalyticFxCostSplit, nullptr, x_vec.data(), &x_vec[1], &x_vec[2], &x_vec[3]);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

void analytic_diff_blocks() {
    const std::vector initial_x_vec{10.0, -1.0, 0.0, 1.0};
    auto x_vec = initial_x_vec;

    ceres::Problem problem;
    problem.AddResidualBlock(new AnalyticF1Cost, nullptr, x_vec.data(), &x_vec[1]);
    problem.AddResidualBlock(new AnalyticF2Cost, nullptr, &x_vec[2], &x_vec[3]);
    problem.AddResidualBlock(new AnalyticF3Cost, nullptr, &x_vec[1], &x_vec[2]);
    problem.AddResidualBlock(new AnalyticF4Cost, nullptr, x_vec.data(), &x_vec[3]);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    utils::print(summary.BriefReport());
    utils::print("initial x vec:", initial_x_vec[0], initial_x_vec[1], initial_x_vec[2], initial_x_vec[3],
                 "\n-> x vec:", x_vec[0], x_vec[1], x_vec[2], x_vec[3]);
}

}  // namespace

int main() {
    utils::print("----------- Numeric Diff -----------");
    numeric_diff();
    utils::print("----------- Numeric Diff Split -----------");
    numeric_diff_split();
    utils::print("----------- Numeric Diff Blocks -----------");
    numeric_diff_blocks();
    utils::print("----------- Automatic Diff -----------");
    auto_diff();
    utils::print("----------- Automatic Diff Split-----------");
    auto_diff_split();
    utils::print("----------- Automatic Diff Blocks-----------");
    auto_diff_blocks();
    utils::print("----------- Analytic Diff -----------");
    analytic_diff();
    utils::print("----------- Analytic Diff Split -----------");
    analytic_diff_split();
    utils::print("----------- Analytic Diff Blocks -----------");
    analytic_diff_blocks();
    return 0;
}