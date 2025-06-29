#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <thread>
#include <vector>

#include "Eigen/Geometry"
#include "ceres/ceres.h"

#include "utils/print.h"

namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct Pose3d {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using MapOfPoses = std::map<int, Pose3d, std::less<>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>>;

std::istream& operator>>(std::istream& input, Pose3d& pose) {
    input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >> pose.q.y() >> pose.q.z() >> pose.q.w();
    pose.q.normalize();

    return input;
}

void read_pose(std::ifstream& infile, MapOfPoses& poses) {
    int id;
    Pose3d pose;
    infile >> id >> pose;
    poses[id] = pose;
}

struct Edge {
    int id1;
    int id2;
    Pose3d T12;

    Eigen::Matrix<double, 6, 6> information;  // 信息矩阵
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using VecOfEdges = std::vector<Edge, Eigen::aligned_allocator<Edge>>;

std::istream& operator>>(std::istream& input, Edge& edge) {
    Pose3d& T12 = edge.T12;
    input >> edge.id1 >> edge.id2 >> T12;
    for (int i = 0; i < 6 && input.good(); ++i) {
        for (int j = i; j < 6 && input.good(); ++j) {
            input >> edge.information(i, j);
            if (i != j) {
                edge.information(j, i) = edge.information(i, j);
            }
        }
    }
    return input;
}

void read_edge(std::ifstream& input, VecOfEdges& edges) {
    Edge edge;
    input >> edge;
    edges.push_back(std::move(edge));
}

void read_g2o(const std::string& path, MapOfPoses& poses, VecOfEdges& edges) {
    poses.clear();
    edges.clear();
    std::ifstream infile(path.c_str());
    if (!infile) {
        utils::print("can't read file in given path!");
        std::exit(-1);
    }

    std::string type;
    while (infile.good()) {
        infile >> type;
        if (type == "VERTEX_SE3:QUAT") {
            read_pose(infile, poses);
        } else if (type == "EDGE_SE3:QUAT") {
            read_edge(infile, edges);
        } else {
            utils::print("unknown data type:", type);
            std::exit(-1);
        }

        infile >> std::ws;
    }
}

void output_poses_to_file(const std::string& file, const MapOfPoses& poses) {
    std::fstream outfile;
    outfile.open(file.c_str(), std::istream::out);
    if (!outfile) {
        utils::print("can't open file in given path!");
        std::exit(-1);
    }
    for (const auto& [id, pose] : poses) {
        outfile << id << " " << pose.p.transpose() << " " << pose.q.x() << pose.q.y() << pose.q.z() << pose.q.w()
                << "\n";
    }
}

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement. We have two poses x_a
// and x_b. Through sensor measurements we can measure the transformation of
// frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
// between the current estimate of the poses and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].

// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1] * q_b         ]
//
// where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
// quaternion. Now we can compute an error metric between the estimated and
// measurement transformation. For the orientation error, we will use the
// standard multiplicative error resulting in:
//
//   error = [ p_ab - \hat{p}_ab                 ]
//           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.
class PoseEdgeError {
public:
    PoseEdgeError(Pose3d Tab_meatured, Matrix6d sqrt_information)
        : Tab_meatured_(std::move(Tab_meatured)), sqrt_information_(std::move(sqrt_information)) {}

    template <class T>
    bool operator()(const T* const p_a_ptr, const T* const q_a_ptr, const T* const p_b_ptr, const T* const q_b_ptr,
                    T* residuals_ptr) const {
        using Matrix3T = Eigen::Matrix<T, 3, 1>;
        using QuaternionT = Eigen::Quaternion<T>;

        Eigen::Map<const Matrix3T> p_a(p_a_ptr);
        Eigen::Map<const Matrix3T> p_b(p_b_ptr);
        Eigen::Map<const QuaternionT> q_a(q_a_ptr);
        Eigen::Map<const QuaternionT> q_b(q_b_ptr);

        QuaternionT q_a_inv = q_a.conjugate();
        QuaternionT q_ab_estimated = q_a_inv * q_b;

        Matrix3T p_ab_estimated = q_a_inv * (p_b - p_a);

        QuaternionT delta_q = Tab_meatured_.q.template cast<T>() * q_ab_estimated.conjugate();

        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);

        residuals.template block<3, 1>(0, 0) = p_ab_estimated - Tab_meatured_.p.template cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // 使用信息矩阵为残差加上权重
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Pose3d& Tab_meatured, const Matrix6d& sqrt_infomation) {
        return new ceres::AutoDiffCostFunction<PoseEdgeError, 6, 3, 4, 3, 4>(
            new PoseEdgeError(Tab_meatured, sqrt_infomation));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Pose3d Tab_meatured_;
    Matrix6d sqrt_information_;
};

void build_problem(const VecOfEdges& edges, MapOfPoses& poses, ceres::Problem* problem) {
    if (edges.empty() || poses.empty() || problem == nullptr) {
        return;
    }

    // ceres 1.14
    // ceres::LocalParameterization* quat_param = new ceres::EigenQuaternionParameterization();

    // ceres 2.0+
    ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;

    for (const auto& edge : edges) {
        auto pose1_itr = poses.find(edge.id1);
        auto pose2_itr = poses.find(edge.id2);
        if (pose1_itr == poses.end() || pose2_itr == poses.end()) {
            return;
        }
        const auto sqrt_info = edge.information.llt().matrixL();
        ceres::CostFunction* cost_function = PoseEdgeError::Create(edge.T12, sqrt_info);
        problem->AddResidualBlock(cost_function, nullptr, pose1_itr->second.p.data(),
                                  pose1_itr->second.q.coeffs().data(), pose2_itr->second.p.data(),
                                  pose2_itr->second.q.coeffs().data());
        // ceres 1.14
        // problem->SetParameterization(pose1_itr->second.q.coeffs().data(), quat_param);
        // problem->SetParameterization(pose2_itr->second.q.coeffs().data(), quat_param);

        // ceres 2.0+
        problem->SetManifold(pose1_itr->second.q.coeffs().data(), quaternion_manifold);
        problem->SetManifold(pose2_itr->second.q.coeffs().data(), quaternion_manifold);
    }

    // 固定住第一个位姿
    auto first_trans = poses.begin();
    problem->SetParameterBlockConstant(first_trans->second.p.data());
    problem->SetParameterBlockConstant(first_trans->second.q.coeffs().data());
}

void solve_problem(ceres::Problem* problem) {
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 200;
    options.num_threads = std::thread::hardware_concurrency();

    ceres::Solve(options, problem, &summary);

    std::cout << std::boolalpha;
    utils::print("solve success or not:", summary.IsSolutionUsable());
    utils::print(summary.FullReport());
}

}  // namespace

int main() {
    MapOfPoses poses;
    VecOfEdges edges;

    read_g2o("../../data/sphere.g2o", poses, edges);
    output_poses_to_file("../../data/initial_poses.txt", poses);

    ceres::Problem problem;
    build_problem(edges, poses, &problem);
    solve_problem(&problem);

    output_poses_to_file("../../data/optimized_poses.txt", poses);

    return 0;
}