#include <fstream>
#include <string>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <Eigen/Core>

#include <sophus/se3.h>
#include <sophus/so3.h>

#include "utils/print.h"
#include "utils/timer.h"

namespace {

/*******************************************************************
 * TypeDef
 ********************************************************************/
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

/*******************************************************************
 * Class/Struct
 ********************************************************************/
// J_R^{-1}的近似
Matrix6d JRInv(const Sophus::SE3& e) {
    Matrix6d JRInv;
    JRInv.block<3, 3>(0, 0) = Sophus::SO3::hat(e.so3().log());
    JRInv.block<3, 3>(0, 3) = Sophus::SO3::hat(e.translation());
    JRInv.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    JRInv.block<3, 3>(3, 3) = Sophus::SO3::hat(e.so3().log());

    JRInv = Matrix6d::Identity() + 1.0 / 2 * JRInv;
    return JRInv;
}

// 模板参数：优化变量的维度、变量类型
class VertexSE3LieAlgebra : public g2o::BaseVertex<6, Sophus::SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override { _estimate = Sophus::SE3(); }

    void oplusImpl(const double* update) override {
        Eigen::Map<const Vector6d> rho_phi(update);
        _estimate = Sophus::SE3::exp(rho_phi) * _estimate;
    }

    bool read(std::istream& in) override {
        std::array<double, 7> data;
        for (auto& data_i : data) {
            in >> data_i;
        }

        _estimate = Sophus::SE3(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                                Eigen::Vector3d(data[0], data[1], data[2]));

        return true;
    }

    bool write(std::ostream& out) const override {
        const auto q_coeffs = _estimate.unit_quaternion().coeffs();
        out << id() << " " << _estimate.translation().transpose() << " " << q_coeffs[0] << " " << q_coeffs[1] << " "
            << q_coeffs[2] << " " << q_coeffs[3] << std::endl;

        return true;
    }
};

// 模板参数：观测值维度、观测值类型、连接顶点类型
class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, Sophus::SE3, VertexSE3LieAlgebra, VertexSE3LieAlgebra> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {
        const auto* const v1 = static_cast<const VertexSE3LieAlgebra*>(_vertices[0]);
        const auto* const v2 = static_cast<const VertexSE3LieAlgebra*>(_vertices[1]);

        _error = Sophus::SE3::log(_measurement.inverse() * v1->estimate().inverse() * v2->estimate());
    }

    void linearizeOplus() override {
        const auto adj_of_Tj_inv = static_cast<const VertexSE3LieAlgebra*>(_vertices[1])->estimate().inverse().Adj();
        const auto J = JRInv(Sophus::SE3::exp(_error));
        _jacobianOplusXj = J * adj_of_Tj_inv;
        _jacobianOplusXi = -_jacobianOplusXj;
    }

    bool read(std::istream& in) override {
        std::array<double, 7> data;
        for (auto& data_i : data) {
            in >> data_i;
        }

        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();

        setMeasurement(Sophus::SE3(q, Eigen::Vector3d(data[0], data[1], data[2])));

        for (int i = 0; i < information().rows() && in.good(); i++) {
            for (int j = i; j < information().cols() && in.good(); j++) {
                in >> information()(i, j);
                if (i != j) {
                    information()(j, i) = information()(i, j);
                }
            }
        }

        return true;
    }

    bool write(std::ostream& out) const override {
        const auto* const v1 = static_cast<VertexSE3LieAlgebra*>(_vertices[0]);
        const auto* const v2 = static_cast<VertexSE3LieAlgebra*>(_vertices[1]);
        out << v1->id() << " " << v2->id() << " ";
        Sophus::SE3 m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        out << m.translation().transpose() << " ";
        out << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // information matrix
        for (int i = 0; i < information().rows(); i++) {
            for (int j = i; j < information().cols(); j++) {
                out << information()(i, j) << " ";
            }
        }
        out << std::endl;

        return true;
    }
};

}  // namespace

int main() {
    std::ifstream fin("../../data/sphere.g2o");
    if (!fin.is_open()) {
        utils::print("can't read file in given path!");
        std::exit(-1);
    }

    // 求解方法，这里选用LM法
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    // 整个图模型
    g2o::SparseOptimizer optimizer;
    // 设置求解器
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int vertex_cnt{}, edge_cnt{};

    std::string data_type;
    std::vector<VertexSE3LieAlgebra*> vectices;
    std::vector<EdgeSE3LieAlgebra*> edges;
    while (fin >> data_type) {
        if (data_type == "VERTEX_SE3:QUAT") {
            auto* v = new VertexSE3LieAlgebra;
            int idx{};
            fin >> idx;
            v->setId(idx);
            v->read(fin);
            optimizer.addVertex(v);
            ++vertex_cnt;
            v->setFixed(idx == 0);
            vectices.push_back(v);
        } else if (data_type == "EDGE_SE3:QUAT") {
            auto* e = new EdgeSE3LieAlgebra;
            int idx1{}, idx2{};
            fin >> idx1 >> idx2;
            e->setId(edge_cnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
            edges.push_back(e);
        }
        if (!fin.good()) {
            break;
        }
    }

    utils::print("vertices cnt:", vertex_cnt, "edge cnt:", edge_cnt);
    utils::print("optimizing");

    auto time_cost = utils::Timer::evaluate_once([&optimizer]() {
        optimizer.initializeOptimization();
        optimizer.optimize(100);
    });
    utils::print("time_cost:", time_cost);
    utils::print("saving result...");

    std::ofstream fout("../../data/result_lie.g2o");
    for (VertexSE3LieAlgebra* v : vectices) {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for (EdgeSE3LieAlgebra* e : edges) {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();

    return 0;
}