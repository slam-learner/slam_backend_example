#include <fstream>
#include <string>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include "utils/print.h"
#include "utils/timer.h"

namespace {

/*******************************************************************
 * TypeDef
 ********************************************************************/
using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

}  // namespace

int main() {
    std::ifstream fin("../../data/sphere.g2o");
    if (!fin.is_open()) {
        utils::print("can't read file in given path!");
        std::exit(-1);
    }
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int vertex_cnt{}, edge_cnt{};

    std::string data_type;
    while (fin >> data_type) {
        if (data_type == "VERTEX_SE3:QUAT") {
            auto* v = new g2o::VertexSE3();
            int idx{};
            fin >> idx;
            v->setId(idx);
            v->read(fin);
            optimizer.addVertex(v);
            ++vertex_cnt;
            if (idx == 0) {
                v->setFixed(true);
            }
        } else if (data_type == "EDGE_SE3:QUAT") {
            auto* e = new g2o::EdgeSE3();
            int idx1{}, idx2{};
            fin >> idx1 >> idx2;
            e->setId(edge_cnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
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
    optimizer.save("../../data/result.g2o");

    return 0;
}