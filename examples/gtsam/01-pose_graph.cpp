#include <fstream>
#include <string>

#include <sophus/se3.h>
#include <sophus/so3.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include "utils/print.h"
#include "utils/timer.h"

int main() {
    std::ifstream fin("../../data/sphere.g2o");
    // 因子图
    gtsam::NonlinearFactorGraph graph;
    // 初始值
    gtsam::Values initial;

    int vertex_cnt{};
    int edge_cnt{};

    std::string data_type;
    while (fin >> data_type) {
        if (data_type == "VERTEX_SE3:QUAT") {
            gtsam::Key id{};
            fin >> id;
            std::array<double, 7> data{};
            for (double& data_i : data) {
                fin >> data_i;
            }

            gtsam::Rot3 R = gtsam::Rot3::Quaternion(data[6], data[3], data[4], data[5]);
            gtsam::Point3 t(data[0], data[1], data[2]);

            initial.insert(id, gtsam::Pose3(R, t));

            ++vertex_cnt;
        } else if (data_type == "EDGE_SE3:QUAT") {
            gtsam::Matrix m = gtsam::I_6x6;
            gtsam::Key id1{}, id2{};
            fin >> id1 >> id2;
            std::array<double, 7> data;
            for (double& data_i : data) {
                fin >> data_i;
            }

            gtsam::Rot3 R = gtsam::Rot3::Quaternion(data[6], data[3], data[4], data[5]);
            gtsam::Point3 t(data[0], data[1], data[2]);

            for (int i = 0; i < 6; ++i) {
                for (int j = i; j < 6; ++j) {
                    double mij{};
                    fin >> mij;
                    m(i, j) = mij;
                    m(j, i) = mij;
                }
            }

            gtsam::Matrix mgtsam = gtsam::I_6x6;
            mgtsam.block<3, 3>(0, 0) = m.block<3, 3>(3, 3);  // cov rotation
            mgtsam.block<3, 3>(3, 3) = m.block<3, 3>(0, 0);  // cov translation
            mgtsam.block<3, 3>(0, 3) = m.block<3, 3>(0, 3);  // off diagonal
            mgtsam.block<3, 3>(3, 0) = m.block<3, 3>(3, 0);  // off diagonal

            gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id1, id2, gtsam::Pose3(R, t), model));
            ++edge_cnt;
        }
        if (!fin.good()) {
            break;
        }
    }

    gtsam::noiseModel::Diagonal::shared_ptr prior_model =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    // 固定第一个顶点
    for (const auto& [key, value] : initial) {
        std::cout << "Adding prior to g2o file " << std::endl;
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(key, value.cast<gtsam::Pose3>(), prior_model));
        break;
    }

    gtsam::LevenbergMarquardtParams params_lm;
    params_lm.setVerbosity("ERROR");
    params_lm.setMaxIterations(20);
    params_lm.setLinearSolverType("MULTIFRONTAL_QR");

    gtsam::LevenbergMarquardtOptimizer optimizer_lm(graph, initial, params_lm);

    utils::print("begin optimization");
    gtsam::Values result;
    auto time_used = utils::Timer::evaluate_once([&optimizer_lm, &result]() { result = optimizer_lm.optimize(); });
    utils::print("done. time cost(ms):", time_used);

    utils::print("initial error:", graph.error(initial));
    utils::print("final error", graph.error(result));

    utils::print("write to g2o ...");
    // 写入 g2o 文件，同样伪装成 g2o 中的顶点和边，以便用 g2o_viewer 查看。
    // 顶点
    std::ofstream fout("../../data/result_gtsam.g2o");
    for (const auto& [key, value] : result) {
        gtsam::Pose3 pose = value.cast<gtsam::Pose3>();
        gtsam::Point3 p = pose.translation();
        gtsam::Quaternion q = pose.rotation().toQuaternion();
        fout << "VERTEX_SE3:QUAT " << key << " " << p.x() << " " << p.y() << " " << p.z() << " " << q.x() << " "
             << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
    }
    // 边
    for (gtsam::NonlinearFactor::shared_ptr factor : graph) {
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr f =
            std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
        if (f) {
            gtsam::SharedNoiseModel model = f->noiseModel();
            gtsam::noiseModel::Gaussian::shared_ptr gaussianModel =
                std::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(model);
            if (gaussianModel) {
                // write the edge information
                gtsam::Matrix info = gaussianModel->R().transpose() * gaussianModel->R();
                gtsam::Pose3 pose = f->measured();
                gtsam::Point3 p = pose.translation();
                gtsam::Quaternion q = pose.rotation().toQuaternion();
                fout << "EDGE_SE3:QUAT " << f->key1() << " " << f->key2() << " " << p.x() << " " << p.y() << " "
                     << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
                gtsam::Matrix infoG2o = gtsam::I_6x6;
                infoG2o.block(0, 0, 3, 3) = info.block(3, 3, 3, 3);  // cov translation
                infoG2o.block(3, 3, 3, 3) = info.block(0, 0, 3, 3);  // cov rotation
                infoG2o.block(0, 3, 3, 3) = info.block(0, 3, 3, 3);  // off diagonal
                infoG2o.block(3, 0, 3, 3) = info.block(3, 0, 3, 3);  // off diagonal
                for (int i = 0; i < 6; i++) {
                    for (int j = i; j < 6; j++) {
                        fout << infoG2o(i, j) << " ";
                    }
                }
                fout << std::endl;
            }
        }
    }
    fout.close();
    utils::print("save completed!");

    return 0;
}