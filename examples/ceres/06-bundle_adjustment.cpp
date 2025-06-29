#include <fstream>
#include <string>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "utils/print.h"

namespace {

using Cam = std::array<double, 9>;
using MapPt = std::array<double, 3>;

struct Observation {
    int cam_index;
    int pt_index;
    double obs_x;
    double obs_y;
};

struct BalProblem {
    void load_file(const std::string& path) {
        std::ifstream ifs(path.c_str());
        if (!ifs.is_open()) {
            utils::print("can't read file in given path");
            std::exit(-1);
        }
        int num_cameras{}, num_points{}, num_obeservations{};
        ifs >> num_cameras >> num_points >> num_obeservations;
        cameras.reserve(num_cameras);
        points.reserve(num_points);
        observations.reserve(num_obeservations);
        int cam_index{}, pt_index{};
        double obs_x{}, obs_y{};
        for (int i = 0; i < num_obeservations; ++i) {
            ifs >> cam_index >> pt_index >> obs_x >> obs_y;
            observations.push_back({cam_index, pt_index, obs_x, obs_y});
        }

        double axis_x{}, axis_y{}, axis_z{}, tx{}, ty{}, tz{}, f{}, k1{}, k2{};
        for (int i = 0; i < num_cameras; ++i) {
            ifs >> axis_x >> axis_y >> axis_z >> tx >> ty >> tz >> f >> k1 >> k2;
            cameras.push_back(Cam{axis_x, axis_y, axis_z, tx, ty, tz, f, k1, k2});
        }

        double X{}, Y{}, Z{};
        for (int i = 0; i < num_points; ++i) {
            ifs >> X >> Y >> Z;
            points.push_back(MapPt{X, Y, Z});
        }
    }

    std::vector<Observation> observations;
    std::vector<Cam> cameras;   // axis_x, axis_y, axis_z, tx, ty, tz, f, k1, k2
    std::vector<MapPt> points;  // X, Y, Z
};

struct SnavelyReprojectionError {
    SnavelyReprojectionError(double obs_x, double obs_y) : obs_x_(obs_x), obs_y_(obs_y) {}

    template <class T>
    bool operator()(T const* const cam, T const* const map_pt, T* residuals) const {
        T pt[3];
        // Pw->Pc
        ceres::AngleAxisRotatePoint(cam, map_pt, pt);
        pt[0] += cam[3];
        pt[1] += cam[4];
        pt[2] += cam[5];

        // Pc->归一化相机坐标
        T xp = pt[0] / -pt[2];
        T yp = pt[1] / -pt[2];

        // 归一化相机坐标->图像坐标系
        T f = cam[6], k1 = cam[7], k2 = cam[8];
        T r2 = xp * xp + yp * yp;
        T distortion = T(1.0) + k1 * r2 + k2 * r2 * r2;
        T ux = f * distortion * xp;
        T uy = f * distortion * yp;

        residuals[0] = ux - obs_x_;
        residuals[1] = uy - obs_y_;

        return true;
    }

    double obs_x_;
    double obs_y_;
};

}  // namespace

int main() {
    BalProblem bal_problem;
    bal_problem.load_file("../../data/problem-49-7776-pre.txt");
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::DENSE_SCHUR;

    auto& cams = bal_problem.cameras;
    auto& pts = bal_problem.points;
    for (const auto& obeservation : bal_problem.observations) {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                                     new SnavelyReprojectionError(obeservation.obs_x, obeservation.obs_y)),
                                 nullptr, cams[obeservation.cam_index].data(), pts[obeservation.pt_index].data());
    }
    ceres::Solve(options, &problem, &summary);
    utils::print(summary.BriefReport());

    return 0;
}