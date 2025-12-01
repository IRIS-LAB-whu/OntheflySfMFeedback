#include "Pose.h"
#include "Triangulation.h"
#include "../Base/math.h"
#include "../Base/logging.h"
#include "../Base/misc.h"
#include "../Base/threading.h"
//#include "../Estimator/Common.h"
//#include "../Estimator/BundleAdjustment.h"
//#include "../Estimator/CostFunction.h"
//#include "../Estimator/EssentialMatrix.h"
//#include "../Scene/CameraModel.h"


#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

Eigen::Matrix3d ComputeClosestRotationMatrix(const Eigen::Matrix3d& matrix) {
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * (svd.matrixV().transpose());
    if (R.determinant() < 0.0) {
        R *= -1.0;
    }
    return R;
}

bool DecomposeProjectionMatrix(const Eigen::Matrix3x4d& P,
    Eigen::Matrix3d* K,
    Eigen::Matrix3d* R,
    Eigen::Vector3d* T) {
    Eigen::Matrix3d RR;
    Eigen::Matrix3d QQ;
    DecomposeMatrixRQ(P.leftCols<3>().eval(), &RR, &QQ);

    *R = ComputeClosestRotationMatrix(QQ);

    const double det_K = RR.determinant();
    if (det_K == 0) {
        return false;
    }
    else if (det_K > 0) {
        *K = RR;
    }
    else {
        *K = -RR;
    }

    for (int i = 0; i < 3; ++i) {
        if ((*K)(i, i) < 0.0) {
            K->col(i) = -K->col(i);
            R->row(i) = -R->row(i);
        }
    }

    *T = K->triangularView<Eigen::Upper>().solve(P.col(3));
    if (det_K < 0) {
        *T = -(*T);
    }

    return true;
}

Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d matrix;
    matrix << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1),
        vector(0), 0;
    return matrix;
}

void RotationMatrixToEulerAngles(const Eigen::Matrix3d& R,
    double* rx,
    double* ry,
    double* rz) {
    *rx = std::atan2(R(2, 1), R(2, 2));
    *ry = std::asin(-R(2, 0));
    *rz = std::atan2(R(1, 0), R(0, 0));

    *rx = std::isnan(*rx) ? 0 : *rx;
    *ry = std::isnan(*ry) ? 0 : *ry;
    *rz = std::isnan(*rz) ? 0 : *rz;
}

Eigen::Matrix3d EulerAnglesToRotationMatrix(const double rx,
    const double ry,
    const double rz) {
    const Eigen::Matrix3d Rx =
        Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d Ry =
        Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d Rz =
        Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return Rz * Ry * Rx;
}

Eigen::Quaterniond AverageQuaternions(
    const std::vector<Eigen::Quaterniond>& quats,
    const std::vector<double>& weights) {
    CHECK_EQ(quats.size(), weights.size());
    CHECK_GT(quats.size(), 0);

    if (quats.size() == 1) {
        return quats[0];
    }

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    double weight_sum = 0;

    for (size_t i = 0; i < quats.size(); ++i) {
        CHECK_GT(weights[i], 0);
        const Eigen::Vector4d qvec = quats[i].normalized().coeffs();
        A += weights[i] * qvec * qvec.transpose();
        weight_sum += weights[i];
    }

    A.array() /= weight_sum;

    const Eigen::Matrix4d eigenvectors =
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(A).eigenvectors();

    const Eigen::Vector4d average_qvec = eigenvectors.col(3);

    return Eigen::Quaterniond(
        average_qvec(3), average_qvec(0), average_qvec(1), average_qvec(2));
}

Rigid3d InterpolateCameraPoses(const Rigid3d& cam_from_world1,
    const Rigid3d& cam_from_world2,
    double t) {
    const Eigen::Vector3d translation12 =
        cam_from_world2.translation - cam_from_world1.translation;
    return Rigid3d(cam_from_world1.rotation.slerp(t, cam_from_world2.rotation),
        cam_from_world1.translation + translation12 * t);
}

double CalculateDepth(const Eigen::Matrix3x4d& cam_from_world,
    const Eigen::Vector3d& point3D) {
    const double proj_z = cam_from_world.row(2).dot(point3D.homogeneous());
    return proj_z * cam_from_world.col(2).norm();
}

bool CheckCheirality(const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    std::vector<Eigen::Vector3d>* points3D) {
    CHECK_EQ(points1.size(), points2.size());
    const Eigen::Matrix3x4d proj_matrix1 = Eigen::Matrix3x4d::Identity();
    Eigen::Matrix3x4d proj_matrix2;
    proj_matrix2.leftCols<3>() = R;
    proj_matrix2.col(3) = t;
    const double kMinDepth = std::numeric_limits<double>::epsilon();
    const double max_depth = 1000.0f * (R.transpose() * t).norm();
    points3D->clear();
    for (size_t i = 0; i < points1.size(); ++i) {
        const Eigen::Vector3d point3D =
            TriangulatePoint(proj_matrix1, proj_matrix2, points1[i], points2[i]);
        const double depth1 = CalculateDepth(proj_matrix1, point3D);
        if (depth1 > kMinDepth && depth1 < max_depth) {
            const double depth2 = CalculateDepth(proj_matrix2, point3D);
            if (depth2 > kMinDepth && depth2 < max_depth) {
                points3D->push_back(point3D);
            }
        }
    }
    return !points3D->empty();
}

Rigid3d TransformCameraWorld(const Sim3d& new_from_old_world,
    const Rigid3d& cam_from_world) {
    const Sim3d cam_from_new_world =
        Sim3d(1, cam_from_world.rotation, cam_from_world.translation) *
        Inverse(new_from_old_world);
    return Rigid3d(cam_from_new_world.rotation,
        cam_from_new_world.translation * new_from_old_world.scale);
}

