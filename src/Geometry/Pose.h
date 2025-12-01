#pragma once
#include "Rigid3D.h"
#include "Sim3D.h"
#include "../Base/types.h"
#include "../Base/logging.h"
#include "../Scene/Camera.h"
#include "../Scene/CameraModel.h"

#include <array>
#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>

// Compute the closes rotation matrix with the closest Frobenius norm by setting
// the singular values of the given matrix to 1.
Eigen::Matrix3d ComputeClosestRotationMatrix(const Eigen::Matrix3d& matrix);

// Decompose projection matrix into intrinsic camera matrix, rotation matrix and
// translation vector. Returns false if decomposition fails.
bool DecomposeProjectionMatrix(const Eigen::Matrix3x4d& proj_matrix,
    Eigen::Matrix3d* K,
    Eigen::Matrix3d* R,
    Eigen::Vector3d* T);

// Compose the skew symmetric cross product matrix from a vector.
Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& vector);

// Convert 3D rotation matrix to Euler angles.
//
// The convention `R = Rx * Ry * Rz` is used,
// using a right-handed coordinate system.
//
// @param R              3x3 rotation matrix.
// @param rx, ry, rz     Euler angles in radians.
void RotationMatrixToEulerAngles(const Eigen::Matrix3d& R,
    double* rx,
    double* ry,
    double* rz);

// Convert Euler angles to 3D rotation matrix.
//
// The convention `R = Rz * Ry * Rx` is used,
// using a right-handed coordinate system.
//
// @param rx, ry, rz     Euler angles in radians.
//
// @return               3x3 rotation matrix.
Eigen::Matrix3d EulerAnglesToRotationMatrix(double rx, double ry, double rz);

// Compute the weighted average of multiple Quaternions according to:
//
//    Markley, F. Landis, et al. "Averaging quaternions."
//    Journal of Guidance, Control, and Dynamics 30.4 (2007): 1193-1197.
//
// @param quats         The Quaternions to be averaged.
// @param weights       Non-negative weights.
//
// @return              The average Quaternion.
Eigen::Quaterniond AverageQuaternions(
    const std::vector<Eigen::Quaterniond>& quats,
    const std::vector<double>& weights);

// Linearly interpolate camera pose.
Rigid3d InterpolateCameraPoses(const Rigid3d& cam_from_world1,
    const Rigid3d& cam_from_world2,
    double t);

// Perform cheirality constraint test, i.e., determine which of the triangulated
// correspondences lie in front of of both cameras. The first camera has the
// projection matrix P1 = [I | 0] and the second camera has the projection
// matrix P2 = [R | t].
//
// @param R            3x3 rotation matrix of second projection matrix.
// @param t            3x1 translation vector of second projection matrix.
// @param points1      First set of corresponding points.
// @param points2      Second set of corresponding points.
// @param points3D     Points that lie in front of both cameras.
bool CheckCheirality(const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    std::vector<Eigen::Vector3d>* points3D);

Rigid3d TransformCameraWorld(const Sim3d& new_from_old_world,
    const Rigid3d& cam_from_world);
