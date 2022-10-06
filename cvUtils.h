//
// Created by shakhinn on 09.08.22.
//

#ifndef STABILIZER_SYNC_CPP_CV_UTILS_H
#define STABILIZER_SYNC_CPP_CV_UTILS_H

#include <cmath>
#include <memory>
#include "Eigen/Dense"

Eigen::Vector3d pixel2local(const Eigen::Vector3d &pixel_coords, const Eigen::Matrix3d &intrinsic_matrix);

Eigen::Vector3d local2world(const Eigen::Vector3d &local_coords, const Eigen::Matrix3d &rotation_matrix,
                            const Eigen::Vector3d &local_view_position);

Eigen::Vector3d world2local(const Eigen::Vector3d &world_coords, Eigen::Matrix3d rotation_matrix,
                            const Eigen::Vector3d &local_view_position);

Eigen::Vector3d norm_angle(double angle, double left, double right, bool degrees = true);

bool angle_less_bounds(double angle, std::shared_ptr<Eigen::Vector2d> left_right, bool degrees = true);

bool angle_greater_bounds(double angle, std::shared_ptr<Eigen::Vector2d> left_right, bool degrees = true);


#endif //STABILIZER_SYNC_CPP_CV_UTILS_H