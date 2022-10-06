//
// Created by a.khmarsky on 10.08.2022.
//

#ifndef TRACKER_ROTATION_H
#define TRACKER_ROTATION_H

#include <cmath>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <memory>



std::shared_ptr<Eigen::Matrix3d> get_rotation_matrix(std::shared_ptr<Eigen::Vector3d> rpy, bool degrees = true);

std::shared_ptr<Eigen::Vector3d> get_euler_angles_from_matrix(std::shared_ptr<Eigen::Matrix3d> m, bool degrees = true);


#endif //TRACKER_ROTATION_H
//rpy
