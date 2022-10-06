//
// Created by a.khmarsky on 06.10.2022.
//

#ifndef MATHTEST_CRSPOSE_H
#define MATHTEST_CRSPOSE_H

#define ECEF_CRS "+proj=geocent +ellps=WGS84 +datum=WGS84"
#define LONLAT_CRS "+proj=longlat +ellps=WGS84 +datum=WGS84"

extern "C" {
#include <proj.h>
}

#include <eigen3/Eigen/Dense>
#include <memory>
#include <iostream>

std::shared_ptr<Eigen::Vector3d> transform_coords(std::shared_ptr<Eigen::Vector3d> coords, std::string old_coords, std::string new_coords);

std::shared_ptr<Eigen::Matrix3d> calculate_rotation_matrix_to_pose_on_Earth_globe(std::shared_ptr<Eigen::Vector3d> coords);

#endif //MATHTEST_CRSPOSE_H
