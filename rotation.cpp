//
// Created by shakhinn on 11.08.22.
//

#include "rotation.h"
#include <math.h>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using spMatrix3d = std::shared_ptr<Matrix3d>;
using spVector3d = std::shared_ptr<Vector3d>;
using std::make_shared;

spMatrix3d get_rotation_matrix(spVector3d rpy, bool degrees) {

    if (degrees)
        rpy = make_shared<Vector3d>((*rpy) * M_PI / 180);

    return make_shared<Matrix3d>(Eigen::AngleAxisd((*rpy)[2], Eigen::Vector3d::UnitZ()) * // roll  Z - YAW
                                 Eigen::AngleAxisd((*rpy)[1], Eigen::Vector3d::UnitX()) * // pitch X - PITCH
                                 Eigen::AngleAxisd((*rpy)[0], Eigen::Vector3d::UnitY()));  // yaw   Y - ROLL
}

spVector3d get_euler_angles_from_matrix(spMatrix3d m, bool degrees) {
    spVector3d vec = make_shared<Vector3d>(m->eulerAngles(2, 0, 1));
    vec->reverseInPlace();

    if (degrees)
        *vec *= 180 / M_PI;
    return vec;
}
