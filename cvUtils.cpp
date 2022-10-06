//
// Created by shakhinn on 09.08.22.
//

#include "cvUtils.h"

using std::make_shared;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using spMatrix3d = std::shared_ptr<Matrix3d>;
using spVector3d = std::shared_ptr<Vector3d>;
using spVector2d = std::shared_ptr<Vector2d>;

Eigen::Vector3d pixel2local(const Eigen::Vector3d &pixel_coords, const Eigen::Matrix3d &intrinsic_matrix) {
    return intrinsic_matrix.inverse() * pixel_coords;
}

Eigen::Vector3d local2world(const Eigen::Vector3d &local_coords, const Eigen::Matrix3d &rotation_matrix,
                            const Eigen::Vector3d &local_view_position) {
    return rotation_matrix * local_coords + local_view_position;
}

Eigen::Vector3d world2local(const Eigen::Vector3d &world_coords, Eigen::Matrix3d rotation_matrix,
                            const Eigen::Vector3d &local_view_position) {
    return rotation_matrix.transpose() * (world_coords - local_view_position);
}

Eigen::Vector3d norm_angle(double angle, double left, double right, bool degrees) {
    if (!degrees) {
        angle *= 360 / M_PI;
        left *= 360 / M_PI;
        right *= 360 / M_PI;
    }
    angle = std::remainder(angle, 360);
    left = std::remainder(left, 360);
    right = std::remainder(right, 360);
    while (angle >= left) {
        angle -= 360;
    }
    angle += 360;
    while (right >= left) {
        right -= 360;
    }
    right += 360;
    return {angle, left, right};
}

bool angle_less_bounds(double angle, spVector2d left_right, bool degrees) {
    if (left_right == nullptr)
        return false;
    Eigen::Vector3d angle_left_right = norm_angle(angle, (*left_right)[0], (*left_right)[1], degrees);
    double middle = (angle_left_right[1] + angle_left_right[2]) / 2;
    middle += 180;
    Eigen::Vector3d angle_middle_left = norm_angle(angle, middle, angle_left_right[1]);
    return (angle_middle_left[1] <=  angle_middle_left[0] && angle_middle_left[0] < angle_middle_left[2] - 1e-6);
}

bool angle_greater_bounds(double angle, spVector2d left_right, bool degrees) {
    if (left_right == nullptr)
        return false;
    Eigen::Vector3d angle_left_right = norm_angle(angle, (*left_right)[0], (*left_right)[1], degrees);
    double middle = (angle_left_right[1] + angle_left_right[2]) / 2;
    middle += 180;
    Eigen::Vector3d angle_right_middle = norm_angle(angle, angle_left_right[2], middle);
    return (angle_right_middle[1] + 1e-6 < angle_right_middle[0] && angle_right_middle[0] <= angle_right_middle[2]);

}
