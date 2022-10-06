
#include "crsPose.h"
#include "eigen3/Eigen/Dense"

using Eigen::Vector3d;
using Eigen::Matrix3d;
using spVector3d = std::shared_ptr<Eigen::Vector3d>;
using spMatrix3d = std::shared_ptr<Eigen::Matrix3d>;
using std::make_shared;
using namespace std;

spVector3d transform_coords(spVector3d coords, std::string old_coords, std::string new_coords) {
    PJ_COORD from = proj_coord((*coords)[0], (*coords)[1], (*coords)[2], 0);
    PJ_CONTEXT *C = proj_context_create();
    PJ *P = proj_create_crs_to_crs(C, old_coords.c_str(), new_coords.c_str(), NULL);
    if (P == 0) {
        std::cout << "ERROR CAN'T CREATE PROJ CONVERTER" << std::endl;
        Eigen::Vector3d error;
        return make_shared<Vector3d>(error.setZero());
    }
    PJ_COORD to = proj_trans(P, PJ_FWD, from);

    spVector3d converted_coords = make_shared<Vector3d>(to.v[0], to.v[1], to.v[2]);

    proj_destroy(P);
    proj_context_destroy(C);

    return converted_coords;
}

spMatrix3d calculate_rotation_matrix_to_pose_on_Earth_globe(spVector3d coords) {
    spVector3d pose = transform_coords(make_shared<Eigen::Vector3d>((*coords)[0], (*coords)[1], 1), LONLAT_CRS, ECEF_CRS);
    spVector3d pose_down = transform_coords(make_shared<Eigen::Vector3d>((*coords)[0], (*coords)[1], 0), LONLAT_CRS, ECEF_CRS);
    Eigen::Vector3d z = *pose_down - *pose;
    z.normalize();

    // a parameter to calculate right direction in the certain place on the Earth globe
    double delta_lon = 0.000002;
    spVector3d pose_right = transform_coords(make_shared<Eigen::Vector3d>((*coords)[0] + delta_lon, (*coords)[1], 0), LONLAT_CRS, ECEF_CRS);
    Eigen::Vector3d x = *pose_right - *pose_down;
    x.normalize();

    // a parameter to calculate forward direction in the certain place on the Earth globe

    double delta_lat = 0.000001;
    spVector3d pose_forward = transform_coords(make_shared<Eigen::Vector3d>((*coords)[0], (*coords)[1] - delta_lat, 0), LONLAT_CRS, ECEF_CRS);
    Eigen::Vector3d y = *pose_forward - *pose_down;
    y.normalize();
    spMatrix3d m = make_shared<Eigen::Matrix3d>();
    *m << x, y, z;
    return m;
}