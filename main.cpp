#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <chrono>

#include "crsPose.h"
#include "rotation.h"
#include "cvUtils.h"
using namespace Eigen;
using namespace std;

// fx = 405., fy = 405., cx = 639., cy = 359.

void find_target_coords() {

    Matrix3d intrinsic_matrix;
    intrinsic_matrix << 405., 0., 639.,
                        0., 405., 359.,
                        0., 0., 1.;

    double lon = 59.8352;
    double lat = 31.4826 ;
    double alt = 125 - 61;

    double realR = 0.;
    double realP = 0.;
    double realY = 0.;
    double r = 0.;
    double p = 0.;
    double y = 0.;

    auto x_coord = 500;
    auto y_coord = 500;

    auto t_rotated_drone_to_camera = std::make_shared<Eigen::Vector3d>(0,0,1.5);

    auto uav_lonlat = make_shared<Vector3d>(lon, lat, alt);
    double cam_alt = alt - 1.5;
    auto uav_rpy = make_shared<Vector3d>(-r, p, y);  // 0 0 0



    auto cam_rpy = make_shared<Vector3d>(-realR, realP, realY);

    *cam_rpy += Vector3d {0, 0, -12};

    auto t_world_to_math_point = transform_coords(uav_lonlat, LONLAT_CRS, ECEF_CRS);
    auto R_world_to_math_point = calculate_rotation_matrix_to_pose_on_Earth_globe(uav_lonlat);

    auto R_math_point_to_camera = get_rotation_matrix(cam_rpy);

    auto t_math_point_to_rotated_drone = make_shared<Vector3d>(0, 0, 0);
    auto R_math_point_to_rotated_drone = get_rotation_matrix(uav_rpy);

    // t_rotated_drone_to_camera = make_shared<Vector3d>(config.get_camera_offset());
    auto R_rotated_drone_to_camera = make_shared<Matrix3d>(
            R_math_point_to_rotated_drone->transpose() * (*R_math_point_to_camera));

    auto *picture_coords = new Vector3d(x_coord, y_coord, 1);
    auto target_in_pixel = picture_coords; //  picture coords x,y,1
    //target_in_pixel = make_shared<Vector3d>(639, 359, 1);
    auto target_in_camera_normed = make_shared<Vector3d>(pixel2local(*target_in_pixel, intrinsic_matrix));
    auto target_in_rotated_drone_normed = make_shared<Vector3d>(
            local2world(*target_in_camera_normed, *R_rotated_drone_to_camera, *t_rotated_drone_to_camera));
    auto target_in_math_point_normed = make_shared<Vector3d>(
            local2world(*target_in_rotated_drone_normed, *R_math_point_to_rotated_drone,
                        *t_math_point_to_rotated_drone));
    /*
    //t_world_to_camera = std::make_shared<Eigen::Vector3d>(transform_coords(cam_lonlat, LATLON_CRS, ECEF_CRS));
    //double cam_alt = cam_lonlat[2];
    double cam_alt = 9;//dataForController->uav->uav_status->get_altitude();
    //std::cout.precision(15);
    //std::cout<<"DRONE: "<<*t_world_to_math_point<<" "<<uav_lonlat<<std::endl;
    //R_math_point_to_rotated_drone = std::make_shared<Eigen::Matrix3d>(get_rotation_matrix(uav_rpy));
    //Eigen::Vector3d target_in_rotated_drone = local2world(target_in_camera, *R_rotated_drone_to_camera,
    //                                                      *t_rotated_drone_to_camera);
    //Eigen::Vector3d target_in_math_point = local2world(target_in_rotated_drone, *R_math_point_to_rotated_drone,
    //                                                   *t_math_point_to_rotated_drone);
    //Eigen::Vector3d target_in_math_point = local2world(target_in_camera, *R_math_point_to_camera, *t_rotated_drone_to_camera);
     */

    Vector3d cam_pos_in_rotated_drone = *t_rotated_drone_to_camera;
    Vector3d cam_pos_in_mat_point = local2world(cam_pos_in_rotated_drone, *R_math_point_to_rotated_drone,
                                                *t_math_point_to_rotated_drone);
    Eigen::Vector3d target_view_in_math_point = *target_in_math_point_normed - cam_pos_in_mat_point;
    double k = cam_alt / target_view_in_math_point.dot(Vector3d(0, 0, 1));
    auto target_in_camera = make_shared<Vector3d>((*target_in_camera_normed) * k / (target_in_camera_normed->norm()));
    auto target_in_rotated_drone = make_shared<Vector3d>(
            local2world(*target_in_camera, *R_rotated_drone_to_camera, *t_rotated_drone_to_camera));
    auto target_in_math_point = make_shared<Vector3d>(
            local2world(*target_in_rotated_drone, *R_math_point_to_rotated_drone, *t_math_point_to_rotated_drone));
    auto target_in_world = make_shared<Vector3d>(
            local2world(*target_in_math_point, *R_world_to_math_point, *t_world_to_math_point));
    //std::cout.precision(15);
    //std::cout<<"TARGET: "<<*target_in_world<<" "<<transform_coords(*target_in_world, ECEF_CRS, LONLAT_CRS)<<std::endl;
    auto time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    auto secs = time / 1000000000;
    auto millisecs = (time - secs * 1000000000) / 1000000;
    auto microsecs = (time - secs * 1000000000 - millisecs * 1000000) / 1000;
    auto nanosecs = time - secs * 1000000000 - millisecs * 1000000 - microsecs * 1000;
    //int old_precision = logfile->precision();
    std::cout << std::endl
             << "--------------------------------------------------------------------------" << std::endl
             << "find_target_coords" << std::endl
             << "time: " << secs << "s " << millisecs << "ms " << microsecs << "mks " << nanosecs << "ns" << std::endl
            // << std::setprecision(15)
             << "uav_lonlat: " << std::endl << uav_lonlat->transpose() << std::endl
            // << std::setprecision(old_precision)
             << "cam_alt: " << std::endl << cam_alt << std::endl
             << "uav_rpy: " << std::endl << uav_rpy->transpose() << std::endl
             << "cam_rpy: " << std::endl << cam_rpy->transpose() << std::endl
             << "t_world_to_math_point: " << std::endl << t_world_to_math_point->transpose() << std::endl
             << "R_world_to_math_point: " << std::endl << *R_world_to_math_point << std::endl
             << "R_math_point_to_camera: " << std::endl << *R_math_point_to_camera << std::endl
             << "t_math_point_to_rotated_drone: " << std::endl << t_math_point_to_rotated_drone->transpose()
             << std::endl
             << "R_math_point_to_rotated_drone" << std::endl << *R_math_point_to_rotated_drone << std::endl
             << "t_rotated_drone_to_camera: " << std::endl << t_rotated_drone_to_camera->transpose() << std::endl
             << "R_rotated_drone_to_camera: " << std::endl << *R_rotated_drone_to_camera << std::endl
             << "target_in_pixel: " << std::endl << target_in_pixel->transpose() << std::endl
             << "target_in_camera_normed: " << std::endl << target_in_camera_normed->transpose() << std::endl
             << "target_in_rotated_drone_normed: " << std::endl << target_in_rotated_drone_normed->transpose()
             << std::endl
             << "target_in_math_point_normed: " << std::endl << target_in_math_point_normed->transpose() << std::endl
             << "k: " << k << std::endl
             << "target_in_camera: " << std::endl << target_in_camera->transpose() << std::endl
             << "target_in_rotated_drone" << std::endl << target_in_rotated_drone->transpose() << std::endl
             << "target_in_math_point" << std::endl << target_in_math_point->transpose() << std::endl
             << "target_in_world" << std::endl << target_in_world->transpose() << std::endl;
}