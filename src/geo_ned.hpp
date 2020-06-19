//
//  geo_ned.hpp
//  ExtendedKalmanFilter
//
//  Created by Karan on 4/6/18.
//  Copyright © 2018 Karan. All rights reserved.
//
#ifndef geo_ned_hpp
#define geo_ned_hpp

#include <stdio.h>
#include "Eigen/Dense"
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <cmath>


using Eigen::MatrixXd;
using Eigen::VectorXd;


namespace geodectic_converter {


class GeodecticConverter {
public:

    GeodecticConverter();

    bool isInitialised();

    void getReference(
        double& latitude,
        double& longitude,
        double& altitude);

    void intializeReference(
        const double latitude,
        const double longitude,
        const double altitude);

    void geodetic2Ecef(
        const double latitude,
        const double longitude,
        const double altitude,
        double* x,
        double* y,
        double* z);

    void ecef2Geodetic(
        const double x,
        const double y,
        const double z,
        double* latitude,
        double* longitude,
        double* altitude);

    void ecef2Ned(
        const double x,
        const double y,
        const double z,
        double* north,
        double* east,
        double* down);

    void ned2Ecef(
        const double north,
        const double east,
        const double down,
        double* x,
        double* y,
        double* z);

    void geodetic2Ned(
        const double latitude,
        const double longitude,
        const double altitude,
        double* north,
        double* east,
        double* down);

    void ned2Geodetic(
        const double north,
        const double east,
        const double down,
        double* latitude,
        double* longitude,
        double* altitude);

    void geodetic2Enu(
        const double latitude,
        const double longitude,
        const double altitude,
        double* east,
        double* north,
        double* up);

    void enu2Geodetic(
        const double east,
        const double north,
        const double up,
        double* latitude,
        double* longitude,
        double* altitude);

private:
    bool _have_reference;
    double _initial_latitude;
    double _initial_longitude;
    double _initial_altitude;

    double _initial_ecef_x;
    double _initial_ecef_y;
    double _initial_ecef_z;

    Eigen::Matrix3d ned_to_ecef_matrix_;
    Eigen::Matrix3d ecef_to_ned_matrix_;

    double rad2Deg(const double radians);

    double deg2Rad(const double degrees);

    Eigen::Matrix3d nRe(
        const double lat_radians,
        const double lon_radians);

};

};

#endif /* geo_ned_hpp */
