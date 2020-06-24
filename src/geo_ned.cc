//
//  geo_ned.cpp
//  EKF
//
//  Created by Karan on 4/9/18.
//  Copyright © 2018 Karan. All rights reserved.
//
#include "geo_ned.h"


using namespace geodectic_converter;


static const double kSemimajorAxis = 6378137;
static const double kSemiminorAxis = 6356752.3142;
static const double kFlattening = 1 / 298.257223563;
static const double kFirstEccentricitySquared = 6.69437999014 * 0.001;
static const double kSecondEccentricitySquared = 6.73949674228 * 0.001;


GeodecticConverter::GeodecticConverter()
    :m_have_reference(false)
{}

bool GeodecticConverter::isInitialised()
{
    return m_have_reference;
}

void GeodecticConverter::getReference(
    double& latitude,
    double& longitude,
    double& altitude)
{
    latitude = m_initial_latitude;
    longitude = m_initial_longitude;
    altitude = m_initial_altitude;

    return;
}

void GeodecticConverter::intializeReference(
    const double latitude,
    const double longitude,
    const double altitude)
{
    // Save origin
    m_initial_latitude = deg2Rad(latitude);
    m_initial_longitude = deg2Rad(longitude);
    m_initial_altitude = altitude;

    // Compute ECEF of NED origin
    geodetic2Ecef(latitude, longitude, altitude, &m_initial_ecef_x, &m_initial_ecef_y, &m_initial_ecef_z);

    // Compute ECEF to NED and NED to ECEF matrices
    double phiP = atan2(m_initial_ecef_z, sqrt(pow(m_initial_ecef_x, 2) + pow(m_initial_ecef_y, 2)));

    ecef_to_ned_matrix_ = nRe(phiP, m_initial_longitude);
    ned_to_ecef_matrix_ = nRe(m_initial_latitude, m_initial_longitude).transpose();

    m_have_reference = true;
}

void GeodecticConverter::geodetic2Ecef(
    const double latitude,
    const double longitude,
    const double altitude,
    double* x,
    double* y,
    double* z)
{
    // Convert geodetic coordinates to ECEF.
    // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
    double lat_rad = deg2Rad(latitude);
    double lon_rad = deg2Rad(longitude);
    double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
    *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
    *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
    *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
}

void GeodecticConverter::ecef2Geodetic(
    const double x,
    const double y,
    const double z,
    double* latitude,
    double* longitude,
    double* altitude)
{
    // Convert ECEF coordinates to geodetic coordinates.
    // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
    // to geodetic coordinates," IEEE Transactions on Aerospace and
    // Electronic Systems, vol. 30, pp. 957-961, 1994.
    double r = sqrt(x * x + y * y);
    double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
    double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
    double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
    double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
    double S = cbrt(1 + C + sqrt(C * C + 2 * C));
    double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
    double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
    double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q) + sqrt(0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q) - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
    double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
    double V = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
    double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);

    *altitude = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
    *latitude = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
    *longitude = rad2Deg(atan2(y, x));
}

void GeodecticConverter::ecef2Ned(
    const double x,
    const double y,
    const double z,
    double* north,
    double* east,
    double* down)
{
    // Converts ECEF coordinate position into local-tangent-plane NED.
    // Coordinates relative to given ECEF coordinate frame.

    Eigen::Vector3d vect, ret;
    vect(0) = x - m_initial_ecef_x;
    vect(1) = y - m_initial_ecef_y;
    vect(2) = z - m_initial_ecef_z;
    ret = ecef_to_ned_matrix_ * vect;
    *north = ret(0);
    *east = ret(1);
    *down = -ret(2);
}

void GeodecticConverter::ned2Ecef(
    const double north,
    const double east,
    const double down,
    double* x,
    double* y,
    double* z)
{
    // NED (north/east/down) to ECEF coordinates
    Eigen::Vector3d ned, ret;
    ned(0) = north;
    ned(1) = east;
    ned(2) = -down;
    ret = ned_to_ecef_matrix_ * ned;
    *x = ret(0) + m_initial_ecef_x;
    *y = ret(1) + m_initial_ecef_y;
    *z = ret(2) + m_initial_ecef_z;
}

void GeodecticConverter::geodetic2Ned(
    const double latitude,
    const double longitude,
    const double altitude,
    double* north,
    double* east,
    double* down)
{
    // Geodetic position to local NED frame
    double x, y, z;
    geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);
    ecef2Ned(x, y, z, north, east, down);
}

void GeodecticConverter::ned2Geodetic(
    const double north,
    const double east,
    const double down,
    double* latitude,
    double* longitude,
    double* altitude)
{
    // Local NED position to geodetic coordinates
    double x, y, z;
    ned2Ecef(north, east, down, &x, &y, &z);
    ecef2Geodetic(x, y, z, latitude, longitude, altitude);
}

void GeodecticConverter::geodetic2Enu(
    const double latitude,
    const double longitude,
    const double altitude,
    double* east,
    double* north,
    double* up)
{
    // Geodetic position to local ENU frame
    double x, y, z;
    geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);

    double aux_north, aux_east, aux_down;
    ecef2Ned(x, y, z, &aux_north, &aux_east, &aux_down);

    *east = aux_east;
    *north = aux_north;
    *up = -aux_down;
}

void GeodecticConverter::enu2Geodetic(
    const double east,
    const double north,
    const double up,
    double* latitude,
    double* longitude,
    double* altitude)
{
    // Local ENU position to geodetic coordinates

    const double aux_north = north;
    const double aux_east = east;
    const double aux_down = -up;
    double x, y, z;
    ned2Ecef(aux_north, aux_east, aux_down, &x, &y, &z);
    ecef2Geodetic(x, y, z, latitude, longitude, altitude);
}



double GeodecticConverter::rad2Deg(const double radians)
{
    return (radians / M_PI) * 180.0;
}



double GeodecticConverter::deg2Rad(const double degrees)
{
    return (degrees / 180.0) * M_PI;
}

Eigen::Matrix3d GeodecticConverter::nRe(
    const double lat_radians,
    const double lon_radians)
{
    const double sLat = sin(lat_radians);
    const double sLon = sin(lon_radians);
    const double cLat = cos(lat_radians);
    const double cLon = cos(lon_radians);

    Eigen::Matrix3d ret;
    ret(0, 0) = -sLat * cLon;
    ret(0, 1) = -sLat * sLon;
    ret(0, 2) = cLat;
    ret(1, 0) = -sLon;
    ret(1, 1) = cLon;
    ret(1, 2) = 0.0;
    ret(2, 0) = cLat * cLon;
    ret(2, 1) = cLat * sLon;
    ret(2, 2) = sLat;

    return ret;
}


