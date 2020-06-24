//
//  main.cpp
//  ExtendedKalmanFilter
//
//  Created by Karan on 4/6/18.
//  Copyright © 2018 Karan. All rights reserved.
//
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "utils.h"
#include "fusion.h"
#include "run_fusion.h"
#include "glog/logging.h"


using namespace std;
static std::vector<DataPoint> all_sensor_data;


static inline int32_t
_parse_line_msg(
    std::string& aiqStr,
    const int32_t aicLineCnt)
{
    int32_t cnt = 0;
    std::string token;
    long long timestamp = 0;
    double ax = 0.0, ay = 0.0, az = 0.0;
    double speed = 0.0, heading = 0.0, yawrate = 0.0;
    double latitude = 0.0, longitude = 0.0, altitude = 0.0;

    //LOG(INFO) << str;
    //std::cout << std::endl;
    std::istringstream iss(aiqStr);
    while (std::getline(iss, token, ',')) {
        // process each token
        //std::cout << token.size() << " ";
        //std::cout << token << " " << std::endl;
        switch (cnt++) {
            case 2:
                timestamp = stoll(token);
                //std::cout << "timestamp: " << timestamp << std::endl;
                break;
            case 3:
                ax = stod(token);
                //std::cout << "ax: " << ax << std::endl;
                break;
            case 4:
                ay = stod(token);
                //std::cout << "ay: " << ay << std::endl;
                break;
            case 5:
                az = stod(token);
                //std::cout << "az: " << az << std::endl;
                break;
            case 8:
                yawrate = stod(token);
                //std::cout << "yawrate: " << yawrate << std::endl;
                break;
            case 12:
                speed = stod(token);
                //std::cout << "speed: " << speed << std::endl;
                break;
            case 13:
                heading = stod(token);
                //std::cout << "heading: " << heading << std::endl;
                break;
            case 14:
                latitude = stod(token);
                //std::cout << "latitude: " << latitude << std::endl;
                break;
            case 15:
                longitude = stod(token);
                //std::cout << "longitude: " << longitude << std::endl;
                break;
            case 16:
                altitude = stod(token);
                //std::cout << "altitude: " << altitude << std::endl;
                break;
        }/* end switch (cnt++) { */
    }/* end while (std::getline(iss, token, ',')) { */
    //------
    DataPoint sensor_data;
    if (0 == (aicLineCnt % 10)) {
        VectorXd gps_vec(GPS_VECTOR_SIZE);
        gps_vec << latitude, longitude, heading, speed;

        sensor_data.set(timestamp, DataPointType::GPS, gps_vec);
    } else {
        VectorXd imu_vec(IMU_VECTOR_SIZE);
        imu_vec << yawrate, ax;

        sensor_data.set(timestamp, DataPointType::IMU, imu_vec);
    }
    all_sensor_data.push_back(sensor_data);

    return 0;
}

static inline int32_t
_ukf_gps_imu(void)
{
    int32_t i = 0;
    Fusion* filter;
    EKFParams params;
    long long timestamp;
    VectorXd measurement;
    DataPoint sensor_data;
    std::string sensor_name;
    DataPointType sensor_type;

    try {
        params = getDefaultParams();
    } catch (const ifstream::failure& e) {
        cout << "Exception opening/reading parameter file";
    }
    //------
    filter = new Fusion(params.maxAccel, params.maxTurnRate, params.maxYawAccel, params.varGPS, params.varSpeed, params.varYaw, params.varAcc, params.xOff, params.yOff, true);
    //------
    for (i = 0; i < all_sensor_data.size(); i++) {
        //LOG(INFO) << "i:" << i;
        sensor_data = all_sensor_data[i];
        timestamp = sensor_data.get_timestamp();
        sensor_type = sensor_data.get_type();
        sensor_name = ((sensor_type == DataPointType::IMU) ? "imu" : "gps");
        //measurement = sensor_data.get();
        measurement = sensor_data.get_state();
        //filter->process(sensor_data);
    }
    //------
    delete filter;
    filter = nullptr;

    return 0;
}

int32_t main(int32_t argc, const char *argv[])
{
    std::string str;
    int32_t lineCnt = 0;

    // get data from file------
    std::string dataCsv = "./../data/data.csv";
    std::ifstream dataFl(dataCsv);
    if ( !dataFl.is_open() ) {
        std::cerr << "Failed to open the data file!!!!" << dataCsv;
        std::exit(EXIT_FAILURE);
    }
    // Skip the first line
    std::getline(dataFl, str);
    LOG(INFO) << str;
    while (std::getline(dataFl, str)) {
        _parse_line_msg(str, lineCnt++);
    }
    dataFl.close();
    LOG(INFO) << "all_sensor_data len: " << all_sensor_data.size();
    // ekf fusion------
    _ukf_gps_imu();
    //------
    LOG(INFO) << "End------";

    return 0;
}


