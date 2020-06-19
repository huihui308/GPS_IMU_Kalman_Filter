//
//  main.cpp
//  ExtendedKalmanFilter
//
//  Created by Karan on 4/6/18.
//  Copyright © 2018 Karan. All rights reserved.
//
#include <fstream>
#include <sstream>
#include <iostream>
#include "fusion.h"
#include "run_fusion.h"
#include "glog/logging.h"


int main(int argc, const char * argv[])
{
    std::string dataCsv = "./../data/data.csv";
    std::ifstream dataFl(dataCsv);
    if ( !dataFl.is_open() ) {
        std::cerr << "Failed to open the data file " << dataCsv;
        std::exit(EXIT_FAILURE);
    }

    std::string str;
    std::string course;
    std::string ax, yaw;
    std::string lat, lon;
    std::string yaw_rate;
    std::string timestamp;
    GpsIns gpsIns(true);

    std::getline(dataFl, str); // Skip the first line
    LOG(INFO) << str;
    while (std::getline(dataFl, str)) {
        //LOG(INFO) << str;
        std::istringstream iss(str);
        std::string token;

        while (std::getline(iss, token, ',')) {
            // process each token
            //std::cout << token.size() << " ";
            std::cout << token << " " << std::endl;
        }
        exit(0);
        //std::cout << std::endl;
    }
    dataFl.close();
    LOG(INFO) << "End------";

    return 0;
}


