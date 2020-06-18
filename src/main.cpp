//
//  main.cpp
//  ExtendedKalmanFilter
//
//  Created by Karan on 4/6/18.
//  Copyright © 2018 Karan. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include "fusion.hpp"

int main(int argc, const char * argv[])
{
    std::string dataCsv = "./../src/data.csv";


    std::ifstream dataFl(dataCsv);
    if ( !dataFl.is_open() ) {
        std::cerr << "Failed to open the data file " << dataCsv;
        std::exit(EXIT_FAILURE);
    }

    std::string timestamp;
    std::string ax;
    std::string yaw_rate;
    std::string yaw;
    std::string course;
    std::string lat;
    std::string lon;

    std::string str;
    std::getline(dataFl, str); // Skip the first line

    while(std::getline(dataFl, str))
    {
        std::istringstream iss(str);
        std::string token;

        while (std::getline(iss, token, ','))
        {
            // process each token
            std::cout << token.size() << " ";
        }
        std::cout << std::endl;
    }

    dataFl.close();

    return 0;
}
