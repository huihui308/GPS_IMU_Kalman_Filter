//
//  utils.hpp
//  ExtendedKalmanFilter
//
//  Created by Karan on 4/6/18.
//  Copyright © 2018 Karan. All rights reserved.
//

#ifndef utils_hpp
#define utils_hpp

#include "Eigen/Dense"


// Read these parameters from the file
struct EKFParams {
    double varGPS, varSpeed, varYaw, varAcc;
    double maxAccel, maxTurnRate, maxYawAccel;
    double xOff, yOff;
};


extern
EKFParams
getDefaultParams(void);


#endif /* utils_hpp */
