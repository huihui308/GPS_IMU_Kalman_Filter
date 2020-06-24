//
//  utils.cpp
//  EKF
//
//  Created by Karan on 4/9/18.
//  Copyright © 2018 Karan. All rights reserved.
//
#include "utils.h"
#include "parameterReader.h"


EKFParams
getDefaultParams(void)
{
    EKFParams params;
    ParameterReader pd("./../src/parameters.txt");

    params.varGPS = atof(pd.getData( "vargps" ).c_str());
    params.varSpeed = atof(pd.getData( "varspeed" ).c_str());
    params.varYaw = atof(pd.getData( "varyaw" ).c_str());
    params.varAcc = atof(pd.getData( "varaccel" ).c_str());
    params.maxAccel = atof(pd.getData( "maxaccel" ).c_str());
    params.maxTurnRate = atof(pd.getData( "maxturnrate" ).c_str());
    params.maxYawAccel = atof(pd.getData( "maxyawaccel" ).c_str());
    params.xOff = atof(pd.getData("xOff").c_str());
    params.yOff = atof(pd.getData("yOff").c_str());

    return params;
}



