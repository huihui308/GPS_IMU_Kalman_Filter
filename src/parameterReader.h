//
//  parameterReader.hpp
//  Fusion
//
//  Created by Karan on 6/6/18.
//  Copyright � 2018 Karan. All rights reserved.
#ifndef paramReader_hpp
#define paramReader_hpp

#include <map>
#include <vector>
#include <fstream>


class ParameterReader {
public:
    ParameterReader(std::string filename = "parameters.txt");

    std::string getData(std::string key);

    std::map<std::string, std::string> data;
};


#endif /* paramReader_hpp */
