/************************************************************************
    > File Name: src/parameterReader.cc
    > Author: david
    > Mail: 305573571@qq.com
    > Created Time: 2020年06月18日 星期四 13时52分48秒
 ************************************************************************/
#include <iostream>
#include "parameterReader.h"


using namespace std;


ParameterReader::ParameterReader(string filename)
{
    ifstream fin(filename.c_str());

    if ( !fin ) {
        std::cerr << "parameter file does not exist." << endl;
        return;
    }
    while( !fin.eof() ) {
        cout << "------ Reading in Parameter File...\r\n";
        string str;
        getline(fin, str);
        cout << "   Line Read: " << str << endl;
        if (str[0] == '#') {
            continue;
        }

        int32_t pos = str.find("=");
        if (pos == -1) {
            cout << "pos found = -1 ---- Continuing loop...\r\n";
            continue;
        }
        string key = str.substr( 0, pos );
        string value = str.substr( pos+1, str.length() );

        this->data[key] = value;

        cout << "   Key Found with Value: " << key << " -> " << value << endl;
        cout << "   Stored data mapping:   key (" << key << ") ------- value(" << this->data[key] << ")\r\n";

        if ( !fin.good() ) {
            cout<<"\r\n";
            break;
        }
    }
}


string ParameterReader::getData(string key)
{
    map<string, string>::iterator iter;

    iter = this->data.find(key.c_str());
    std::cout << "Searching for key (" << key.c_str() << ") => " << this->data[key] << '\n';
    if (iter == this->data.end()) {
        std::cerr<<" Parameter name "<< key <<" not found!"<<endl;
        return string("NOT_FOUND");
    }

    return iter->second;
}


