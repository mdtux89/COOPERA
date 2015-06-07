/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <new>      // To set handler for new
#include <limits>   // To set precision for streams
#include <iomanip>  // Some more on precision
#include <fstream>
#include <ostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <stack>
#include <map>
#include <tr1/unordered_map>

// YARP
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

//Boost
#include <boost/graph/graphviz.hpp>

// Jason
#include <json/json.h>
#include <Sampler.h>

#define EPSILON 1e-15

inline timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

namespace state
{
    enum {x,y};
}
namespace action
{
    enum {dX,dY};
}

#define PATH2CONFIG "../resources/conf/Config.json"

/**
 * Utilities used to configure parameters in Sarsa RL experiments.
 */
class Config
{

public:
    static Config* instance();
    Json::Value root;

    ~Config()
    {
        instanceFlag = false;
    }

private:
    static bool instanceFlag;
    static bool isParsed;
    static Config *single;      // Trivia: This is thread-safe only for GNU compiler and also for c++11,
                                // we dont give damn for airhockey as of now since its serial!
    Config()
    {
        Json::Reader rd;
        std::ifstream tst(PATH2CONFIG, std::ifstream::binary);
        isParsed = rd.parse(tst,root,false);
    }
};



bool file_is_empty(std::ifstream& pFile);
#endif // CONFIGURATION_H
