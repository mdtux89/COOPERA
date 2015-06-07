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
#include <ctime>

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

// iCub
// #include <iCub/iKin/iKinInv.h>


// GSL
#include <gsl/gsl_math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_fit.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_rng.h>

//Boost
#include <boost/graph/graphviz.hpp>

// Jason
#include <json/json.h>
#include <Sampler.h>

#define EPSILON 1e-15
#define ACTION_DIM 2
#define PRIMITIVE_ACTION_DIM 1
#define STATE_DIM 2
#define MINUS_INF_Q (-50.0)
#define INVALID_STATE -1
#define W0 0
#define WBAD 1
#define WOK 2


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
    double totTime;
    bool hasCollided;
    int gridSizeX;
    int gridSizeY;
    UniformSampler *uniformSampler;

    ~Config()
    {
        instanceFlag = false;
        delete uniformSampler;
    }

private:
    static bool instanceFlag;
    static bool isParsed;
    static Config *single;      // Trivia: This is thread-safe only for GNU compiler and also for c++11,
                                // we dont give damn for airhockey as of now since its serial!
    Config()
    {
        uniformSampler = new UniformSampler(static_cast<unsigned int>(time(NULL)));
        Json::Reader rd;
        std::ifstream tst(PATH2CONFIG, std::ifstream::binary);
        isParsed = rd.parse(tst,root,false);
        hasCollided = false;

        int rangeX  = root["State"]["UnsafeRange"].asDouble() *
                gridSizeX;
        int rangeY = root["State"]["UnsafeRange"].asDouble() *
                gridSizeY;
    }
};



bool file_is_empty(std::ifstream& pFile);
#endif // CONFIGURATION_H
