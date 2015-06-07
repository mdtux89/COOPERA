/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef SHAREDINFO_H
#define SHAREDINFO_H

#include <yarp/dev/all.h>
#include "../../common/CommonThreads.h"
#include <map>
using namespace yarp::dev;

/**
 * Class used for sharing parameteres between MotorsController and Pose.
 */
class SharedInfo {
public:
    int nParts;
    vector<int> nJoints;
    map<string,int> parts; // maps robot part to index
    double velocity;
    double torqueLimit;
    double joinTime;
    int waitTime;
    int motionRange;
    vector<int> sensorIndexes;
    Port sensorPort;
    PolyDriver* drivers;
    MotionThread* threads;
};

#endif
