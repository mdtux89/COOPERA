/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef Info__H__
#define Info__H__

#include <string>
#include <vector>
#include <map>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
using namespace std;

/**
 * Class used for passing parameteres to Behaviors threads
 */

class Info {
public:
    static int nParts;
    static vector<int>* nJoints;
    static int nTotalJoints;
    static yarp::dev::PolyDriver* drivers;
    static map<string,int>* starts;
    static map<string,int>* parts;
    static yarp::os::BufferedPort<yarp::os::Bottle>* sensorPort;
    static int velocity;
    static int tiltIndex;
    static int motionWait;
    static int macroWait;
    static bool poseEstimation;
    static string pose;
};

#endif
