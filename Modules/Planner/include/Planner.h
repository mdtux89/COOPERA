/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef Planner_MODULE_H__
#define Planner_MODULE_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <fcntl.h>
#include "../../common/CommonThreads.h"
#define PLANPATH "../resources/data/plan.txt"

/**
 * Yarp module for planning actions using the Metric-FF planner.
 * This module does not interact in any way with the robot but can be easily
 * integrated in modules that do. Domain file and problem file must be provided
 * for the planner to work.
 */
class Planner:public yarp::os::RFModule {

    yarp::os::Port handlerPort;
    LoopThread *myThread;
    std::string path,domain,problem;

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();
};

#endif

