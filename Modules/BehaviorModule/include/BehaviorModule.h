/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef BehaviorModule_MODULE_H__
#define BehaviorModule_MODULE_H__

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
#include "Evolution.h"
#include "Experiment.h"
#include "Pose.h"
#include "Controller.h"

/**
 * Yarp module used to launch the "in-vivo" RL algorithm
 */
class BehaviorModule:public yarp::os::RFModule {

    yarp::os::Port handlerPort;
    Guardian *myThread;
    Info info;
    string robotName;
    string csvFile;
    std::vector<std::string> partArray;
    Pose * pose;

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();
};

#endif

