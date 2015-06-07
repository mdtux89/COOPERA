/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef Spatials_MODULE_H__
#define Spatials_MODULE_H__

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include "../../common/CommonThreads.h"
#define OUTFILENAME "../resources/data/readings.txt"

/**
 * Yarp module for reading spatials feedback (provided by phidget sensor in COOPERA).
 * It allows to detect spatials from the robot and save them to file.
 */
class Spatials:public yarp::os::RFModule {

   std::string robotName;
   yarp::os::Port handlerPort;
   LoopThread *myThread;
   yarp::os::Port sensorPort;
   std::ofstream outFile;
   int average;
   int nSensors;
   bool detect();

public:

   bool configure(yarp::os::ResourceFinder &rf);
   bool interruptModule();
   bool close();
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod();
   bool updateModule();
};


#endif

