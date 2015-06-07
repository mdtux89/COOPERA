/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef Encoders_MODULE_H__
#define Encoders_MODULE_H__

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
 * Yarp module for reading encoders from all robot joints and using them to replicate robot positions
 * and movements.
 * It allows to detect encoders from the robot and save them to file as well as replicate movements by following
 * encoders read from file.
 */
class Encoders:public yarp::os::RFModule {

   std::string robotName;
   yarp::os::Port handlerPort;
   LoopThread *myThread;

   int nParts;
   std::vector<int> nJoints;
   double velocity;
   std::string encodersNameFile;
   std::ofstream outFile;
   std::ifstream inFile;
   yarp::dev::PolyDriver* drivers;
   MotionThread* threads;
   bool detect();
   bool perform();

public:

   bool configure(yarp::os::ResourceFinder &rf);
   bool interruptModule();
   bool close();
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod();
   bool updateModule();
};


#endif

