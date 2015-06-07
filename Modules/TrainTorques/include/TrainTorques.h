/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef TrainTorques_MODULE_H__
#define TrainTorques_MODULE_H__

#define MINACTION 10

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include "../../common/CommonThreads.h"
#define OUTFILENAME "../resources/data/readings.txt"

/**
 * Yarp module for getting torques while moving all robot joints and save them on file.
 * This module is used to prepare the dataset for the pose estimation classification
 * implemented as a case study for COOPERA.
 */
class TrainTorques:public yarp::os::RFModule {

   std::string robotName;
   yarp::os::Port handlerPort;
   LoopThread *myThread;

   int nParts;
   std::vector<int> nJoints;
   double velocity;
   double torqueLimit;
   int motionRange;
   int waitTime;
   std::ofstream torquesFile;
   yarp::dev::PolyDriver* drivers;
   bool read();

public:

   bool configure(yarp::os::ResourceFinder &rf);
   bool interruptModule();
   bool close();
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod();
   bool updateModule();
};


#endif

