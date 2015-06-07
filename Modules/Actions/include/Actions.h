/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef Actions_MODULE_H__
#define Actions_MODULE_H__

#define MINACTION 10

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <map>

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include "../../common/CommonThreads.h"
#define OUTFILENAME "../resources/data/readings.txt"

/**
 * Yarp module for detecting actions and replicate them.
 * It allows to detect actions and save them to file as well as to replicate actions previously saved to file.
 */
class Actions:public yarp::os::RFModule {

   std::string robotName;
   yarp::os::Port handlerPort;
   LoopThread *myThread;

   int nParts;
   std::vector<int> nJoints;
   std::string fileName;
   std::ofstream outFile;
   std::ifstream inFile;
   std::vector<std::string> partArray; // maps index to robot part
   std::map<std::string,int> parts; // maps robot part to index
   yarp::dev::PolyDriver* drivers;
   double velocity;
   std::vector<std::vector<double> > old;
   int count;
   bool perform();
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

