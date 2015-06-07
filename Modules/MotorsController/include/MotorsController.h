/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef MotorsController_MODULE_H__
#define MotorsController_MODULE_H__

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <Pose.h>
#include <SharedInfo.h>

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include "Experiment.h"
#include "../../common/CommonThreads.h"

/**
 * Yarp module for working with the robot motors.
 * It allows to interact with each motor independently as well as to trigger complex behaviors. In addition
 * it updates a log file with the encoders and the sensor readings.
 * If the battery status becomes critical a audio signal is generated to alarm the user.
 */
class MotorsController: public yarp::os::RFModule {

   std::string robotName;
   Pose *pose;
   yarp::os::Port handlerPort;
   yarp::os::Port sensorPort;
   std::ofstream log;
   bool keepLog;
   LoopThread *myThread;

   std::vector<std::string> partArray; // maps index to robot part
   std::string csvFile,logFile;
   SharedInfo shared;

   bool copy(std::ifstream *ifile);
   int poseEstimation();
   std::vector<int> encodersFromTask(std::string task);
   bool runWithSarsa(std::string task);
   bool updateLog();
   std::vector<int> getData();

public:

   bool configure(yarp::os::ResourceFinder &rf);
   bool interruptModule();
   bool close();
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod();
   bool updateModule();
};

#endif


