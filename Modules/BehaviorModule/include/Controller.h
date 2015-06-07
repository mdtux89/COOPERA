/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef Controller__H__
#define Controller__H__

#define TIMEOUT_MOTION 10

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <Info.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

#define VOLTAGE_ERROR 0
#define ANGLE_ERROR 1
#define OVERHEATING_ERROR 2
#define OVERLOAD_ERROR 5
#define UNKNOWN 7

#define FEEDBACK_RESET 3
#define OVERHEATING_RESET 4
#define OVERLOAD_RESET 5

#define SWITCHUNKNOWN 600

/**
 * Thread that actually perform the motors movements
 */
class MotorsThread
{
private:
    PolyDriver* driver;
    vector<double> encoders;
    boost::thread* api_caller;
    static int lastError;

public:
    MotorsThread(PolyDriver* driver, vector<double> encoders);
    MotorsThread(PolyDriver* driver, double encoder);
    void motion();
    bool join();
};

/**
 * Used to prevent overload errors by detecting high torques on the motors
 */
class TorqueControl: public Thread {
public:
    void run(){
       while(true) {
           for(int i=0;i<Info::nParts;i++){ //for each robot parts
               ITorqueControl *torques;
               IPositionControl *position;
               if (!Info::drivers[i].view(torques) || !Info::drivers[i].view(position)) {
                 return;
               }
               for(int j = 0; j < (*Info::nJoints)[i]; j++){
                   double v;
                   torques->getTorque(j,&v);
                   if ( v > 300) {
                       position->stop();
                       return;
                   }
               }
           }
       }
    }
};


/**
 * Thread used to check battery status, detect robot falling, avoid high torque errors and keep track of the
 * last issued hardware fault
 */
class Guardian : public Thread
{
private:
    static vector<int> lastState;
    static bool overload;
    static yarp::os::Semaphore lock;
    static int lastError;
    string name;
    double batteryLimit, accLimit;
    int batteryIndex, accIndex;
    string batteryAlarm;

public:
   bool fall;

   /**
    * Allow to get the likely cause of the last hardware fault
    * @retrun the error code
    */
   static int getLastError() {
       if (Guardian::lastError == ANGLE_ERROR || Guardian::lastError == UNKNOWN) return FEEDBACK_RESET;
       if (Guardian::lastError == OVERHEATING_ERROR) return OVERHEATING_RESET;
       if (Guardian::lastError == OVERLOAD_ERROR) return OVERLOAD_RESET;
       else return -1;
   }


   Guardian(string name, int batteryIndex, double batteryLimit, string batteryAlarm, int accIndex, double accLimit);
   void run();
};
#endif


