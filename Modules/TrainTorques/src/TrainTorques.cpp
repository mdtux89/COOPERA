/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "TrainTorques.h"
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool TrainTorques::configure(yarp::os::ResourceFinder &rf){

    string moduleName = rf.check("name",Value("TrainTorques"),"module name (string)").asString();
    setName(moduleName.c_str());

    //will form stem for port names
    robotName = rf.check("robot",Value("bioloid"),"Robot name (string)").asString();

    nParts = rf.find("njoints").asList()->size();
    velocity = rf.check("velocity",Value(5),"Joint velocity (double)").asDouble();
    torqueLimit = rf.check("torquelimit",Value(400),"Absolute Torque limit (double)").asDouble();
    waitTime = rf.check("waittime",Value(1),"Time to wait for each movement in seconds (int)").asInt();
    motionRange = rf.check("motionrange",Value(10),"Range of motion in degree (int)").asInt();
    torquesFile.open(OUTFILENAME);
    drivers = new PolyDriver[nParts];
    Property config;
    string name = getName();
    for(int i=0;i<nParts;i++){
        nJoints.push_back(rf.find("njoints").asList()->get(i).asInt());
        string part = rf.find("parts").asList()->get(i).asString();
        config.fromString("(device remote_controlboard) (local /local"+name+part+") (remote /" +robotName+"/"+part+") (carrier tcp)");
        if(!drivers[i].open(config)){
            return false;
        }
    }

    /*
     * attach a port of the same name as the module (prefixed with a /) to the module
     * so that messages received from the port are redirected to the respond method
     */
    string handlerPortName = "/" + name;
    if (!handlerPort.open(handlerPortName.c_str())) {
       cout << name << ": Unable to open port " << handlerPortName << endl;
       return false;
    }
    if(!attach(handlerPort)) {
        return false;
    }
    myThread = new LoopThread();
    if(!myThread->start()){
        return false;
    }

    return true ;
}

/**
 * Interrupt the module
 * @return true after closing the handler port
 */
bool TrainTorques::interruptModule(){
   handlerPort.interrupt();
   return true;
}

/**
 * Close the module
 * @return true if all resources are closed properly
 */
bool TrainTorques::close(){
   torquesFile.close();
   for(int i=0;i<nParts;i++){
       if(!drivers[i].close()){
           return false;
       }
   }
   handlerPort.close();
   if(!myThread->stop()){
       return false;
   }

   return true;
}

/**
 * Read the torques from all joints (while moving them) and save them to file
 * @return true if torque values are read successfully
 */
bool TrainTorques::read(){

   for(int i=0;i<nParts;i++){ //for each robot parts

       //create the IPositionControl and ITorqueControl interfaces
       IPositionControl *position;
       ITorqueControl *torque;
       IEncoders *encoders;
       if (!drivers[i].view(position) || !drivers[i].view(torque) || !drivers[i].view(encoders)) {
         return false;
       }

       // two movements will be performed while getting torques
       double t1,t2;
       for(int j=0;j<nJoints[i];j++){
           //first movement
           double enc;
           encoders->getEncoder(j,&enc);
           position->setRefSpeed(j,velocity);
           position->relativeMove(j,motionRange);
           for(int k=0; k<waitTime*10;k++){
               if(!torque->getTorque(j,&t1)){
                   return false;
               }
               Time::delay(0.1);
               if(abs(t1)>torqueLimit){
                   position->stop(j);
                   if(t1 > 0) t1 = torqueLimit;
                   else t1 = -torqueLimit;
                   break;
               }
           }

           //back to initial position
           position->positionMove(j,enc);
           Time::delay(0.1);

           //second movement
           encoders->getEncoder(j,&enc);
           position->relativeMove(j,-motionRange);
           for(int k=0; k<waitTime*10;k++){
               if(!torque->getTorque(j,&t2)){
                   return false;
               }
               Time::delay(0.1);
               if(abs(t2)>torqueLimit){
                   position->stop(j);
                   if(t2 > 0) t2 = torqueLimit;
                   else t2 = -torqueLimit;
                   break;
               }
           }

           //back to initial position
           position->positionMove(j,enc);
           Time::delay(0.1);

           //save values to file
           torquesFile << t1 << ", ";
           torquesFile << t2;
           if(!(i==(nParts-1) && j==(nJoints[i]-1))) torquesFile << ", ";
       }
  }
  torquesFile << endl;
  return true;
}



/**
 * Handles command line commands to get the torque values
 * @return true unless quit is called
 */
bool TrainTorques::respond(const Bottle& command, Bottle& reply){
  string helpMessage =  string(getName().c_str()) +
                        " commands are: \n" +
                         "help \n" +
                         "quit \n" +
                         "get \n" +
                         "multiget <n> \n" +
                         "(where <n> is an integer number) \n";

   reply.clear();

   if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
   }
   else if (command.get(0).asString()=="help") {
      cout << helpMessage;
      reply.addString("ok");
   }
   else if (command.get(0).asString()=="get") {
      if(read()) reply.addString("ok");
      else reply.addString("failed to get torques");
   }
   else if (command.get(0).asString()=="multiget") {
      for(int i=0;i<command.get(1).asInt();i++){
        cout << "reading cycle number: " << (i+1) << endl;
        if(!read()){
            break;
        }
      }
      reply.addString("ok");
   }
   else if (command.get(0).asString()=="set") {
      if (command.get(1).asString()=="vel") {
         velocity = command.get(2).asDouble(); // new velocity
         reply.addString("ok");
      }
   }
   else reply.addString("command not available");
   return true;
}

/**
 * Used to check the module state.
 * @return true if module can continue (always for this module)
 */
bool TrainTorques::updateModule(){
   return true;
}

/**
 * To control the aproximate periodicity of updateModule() class.
 * @return the period
 */
double TrainTorques::getPeriod(){
   return 0.1;
}
