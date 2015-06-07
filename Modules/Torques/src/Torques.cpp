/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "Torques.h"
using namespace std;
using namespace yarp::dev;
using namespace yarp::os;

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool Torques::configure(yarp::os::ResourceFinder &rf){

   string moduleName = rf.check("name",Value("Torques"),"module name (string)").asString();
   setName(moduleName.c_str());

   //will form stem for port names
   robotName = rf.check("robot",Value("bioloid"),"Robot name (string)").asString();
   nParts = rf.find("njoints").asList()->size();
   drivers = new PolyDriver[nParts];
   outFile.open(OUTFILENAME);
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
   if(!attach(handlerPort)){
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
bool Torques::interruptModule(){
   handlerPort.interrupt();
   return true;
}

/**
 * Read torques from all robot joints and save them in a file
 * @return true if the torque values are read successfully
 */
bool Torques::detect(){

   while(true){
   vector<int> val;
   for(int i=0;i<nParts;i++){ //for each robot parts

       //create the ITorqueControl interfaces
       ITorqueControl *torques;
       if (!drivers[i].view(torques)) {
         return false;
       }
       //get torques for this robot part (one for each joint)
       for(int j=0;j<nJoints[i];j++){
           double v;
           torques->getTorque(j,&v);
           val.push_back((int)v);
       }
   }

   //save torques to file
   /*for(int j=0;j<(val.size()-1);j++){
       outFile << val[j] << ", ";
   }
   outFile << val[val.size()-1];
   outFile << endl;*/
   for(int j=0;j<(val.size()-1);j++){
       cout << val[j] << ", ";
   }
   cout << endl;
   }
   return true;
}

/**
 * Close the module
 * @return true if all resources are closed properly
 */
bool Torques::close(){
   outFile.close();
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
 * Handles command line commands to detect and perform actions
 * @return true unless quit is called
 */
bool Torques::respond(const Bottle& command, Bottle& reply){
  string helpMessage =  string(getName().c_str()) +
                        " commands are: \n" +
                         "help \n" +
                         "quit \n" +
                         "detect \n" +
                         "reload" +
                         "(where <n> is an float number) \n";
   reply.clear();

   if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
   }
   else if (command.get(0).asString()=="help") {
      cout << helpMessage;
      reply.addString("ok");
   }
   else if (command.get(0).asString()=="detect") {
       while(true) {
           detect();
       }
       //if(detect()) reply.addString("ok");
       //else reply.addString("fail to detect torques");
   }
   else if(command.get(0).asString()=="reload") {
       outFile.close();
       outFile.open(OUTFILENAME);
       reply.addString("ok");
   }
   else reply.addString("command not available");
   return true;
}

/**
 * Used to check the module state.
 * @return true if module can continue (always for this module)
 */
bool Torques::updateModule(){
   return true;
}

/**
 * To control the aproximate periodicity of updateModule() class.
 * @return the period
 */
double Torques::getPeriod(){
   return 0.1;
}
