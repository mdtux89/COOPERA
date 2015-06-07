/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "Sensors.h"
using namespace std;
using namespace yarp::dev;
using namespace yarp::os;

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool Spatials::configure(yarp::os::ResourceFinder &rf){

   string moduleName = rf.check("name",Value("Spatials"),"module name (string)").asString();
   setName(moduleName.c_str());

   //will form stem for port names
   robotName = rf.check("robot",Value("bioloid"),"Robot name (string)").asString();
   average = rf.check("average",Value(30),"How many samples to average (int)").asInt();
   nSensors = rf.check("nsensors",Value(0),"How many sensors (int)").asInt();
   outFile.open(OUTFILENAME);
   Property config;
   string name = getName();
   sensorPort.open("/"+name+"/read");
   Network::connect("/"+robotName+"/sensors","/"+name+"/read");

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
bool Spatials::interruptModule(){
   handlerPort.interrupt();
   return true;
}

/**
 * Read spatials feedback and save them in a file
 * @return true if the values are read successfully
 */
bool Spatials::detect(){
   Bottle b;
   vector<double> sens;
   for (int i=0; i < nSensors; i++) {
       sens.push_back(0);
   }
   string tok;
   for (int i = 0; i<average; i++) {
       if(!sensorPort.read(b)){
           return false;
       }
       stringstream ss(b.toString());
       int k = 0;
       while(getline(ss,tok,' ')){
           sens[k] = sens[k] + atof(tok.c_str());
           k++;
       }
   }
   for (int i = 0; i < sens.size(); i++) {
       outFile << (sens[i]/average) << " ";
   }
   outFile << endl;
   return true;
}

/**
 * Close the module
 * @return true if all resources are closed properly
 */
bool Spatials::close(){
   outFile.close();
   handlerPort.close();
   sensorPort.close();
   if(!myThread->stop()){
       return false;
   }
   return true;
}

/**
 * Handles command line commands to detect and perform actions
 * @return true unless quit is called
 */
bool Spatials::respond(const Bottle& command, Bottle& reply){
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
	while(true){
       if(detect()) reply.addString("ok");
       else reply.addString("fail to detect spatials feeback");
	}
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
bool Spatials::updateModule(){
   return true;
}

/**
 * To control the aproximate periodicity of updateModule() class.
 * @return the period
 */
double Spatials::getPeriod(){
   return 0.1;
}
