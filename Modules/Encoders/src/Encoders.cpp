/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "Encoders.h"
using namespace std;
using namespace yarp::dev;
using namespace yarp::os;

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool Encoders::configure(yarp::os::ResourceFinder &rf){

   string moduleName = rf.check("name",Value("Encoders"),"module name (string)").asString();
   setName(moduleName.c_str());

   //will form stem for port names
   robotName = rf.check("robot",Value("bioloid"),"Robot name (string)").asString();
   velocity = rf.check("velocity",Value(5),"Joint velocity (int)").asDouble();
   nParts = rf.find("njoints").asList()->size();
   encodersNameFile = rf.check("encoders",Value("../resources/data/encoders.txt"),"Encoders name file (string)").asString();
   drivers = new PolyDriver[nParts];
   threads = new MotionThread[nParts];
   outFile.open(OUTFILENAME);
   inFile.open(encodersNameFile.c_str());
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
bool Encoders::interruptModule(){
   handlerPort.interrupt();
   return true;
}

/**
 * Read encoders from all robot joints and save them in a file
 * @return true if the encoder values are read successfully
 */
bool Encoders::detect(){

   vector<double> val;
   for(int i=0;i<nParts;i++){ //for each robot parts

       //create the IPositionControl and IEncoders interfaces
       IPositionControl *position;
       IEncoders *encoders;
       if (!drivers[i].view(position) || !drivers[i].view(encoders)) {
         return false;
       }
       //get encoders for this robot part (one for each joint)
       for(int j=0;j<nJoints[i];j++){
           double v;
           int k=0;
           for(k=0;k<100;k++){
                encoders->getEncoder(j,&v);
                if(v>0.001){
                    val.push_back(v);
                    break;
                }
           }
           if(k==100) return false;
       }
   }

   //save encoders to file
   for(int j=0;j<(val.size()-1);j++){
       outFile << val[j] << ", ";
   }
   outFile << val[val.size()-1];
   outFile << endl;
   return true;
}

/**
 * Move all the motors accordingly to the encoders read from file
 * @return true if the movements are triggered successfully
 */
bool Encoders::perform(){
    string read;
    string tok;
    if(!getline(inFile,read)){
        return false;
    }
    vector<double> encoders;
    stringstream ss(read);

    for(int i=0;i<nParts;i++){ //for each robot parts
        encoders.clear();

        //split the line and push the values in a vector
        for(int j=0; j<nJoints[i]; j++){
            getline(ss,tok,',');
            encoders.push_back(atof(tok.c_str()));
        }

        //pass everything to the threads
        threads[i].set(&drivers[i],encoders,velocity);
        if(!threads[i].start()){
            return false;
        }
   }
   //wait threads
   for(int i=0;i<nParts;i++){
        threads[i].join();
   }
   return true;
}

/**
 * Close the module
 * @return true if all resources are closed properly
 */
bool Encoders::close(){
   outFile.close();
   inFile.close();
   for(int i=0;i<nParts;i++){
       if(!threads[i].stop()){
           return false;
       }
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
bool Encoders::respond(const Bottle& command, Bottle& reply){
  string helpMessage =  string(getName().c_str()) +
                        " commands are: \n" +
                         "help \n" +
                         "quit \n" +
                         "detect \n" +
                         "perform \n" +
                         "set vel <n>\n" +
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
   else if (command.get(0).asString()=="set") {
      if (command.get(1).asString()=="vel") {
         velocity = command.get(2).asDouble(); // new velocity
         reply.addString("ok");
      }
   }
   else if (command.get(0).asString()=="detect") {
       if(detect()) reply.addString("ok");
       else reply.addString("fail to detect encoders");
   }
   else if (command.get(0).asString()=="perform") {
       if(perform()) reply.addString("ok");
       else reply.addString("nothing to do");
   }
   else if(command.get(0).asString()=="reload") {
       outFile.close();
       inFile.close();
       outFile.open(OUTFILENAME);
       inFile.open(encodersNameFile.c_str());
       reply.addString("ok");
   }
   else reply.addString("command not available");
   return true;
}

/**
 * Used to check the module state.
 * @return true if module can continue (always for this module)
 */
bool Encoders::updateModule(){
   return true;
}

/**
 * To control the aproximate periodicity of updateModule() class.
 * @return the period
 */
double Encoders::getPeriod(){
   return 0.1;
}
