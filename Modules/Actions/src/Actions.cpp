/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "Actions.h"
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool Actions::configure(yarp::os::ResourceFinder &rf){

   string moduleName = rf.check("name",Value("Actions"),"module name (string)").asString();
   setName(moduleName.c_str());

   //will form stem for port names
   robotName = rf.check("robot",Value("bioloid"),"Robot name (string)").asString();
   velocity = rf.check("velocity",Value(5),"Joint velocity (int)").asDouble();
   nParts = rf.find("njoints").asList()->size();
   drivers = new PolyDriver[nParts];
   outFile.open(OUTFILENAME);
   fileName = rf.check("../resources/data/actions",Value("actions.txt"),"Actions file name (string)").asString();
   inFile.open(fileName.c_str());
   Property config;
   string name = getName();
   for(int i=0;i<nParts;i++){
       nJoints.push_back(rf.find("njoints").asList()->get(i).asInt());
       string part = rf.find("parts").asList()->get(i).asString();
       partArray.push_back(part);
       config.fromString("(device remote_controlboard) (local /local"+name+part+") (remote /" +robotName+"/"+part+") (carrier tcp)");
       if(!drivers[i].open(config)){
           return false;
       }
       Actions::parts[partArray[i]] = i; //to map index to string representing robot parts
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

   old.resize(nParts);
   for(int i=0;i<nParts;i++){
       old[i].resize(nJoints[i]);
   }
   count=0;

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
bool Actions::interruptModule(){
   handlerPort.interrupt();
   return true;
}

/**
 * Perform one action read from file
 * @return true if the action is performed successfully
 */
bool Actions::perform(){
    string read;

    while(true){ //until action ends (because of the return below)

        if(!getline(inFile,read)){ //read one line until end of file..
            return true;
        }
        if(read == "--"){ // ..or action delimiter is found
            return true;
        }
        stringstream ss(read);

        //split the line in part of the robot, joint id and polarity of the action
        string part,s_id,s_pol;
        getline(ss,part,',');
        getline(ss,s_id,',');
        getline(ss,s_pol,',');
        int id = atoi(s_id.c_str());
        int pol = atoi(s_pol.c_str());

        //create the IPositionControl interface
        IPositionControl *position;
        if (!drivers[parts[part]].view(position)) {
          return false;
        }
        if(!position->setRefSpeed(id,this->velocity)){
            return false;
        }
        if(!position->relativeMove(id,pol)){
            return false;
        }
        Time::delay(0.1);
    }
}

/**
 * Detect movements from all robot joints and save them as a unique action in a file
 * @return true if the action is correctly detected
 */
bool Actions::detect(){

   count++;
   for(int i=0;i<nParts;i++){ //for each robot parts
       //create the IPositionControl and IEncoders interfaces
       IPositionControl *position;
       IEncoders *encoders;
       if (!drivers[i].view(position) || !drivers[i].view(encoders)) {
         return false;
       }

       //get encoders for this robot part (one for each joint)
       vector<double> val;
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

       //detect action and save to file
       for(int j=0;j<nJoints[i];j++){
           double diff = val[j]-old[i][j];
           old[i][j] = val[j];
           if(count>1 && abs(diff)>MINACTION){
               outFile << partArray[i] << ", " << j << ", " << ((int) diff/MINACTION)*MINACTION << endl;
           }
       }
  }
  outFile << "--" << endl;
  return true;
}

/**
 * Close the module
 * @return true if all resources are closed properly
 */
bool Actions::close(){
   for(int i=0;i<nParts;i++){
       if(!drivers[i].close()){
           return false;
       }
   }
   outFile.close();
   inFile.close();
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
bool Actions::respond(const Bottle& command, Bottle& reply){
  string helpMessage =  string(getName().c_str()) +
                        " commands are: \n" +
                         "help \n" +
                         "quit \n" +
                         "set vel <n>\n" +
                         "detect \n" +
                         "perform \n" +
                         "reload \n";
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
      if(detect()) reply.addString("ok");
      else reply.addString("failed to detect actions");
   }
   else if (command.get(0).asString()=="perform") {
      if(perform()) reply.addString("ok");
      else reply.addString("failed or no new actions left");
   }
   else if (command.get(0).asString()=="set") {
      if (command.get(1).asString()=="vel") {
         velocity = command.get(2).asDouble(); // new velocity
         reply.addString("ok");
      }
   }
   else if(command.get(0).asString()=="reload") {
       outFile.close();
       inFile.close();
       outFile.open(OUTFILENAME);
       inFile.open(("../resources/data/" + fileName).c_str());
       reply.addString("ok");
   }
   else reply.addString("command not available");
   return true;
}

/**
 * Used to check the module state.
 * @return true if module can continue (always for this module)
 */
bool Actions::updateModule(){
   return true;
}

/**
 * To control the aproximate periodicity of updateModule() class.
 * @return the period
 */
double Actions::getPeriod(){
   return 0.1;
}
