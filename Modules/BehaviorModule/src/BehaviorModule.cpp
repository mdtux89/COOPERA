/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "BehaviorModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool BehaviorModule::configure(yarp::os::ResourceFinder &rf){

   string moduleName = rf.check("name",Value("MotorsContoller"),"module name (string)").asString();
   setName(moduleName.c_str());

   //will form stem for port names
   robotName = rf.check("robot",Value("bioloid"),"Robot name (string)").asString();
   Info::nParts = rf.find("njoints").asList()->size();
   Info::drivers = new PolyDriver[Info::nParts];
   Info::velocity = rf.check("defaultvelocity",Value(5,"Default velocity (int)")).asInt();
   double batteryLimit = rf.check("batterylimit",Value(10.5,"Critical limit for the battery, only on Linux (double)")).asDouble()*10;
   string batteryAlarm = rf.check("batteryalarm",Value(""),"Alarm mp3 for battery (string)").asString();
   int batteryIndex = rf.check("batteryindex",Value(3,"Sensor's index for the battery(int)")).asInt();
   double accLimit = rf.check("acclimit",Value(0.6,"Critical limit for the accelerometer, for fall detection (double)")).asDouble();
   int accIndex = rf.check("accindex",Value(7,"Sensor's index for the accelerometer(int)")).asInt();
   Info::tiltIndex = rf.check("tiltindex",Value(5,"Sensor's index for the tilting(int)")).asInt();
   Info::motionWait = rf.check("motionwait",Value(3,"Time to wait for completion of each single motion during descent (int)")).asInt();
   Info::macroWait = rf.check("macrowait",Value(5,"Time to wait after an entire macrobehavior (int)")).asInt();
   Info::poseEstimation = rf.check("poseestimate",Value(false,"Whether to use pose estimation or not (bool)")).asBool();
   Info::pose = rf.check("pose",Value("facedown","(string)")).asString();
   csvFile = rf.check("csv",Value("csv.txt"),"Training set file per pose detection (string)").asString();
   Property config;
   string name = getName();
   Info::sensorPort->open("/"+name+"/read");
   Network::connect("/"+robotName+"/sensors","/"+name+"/read");

   Info::nTotalJoints = 0;
   for(int i=0;i<Info::nParts;i++){
       Info::nJoints->push_back(rf.find("njoints").asList()->get(i).asInt());
       partArray.push_back(rf.find("parts").asList()->get(i).asString());
       config.fromString("(device remote_controlboard) (local /local"+name+partArray[i]+") (remote /" +robotName+"/"+partArray[i]+") (carrier tcp)");
       if(!Info::drivers[i].open(config)){
           return false;
       }
       (*Info::parts)[partArray[i]] = i; //to map strings to indexes representing robot parts
       (*Info::starts)[partArray[i]] = Info::nTotalJoints;
       Info::nTotalJoints += rf.find("njoints").asList()->get(i).asInt();
   }

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
   string handlerPortName = "/" + name;
   if (!handlerPort.open(handlerPortName.c_str())) {
      cout << getName() << ": Unable to open port " << handlerPortName << endl;
      return false;
   }
   if(!attach(handlerPort)){
       return false;
   }
   myThread = new Guardian(robotName,batteryIndex,batteryLimit,batteryAlarm,accIndex,accLimit);
   if(!myThread->start()){
       return false;
   }
   pose = new Pose(csvFile);
   return true ;
}


/**
 * Interrupt the module
 * @return true after closing the handler port
 */
bool BehaviorModule::interruptModule(){
    handlerPort.interrupt();
    return true;
}

/**
 * Close the module
 * @return true if all resources are closed properly
 */
bool BehaviorModule::close(){
    handlerPort.close();
    if(!myThread->stop()) {
        return false;
    }
    return true;
}

/**
 * Handles command line commands to trigger the planner
 * @return true unless quit is called
 */
bool BehaviorModule::respond(const Bottle& command, Bottle& reply){
    string helpMessage =  string(getName().c_str()) +
            " commands are: \n" +
             "help \n" +
             "quit \n" +
             "learn \n" +
             "exploit \n" +
             "reset\n";

     reply.clear();
     string str = command.get(0).asString();
     if (str=="quit") {
          reply.addString("quitting");
          return false;
     }
     else if (str=="help") {
       cout << helpMessage;
       reply.addString("ok");
     }
     else if(str=="learn"){
        Evolution* evo = new Evolution(myThread);
        LearnerExperiment learner(evo,pose,true);
        learner.start();
        learner.stop();
     }
     else if(str=="exploit"){
        Evolution* evo = new Evolution(myThread);
        LearnerExperiment learner(evo,pose,false);
        learner.start();
        learner.stop();
     }
     else if (str == "reset") {
        system("../resources/data/reset.sh");
        reply.addString("ok");
     }
     else {
         reply.addString("command not available");
     }
     return true;
}

/**
 * Used to check the module state.
 * @return true if module can continue (always for this module)
 */
bool BehaviorModule::updateModule(){
   return true;
}

/**
 * To control the aproximate periodicity of updateModule() class.
 * @return the period
 */
double BehaviorModule::getPeriod(){
   return 0.1;
}
