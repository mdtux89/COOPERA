/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#include <MotorsController.h>
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

/**
 * Writes the log after (almost) each command
 * @return true if log updated successfully
 */
bool MotorsController::updateLog(){
    if(!keepLog) return true;
    vector<double> val;
    for(int i=0;i<shared.nParts;i++){ //for each robot parts
        IEncoders *encoders;
        if (!shared.drivers[i].view(encoders)) {
          return false;
        }
        //get encoders for this robot part (one for each joint)
        for(int j=0;j<shared.nJoints[i];j++){
            double v;
            int k;
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
        log << val[j] << ", ";
    }
    log << val[val.size()-1];
    log << endl;

    //Read the sensors' feedback ans save to file
    Bottle b;
    if(!sensorPort.read(b)){
        return false;
    }
    log << b.toString() << endl;
    return true;
}

/**
 * Read the torques used to perform pose estimation
 * @return detected torques
 */
vector<int> MotorsController::getData(){

   vector<int> v;
   for(int i=0;i<shared.nParts;i++){ //for each robot parts

       //create the IPositionControl and ITorqueControl interfaces
       IPositionControl *position;
       ITorqueControl *torque;
       if (!shared.drivers[i].view(position) || !shared.drivers[i].view(torque)) {
         break;
       }

       // two movements will be performed while getting torques
       double t1,t2;
       for(int j=0;j<shared.nJoints[i];j++){
           //first movement
           position->setRefSpeed(j,shared.velocity);
           position->relativeMove(j,shared.motionRange);
           for(int k=0; k<shared.waitTime*20;k++){
               torque->getTorque(j,&t1);
               Time::delay(0.05);
               if(abs(t1)>shared.torqueLimit){
                   position->stop(j);
                   if(t1 > 0) t1 = shared.torqueLimit;
                   else t1 = -shared.torqueLimit;
                   break;
               }
           }

           //back to initial position
           position->relativeMove(j,-shared.motionRange);
           Time::delay(0.1);

           //second movement
           position->relativeMove(j,-shared.motionRange);
           for(int k=0; k<shared.waitTime*20;k++){
               torque->getTorque(j,&t2);
               Time::delay(0.05);
               if(abs(t2)>shared.torqueLimit){
                   position->stop(j);
                   if(t2 > 0) t2 = shared.torqueLimit;
                   else t2 = -shared.torqueLimit;
                   break;
               }
           }

           //back to initial position
           position->relativeMove(j,shared.motionRange);
           Time::delay(0.1);

           //save values
           v.push_back(t1);
           v.push_back(t2);
       }
  }
  return v;
}

/**
 * Given the task passed as a parameter to the Sarsa Experiment read the proper initial encoders
 * @param task the chosen task
 * @return the proper encoders to be used in the experiment
 */
vector<int> MotorsController::encodersFromTask(string task){
     ifstream ifile;
     ifile.open("../resources/data/"+task+".txt");
     vector<int> encoders;
     string read;
     string tok;
     if(!getline(ifile,read)){
         return encoders;
     }
     stringstream ss(read);
     while(getline(ss,tok,',')){
         encoders.push_back(atoi(tok.c_str()));
     }
     ifile.close();
     return encoders;
}

/**
 * Run the Sarsa experiment for the chosen task.
 * We pass to the Sarsa algorithm the initial state to start with (initial) and the task needed to ensue the
 * relevant actions
 * @param task the chosen task
 */
bool MotorsController::runWithSarsa(string task){
    cout << "Sarsa running for task: " << task << endl;
    vector<int> initial = encodersFromTask(task);
    Experiment experiment(initial);
    if(!experiment.start()){
        return false;
    }
    if(!experiment.stop()){
        return false;
    }
    //get the plan
    vector<vector<int> > plan = experiment.getPlan();

    for(int i=0;i<plan.size();i++){ //for each step

         int c=0;
         for(int j=0;j<shared.nParts;j++){ //for each robot parts

             vector<double> encoders(plan[i].begin()+c,plan[i].begin() + c + shared.nJoints[j]);

             //pass everything to the threads
             shared.threads[j].set(&shared.drivers[j],encoders,shared.velocity);
             if(!shared.threads[j].start()){
                 return false;
             }
             c+= shared.nJoints[j];

        }

        //wait threads
        for(int j=0;j<shared.nParts;j++){
            shared.threads[j].join();
        }

        // wait for each motion to terminate
        Time::delay(1);
    }
    if(!updateLog()){
        return false;
    }
    return true;
}

 /**
  * Perform the movements by reading the sequence of encoders from file.
  * @param ifile the file used to read the encoders
  */
bool MotorsController::copy(ifstream *ifile){
    while(true){ //until file ends

        string read;
        string tok;
        if(!getline(*ifile,read)){ //read one line
            if(!updateLog()){
                return false;
            }
            return true;
        }
        stringstream ss(read);
        vector<double> encoders;

        for(int i=0;i<shared.nParts;i++){ //for each robot parts
            encoders.clear();
            //split the line and push the values in a vector
            for(int j=0; j<shared.nJoints[i]; j++){
                getline(ss,tok,',');
                encoders.push_back(atof(tok.c_str()));
            }

            //pass everything to the threads
            shared.threads[i].set(&shared.drivers[i],encoders,shared.velocity);
            if(!shared.threads[i].start()){
                return false;
            }
       }

       //wait threads
       for(int i=0;i<shared.nParts;i++){
           shared.threads[i].join();
       }

       // wait for each motion to terminate
       Time::delay(shared.joinTime);
    }
    return true;
}

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool MotorsController::configure(yarp::os::ResourceFinder &rf){
   string moduleName = rf.check("name",Value("MotorsContoller"),"module name (string)").asString();
   setName(moduleName.c_str());

   //will form stem for port names
   robotName = rf.check("robot",Value("bioloid"),"Robot name (string)").asString();
   shared.nParts = rf.find("njoints").asList()->size();
   shared.velocity = rf.check("velocity",Value(5),"Joint shared.velocity (double)").asDouble();
   shared.torqueLimit = rf.check("torqueLimit",Value(400),"Absolute Torque limit (double)").asDouble();
   shared.waitTime = rf.check("waitTime",Value(1.5),"Time to wait for each movement in seconds (int)").asInt();
   shared.joinTime = rf.check("jointime",Value(5),"Time to wait to let end the movements(double)").asDouble();
   shared.motionRange = rf.check("motionRange",Value(10),"Range of motion in degree (int)").asInt();
   csvFile = rf.check("csv",Value("csv.txt"),"Training set file per pose detection (string)").asString();
   keepLog = rf.check("keeplog",Value(true),"Whether to use or not a log file (bool)").asBool();
   logFile = rf.check("logfile",Value("log.txt"),"Log file (string)").asString();
   int limit = (int) (rf.check("batterylimit",Value(0,"Critical limit for the battery, only on Linux (double)")).asDouble()*10);
   int index = rf.check("batteryindex",Value(3,"Sensor's index for the battery(int)")).asInt();
   shared.drivers = new PolyDriver[shared.nParts];
   shared.threads = new MotionThread[shared.nParts];
   Property config;
   log.open(logFile);
   string name = getName();
   sensorPort.open("/"+name+"/read");
   Network::connect("/"+robotName+"/sensors","/"+name+"/read");

   for(int i=0;i<shared.nParts;i++){
       shared.nJoints.push_back(rf.find("njoints").asList()->get(i).asInt());
       partArray.push_back(rf.find("parts").asList()->get(i).asString());
       config.fromString("(device remote_controlboard) (local /local"+name+partArray[i]+") (remote /" +robotName+"/"+partArray[i]+") (carrier tcp)");
       if(!shared.drivers[i].open(config)){
           return false;
       }
       MotorsController::shared.parts[partArray[i]] = i; //to map strings to indexes representing robot parts
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
   pose = new Pose(csvFile, shared);
   myThread = new LoopThread(robotName,index,limit);
   if(!myThread->start()){
       return false;
   }
   if(!updateLog()){
       return false;
   }
   return true ;
}

/**
 * Interrupt the module
 * @return true after closing the handler port
 */
bool MotorsController::interruptModule(){
   handlerPort.interrupt();
   return true;
}

/**
 * Close the module
 * @return true if all resources are closed properly
 */
bool MotorsController::close(){
   for(int i=0;i<shared.nParts;i++){
       if(!shared.drivers[i].close()){
           return false;
       }
       if(!shared.threads[i].stop()){
           return false;
       }
   }
   handlerPort.close();
   if(!myThread->stop()){
       return false;
   }
   log.close();
   sensorPort.close();
   return true;
}

/**
 * Handles command line commands to interact with the motors and run procedures
 * @return true unless quit is called
 */
bool MotorsController::respond(const Bottle& command, Bottle& reply){

  string helpMessage =  string(getName().c_str()) +
          " commands are: \n" +
           "help \n" +
           "quit \n" +
           "home \n" +
           "sideleft \n" +
           "sideright \n" +
           "faceup \n" +
           "facedown \n" +
           "standup <from>" +
           "(where <from> is a string) \n" +
           "set vel <n> " +
           "(where <n> is an float number) \n"
           "set pos <part> <id> <n> " +
           "(where <part> is a string, " +
           "<id> is an int number, " +
           "<n> is a float number) \n" +
           "set dpos <part> <id> <n> " +
           "(where <part> is a string, " +
           "<id> is an int number, " +
           "<n> is a float number) \n" +
           "get pos <part> <id> <n> " +
           "(where <part> is a string, " +
           "<id> is an int number) \n" +
           "get torque <part> <id> <n> " +
           "(where <part> is a string, " +
           "<id> is an int number) \n" +
           "get pose \n" +
           "detectpose \n" +
           "turn\n" +
           "demo\n";

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

   else if (str=="set") {
      if (command.get(1).asString()=="vel") {
         shared.velocity = command.get(2).asInt(); // new shared.velocity
         reply.addString("ok");
      }
      else if (command.get(1).asString()=="pos") {
         string part = command.get(2).asString();
         if(shared.parts.find(part) == shared.parts.end()){
             reply.addString("fail");
         }
         else{
             int id = command.get(3).asInt(); // joint id
             double position = command.get(4).asInt(); // new position

             //set new position to joint
             IPositionControl *pos;
             shared.drivers[shared.parts[part]].view(pos);
             pos->setRefSpeed(id,this->shared.velocity);
             pos->positionMove(id,position);
             if(!updateLog()){
                 reply.addString("update log failed");
             }
             else reply.addString("ok");
         }
      }
      else if (command.get(1).asString()=="dpos") {
         string part = command.get(2).asString();
         if(shared.parts.find(part) == shared.parts.end()){
             reply.addString("fail");
         }
         else{
             int id = command.get(3).asInt(); // joint id
             double delta = command.get(4).asInt(); // relative position

             //set new position to joint
             IPositionControl *pos;
             shared.drivers[shared.parts[part]].view(pos);
             pos->setRefSpeed(id,this->shared.velocity);
             pos->relativeMove(id,delta);
             if(!updateLog()){
                 reply.addString("update log failed");
             }
             else reply.addString("ok");
         }
      }
      else reply.addString("command not available");
   }

   else if (str=="get") {

       if (command.get(1).asString()=="pos") {
           string part = command.get(2).asString();
           int id = command.get(3).asInt(); // joint id
           //get joint encoder
           IEncoders *enc;
           double v=0;
           shared.drivers[shared.parts[part]].view(enc);
           enc->getEncoder(id,&v);
           reply.addDouble(v);
       }
       else if(command.get(1).asString()=="torque") {
               string part = command.get(2).asString();
               int id = command.get(3).asInt(); // joint id
               //get joint encoder
               ITorqueControl *trq;
               double v=0;
               shared.drivers[shared.parts[part]].view(trq);
               trq->getTorque(id,&v);
               reply.addDouble(v);
       }
       else if (command.get(1).asString()=="pose") {
            reply.addString("Pose: " + pose->pose);
       }
       else reply.addString("command not available");
   }

   else if(str=="detectpose"){
       pose->classify(getData());
       reply.addString("Detected: " + pose->pose);
       pose->test();
       reply.addString("Pose: " + pose->pose);
   }

   else if(str=="turn"){
       string from = command.get(1).asString();
       if(from == "sideright" || from == "sideleft"){
           pose->pose = from;
       }
       if(pose->pose == "sideright" || pose->pose == "sideleft"){
           //Run Reinforcement Learning algorithm
           if(!runWithSarsa(pose->pose)){
               reply.addString("fail");
           }
           pose->pose = "faceup";
           pose->test();
           reply.addString("Pose: " + pose->pose);
           if(!updateLog()){
               reply.addString("update log failed");
           }
       }
       else reply.addString("turning not possible");
   }

   else if(str=="home" || str=="sideleft" || str=="sideright" || str=="facedown" || str=="faceup" || str=="standup") {
       string from = command.get(1).asString();
       if(str=="standup"){
            if(from=="facedown" || from=="faceup"){
                pose->pose = from;
            }
            if(pose->pose=="facedown" || pose->pose=="faceup"){
                str = str +  pose->pose;
            }
            else {
                reply.addString("standing up not possible");
                return true;
            }
       }
       //open files containing encoders for home position
       ifstream *ifile = new ifstream;
       ifile->open("../resources/data/"+str+".txt");
       //use the copy encoder procedure
       if(!copy(ifile)){
           reply.addString("fail");
       }
       if(!updateLog()){
           reply.addString("update log failed");
       }
       if(str=="home" || str=="standup" || str=="standupfacedown" || str=="standupfaceup"){
           pose->pose = "standup";
       }
       else{
            pose->pose = str;
            pose->test();
       }
       reply.addString("Pose: " + pose->pose);
       ifile->close();

   }

   else if(str=="demo"){
       //Pose estimation
       reply.addString("Classification...");
       pose->classify(getData());
       pose->test();
       reply.addString("Pose: " + pose->pose);

       //Turn
       if(pose->pose=="sideright" || pose->pose=="sideleft"){
           reply.addString("Turning...");
           if(!runWithSarsa(pose->pose)){
               reply.addString("fail");
           }
           pose->pose = "faceup";
           pose->test();
       }

       if(pose->pose=="faceup" || pose->pose=="facedown"){
           //Stand up
           reply.addString("Standing up...");
           ifstream* ifile = new ifstream;
           ifile->open("../resources/data/standup"+pose->pose+".txt");
           if(!copy(ifile)){
               reply.addString("fail");
           }
           if(!updateLog()){
               reply.addString("update log failed");
           }
           pose->pose = "standup";
           pose->test();
           reply.addString("Pose: " + pose->pose);
           ifile->close();
       }
   }
   else reply.addString("command not available");

   return true;
}

/**
 * Used to check the module state.
 * @return true if module can continue (always for this module)
 */
bool MotorsController::updateModule(){
   return true;
}

/**
 * To control the aproximate periodicity of updateModule() class.
 * @return the period
 */
double MotorsController::getPeriod(){
   return 0.1;
}
