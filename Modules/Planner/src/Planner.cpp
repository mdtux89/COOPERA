/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "Planner.h"
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool Planner::configure(yarp::os::ResourceFinder &rf){
    string moduleName = rf.check("name",Value("Planner"),"module name (string)").asString();
    setName(moduleName.c_str());
    path = rf.check("path",Value("../Metric-FF-v2.1/ff"),"Planner executable file").asString();
    domain = rf.check("domain",Value("../resources/data/domain.pddl"),"Domain pddl file").asString();
    problem = rf.check("problem",Value("../resources/data/problem.pddl"),"Problem pddl file").asString();

    /*
     * attach a port of the same name as the module (prefixed with a /) to the module
     * so that messages received from the port are redirected to the respond method
     */
    string handlerPortName = "/" + moduleName;
    if (!handlerPort.open(handlerPortName.c_str())) {
       cout << getName() << ": Unable to open port " << handlerPortName << endl;
       return false;
    }
    if (!attach(handlerPort)) {
        return false;
    }
    myThread = new LoopThread();
    if(!myThread->start()) {
        return false;
    }
    return true ;
}

/**
 * Interrupt the module
 * @return true after closing the handler port
 */
bool Planner::interruptModule(){
    handlerPort.interrupt();
    return true;
}

/**
 * Close the module
 * @return true if all resources are closed properly
 */
bool Planner::close(){
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
bool Planner::respond(const Bottle& command, Bottle& reply){
    string helpMessage =  string(getName().c_str()) +
            " commands are: \n" +
             "help \n" +
             "quit \n" +
             "plan\n";

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
     else if(str=="plan"){
         //Output redirection for planner execution a few lines below..
         int stdoutCopy = dup(1);
         int fd;
         mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
         if((fd = open(PLANPATH, O_RDWR | O_CREAT, mode))==-1){
           reply.addString("fail");
           perror("open");
           return 1;
         }
         dup2(fd,STDOUT_FILENO);
         int pid = fork();
         if(pid==0){
            //Call the planner
            execl(path.c_str(), path.c_str(), "-o", domain.c_str(), "-f" , problem.c_str(), (char*)0);
         }
         else{
            //Now read the planner and display it
            dup2(stdoutCopy,STDOUT_FILENO);
            ifstream plan;
            plan.open(PLANPATH);
            string read;
            sleep(1);
            vector<string> actions;
            while(getline(plan,read)){
                if(read=="ff: found legal plan as follows"){
                    while(getline(plan,read)){
                        string tok;
                        stringstream ss(read);
                        getline(ss,tok,':');
                        if(tok=="plan cost"){
                            reply.addString("ok");
                            return true;
                        }
                        getline(ss,tok,':');
                        reply.addString(tok+" ");
                    }
                }
                else{
                    continue;
                }
            }
            reply.addString("fail");
        }



     }
     else reply.addString("command not available");
     return true;
}

/**
 * Used to check the module state.
 * @return true if module can continue (always for this module)
 */
bool Planner::updateModule(){
   return true;
}

/**
 * To control the aproximate periodicity of updateModule() class.
 * @return the period
 */
double Planner::getPeriod(){
   return 0.1;
}
