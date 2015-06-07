/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#include <SarsaLearner.h>

using namespace std;
using namespace yarp::os;

/**
 * Process all parameters from both command-line and ini file and open device accordingly
 * @return true if the module is correctly configured
 */
bool SarsaLearner::configure(yarp::os::ResourceFinder &rf)
{
    handlerPortName =  "";
    handlerPortName += getName();
    handlerPortName += "/cntrlPort";

    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << "\n";
        return false;
    }
    attach(handlerPort);
    ptrExp = new LearnerExperiment();
    ptrExp->start();

    return true;
}

/**
 * Close the module
 * @return true after closing all resources
 */
bool SarsaLearner::close()
{
    std::cout<<"Closing module...\n";
    handlerPort.close();
    delete ptrExp;
    std::cout<<"Done with closing module!\n";
    return true;
}

bool SarsaLearner::interruptModule()
{
    handlerPort.interrupt();
    return true;
}

/**
 * Handles command line commands to interact with the motors and run procedures
 * @return true unless quit is called
 */
bool SarsaLearner::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    string helpMessage =  string(getName().c_str()) +
                        " commands are: \n" +
                        "help \n" +
                        "quit \n" ;

    reply.clear();

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }

    return true;
}
