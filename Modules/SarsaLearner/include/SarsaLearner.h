/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef ARMCTRLMODULE_H
#define ARMCTRLMODULE_H

#include <Configuration.h>
#include <Experiment.h>

/**
 * Yarp module used for running the Sarsa learning experiment.
 */
class SarsaLearner: public yarp::os::RFModule
{
protected:

    LearnerExperiment *ptrExp;

    // Global variables
    bool resetArm;
    bool runArm;    // true => arm should be moved through xd, od


    yarp::sig::Vector xd;
    yarp::sig::Vector od;

    yarp::os::Port handlerPort;      // a port to handle messages
    std::string handlerPortName;


public:
    virtual bool configure(yarp::os::ResourceFinder &rf);

    virtual bool interruptModule();
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    virtual bool close();

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};

#endif // ARMCTRLMODULE_H
